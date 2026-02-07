#include <chrono>
#include <memory>
#include <cmath>
#include <map>
#include <vector>
#include <algorithm>
#include <random>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "irobot_create_msgs/msg/hazard_detection_vector.hpp"
#include "tf2/utils.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

inline double rad2deg(double rad) { return rad * 180.0 / M_PI; }
inline double deg2rad(double deg) { return deg * M_PI / 180.0; }


class Contest1Node : public rclcpp::Node
{
public:
    Contest1Node()
        : Node("contest1_node")
    {
        // Initialize publisher for velocity commands
        vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);

        //Initialize subscriber for laser scan data (LiDAR)
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&Contest1Node::laserCallback, this, std::placeholders::_1));
        
        //Initialize subscriber for hazard detection data
        hazard_sub_ = this->create_subscription<irobot_create_msgs::msg::HazardDetectionVector>(
            "/hazard_detection", rclcpp::SensorDataQoS(),
            std::bind(&Contest1Node::hazardCallback, this, std::placeholders::_1));
        
        //Initialize subscriber for odometry data
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::SensorDataQoS(),
            std::bind(&Contest1Node::odomCallback, this, std::placeholders::_1));

        // Timer for main control loop at 10 Hz
        timer_ = this->create_wall_timer(
            100ms, std::bind(&Contest1Node::controlLoop, this));

        // Obtain a random seed from the system clock
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();

        // Initialize the Mersenne Twister pseudo-random number generator
        std::mt19937 gen(seed); 

        // Initialize variables
        start_time_ = this->now(); //define start time
        startup = true; //boolean to indicate control loop is executing program to startup before wall following
        callstartupRoutine = true;  // boolean to indicate whether to call startup routines
        angular_ = 0.0; //initialize angular velocity
        linear_ = 0.0;  //initialize linear velocity
        pos_x_ = 0.0; //initialize x position
        pos_y_ = 0.0; // initialize y position
        yaw_ = 0.0; //initialize yaw
        minLaserDist_ = std::numeric_limits<float>::infinity(); //initialize minimum laser distance to infinity (ensures all other measurements will be smaller)
        minLaserDist_idx_ = 0; //initialize index of minimum laser distance measurement
        nLasers_ = 0; //initilaize counter for number of laser data points
        desiredNLasers_ = 0; // part of the default code: for setting up minLaserDistance in laserCallback - sets up number of laser measurements to consider on either side of the center front laser measurement for minimum laser distance calculation (set as however many lasers are within the desiredAngle range)
        desiredAngle_ = 90; // CHANGED FROM 5 TO 90- defines the angle range on either side of the front center laser measurement to consider for minimum laser distance calculation (returns the min laser distance in this range) - set to 90 degrees by default so that we consider the full front half of the robot for minimum laser distance calculation
        minimumFrontDistance_ = 0.5; // minimum distance the robot can have to an obstacle in front before it is considered too close and needs to turn away
        minimumObstacleDistance = 0.4; //minimum distance the robot can have to an obstacle in any direction before it is considered too close and needs to turn away
        isTurning = false; //boolean to indicate whether the robot is currently turning
        enterBumperHandling = false; //boolean to indicate whether robot should enter bumper handling routine
        backup = false; // boolean to indicate whether robot should be in backup routine within bumper handling
        start_yaw_ = 0.0; // set to current yaw when setPositions() is called - used for some handler routines (e.g. startup routine)
        start_pos_x_ = 0.0; // set to current X position when setPositions() is called - used for some handler routines (e.g. startup routine)
        start_pos_y_ = 0.0; // set to current Y position when setPositions() is called - used for some handler routines (e.g. startup routine)
        anyBumperPressed = false; //boolean to indicate whether any bumper is pressed
        front_distance_ = std::numeric_limits<float>::infinity(); //initiaze front distance to infinity, all other measurements will be smaller
        right_distance_ = std::numeric_limits<float>::infinity(); //initiaze right distance to infinity, all other measurements will be smaller
        left_distance_ = std::numeric_limits<float>::infinity(); //initiaze left distance to infinity, all other measurements will be smaller
        back_distance_ = std::numeric_limits<float>::infinity();// initiaze back distance to infinity, all other measurements will be smaller
        front_idx_ = 0; //find index of front obstacle distance measurement
        back_idx_ = 0; //find index of back obstacle distance measurement
        left_idx_ = 0; //find index of left obstacle distance measurement
        right_idx_ = 0; //find index of right obstacle distance measurement

        target_yaw = 0.0; // initialize target yaw for random routine
        randomTurn = false; // initialize randomTurn boolean for random routine

        // Initialize bumper states to false 
        bumpers_["bump_front_left"] = false;
        bumpers_["bump_front_center"] = false;
        bumpers_["bump_front_right"] = false;
        bumpers_["bump_left"] = false;
        bumpers_["bump_right"] = false;
        
        // Initializes node to execute program for contest 1
        RCLCPP_INFO(this->get_logger(), "Contest 1 node initialized. Running for 480 seconds.");
    }

private:
    #pragma region Callbacks
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        // implement your code here
        nLasers_ = (scan->angle_max - scan-> angle_min) / scan->angle_increment;
        laserRange_ = scan->ranges;
        desiredNLasers_ = deg2rad(desiredAngle_) / scan->angle_increment; // number of lasers to consider on each side
        //RCLCPP_INFO(this->get_logger(), "Size of laser scan array: %d, and size of offset: %d", nLasers_, desiredNLasers_);
        //RCLCPP_INFO(this->get_logger(), "angle max %.2f, angle min %.2f, range_min %.2f, range_max %.2f", scan->angle_max, scan->angle_min, scan->range_min, scan->range_max);

        /* THIS IS PART OF THE DEFAULT CODE, BUT IT'S BECOME UNNECCESSARY BECAUSE FRONT_IDX CALCULATION IS EQUIVALENT TO FRONT_IDX_ CALCULATION - not sure if we should remove it though, because it is apart of the default code.
        // Find minimum laser distance within +/- desiredAngle from front center
        float laser_offset = deg2rad(-90.0);
        uint32_t front_idx = (laser_offset - scan->angle_min) / scan->angle_increment;   
        */

        front_idx_ = nLasers_ / 4; //find index of front obstacle distance measurement
        back_idx_ = nLasers_ * 3/4 ; //find index of back obstacle distance measurement
        left_idx_ = nLasers_ / 2; //find index of left obstacle distance measurement
        right_idx_ = 0; //find index of right obstacle distance measurement


        minLaserDist_ = std::numeric_limits<float>::infinity(); //find the closest obstacle to the current position of the robot FROM front_idx_ - desiredNLasers_ to front_idx_ + desiredNLasers_ (NOT FULL SCAN AROUND ROBOT) 
        if (deg2rad(desiredAngle_) < scan->angle_max &&deg2rad(desiredAngle_) > scan->angle_min) { 
            for (uint32_t laser_idx = front_idx_ - desiredNLasers_; laser_idx < front_idx_ + desiredNLasers_; ++laser_idx) {
                minLaserDist_ = std::min(minLaserDist_, laserRange_[laser_idx]); // find minimum laser distance within desired angle range
                minLaserDist_idx_ = laser_idx; // index of closest obstacle within the desired angle range
            }
        } else {
            for (uint32_t laser_idx = 0; laser_idx < nLasers_; ++laser_idx) {
                minLaserDist_ = std::min(minLaserDist_, laserRange_[laser_idx]); // find minimum laser distance within the full scan around the robot (if desired angle range is larger than the total scan range)
                minLaserDist_idx_ = laser_idx; // index of closest obstacle in the full scan
            }
        }

        front_distance_ = laserRange_[front_idx_];  //extract distance of obstacle in front of robot
        right_distance_ = laserRange_[right_idx_];  //extract distance of obstacle to right of robot
        left_distance_ = laserRange_[left_idx_];  //extract distance of obstacle to left of robot
        back_distance_ = laserRange_[back_idx_]; //extract distance of obstacle behind robot

        createLasersArray(); // Define filteredLaserRange array with values from laserRange_ above 20cm

        RCLCPP_INFO(this->get_logger(), "front_distance_: %f, right_distance_: %f, left_distance: %f, back_distance: %f" , front_distance_, right_distance_, left_distance_, back_distance_);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
    {
        //extract (current?) position of the robot

        pos_x_= odom->pose.pose.position.x; 
        pos_y_= odom->pose.pose.position.y;

        //extract yaw from quaternion using tf2
        yaw_ = tf2::getYaw(odom->pose.pose.orientation);        //yaw in radians
        //RCLCPP_INFO(this -> get_logger(),"Position: (%.2f, %.2f), orientation: %f rad or %f deg", pos_x_, pos_y_, yaw_, rad2deg(yaw_));
    }

    void hazardCallback(const irobot_create_msgs::msg::HazardDetectionVector::SharedPtr hazard_vector)
    {
        // Reset all bumpers to released state
        for (auto& [key, val] : bumpers_) {
            val = false;
        }
        
        // Update bumper states based on current detections
        for (const auto& detection : hazard_vector->detections) {
            // HazardDetection types include: BUMP, CLIFF, STALL, WHEEL_DROP, etc.
            // Type 1 corresponds to BUMP
            if (detection.type == irobot_create_msgs::msg::HazardDetection::BUMP) {
                bumpers_[detection.header.frame_id] = true;
                RCLCPP_INFO(this->get_logger(), "Bumper pressed: %s",
                            detection.header.frame_id.c_str());  
            }
        }
    }
    #pragma endregion Callbacks
    
    #pragma region Routines
    void startupRoutine()
    {
        RCLCPP_INFO(this -> get_logger(),"WE'RE IN STARTUP ROUTINE");
        // Startup routine to orient robot towards direction of maximum laser distance and drive to within 50cm of an obstacle

        // Turn robot towards furthest wall
        float distanceDifference = abs(front_distance_ - max_element); // calculate distance difference - max_element defined in control loop
        if (distanceDifference > 0.01)  // if not yet facing furthest wall
        { 
            RCLCPP_INFO(this->get_logger(), "turning to face furthest wall");
            linear_ = 0.0;
            angular_ = turnClockwise ? 0.1 : -0.1; // turn in specified direction - turnClockwise defined in control loop
        }
        else if (front_distance_ > 0.5) // drive facing furthest wall until wall is within 50cm or less
        {
            RCLCPP_INFO(this->get_logger(), "facing furthest wall, moving to approach");
            linear_ = 0.2;
            angular_ = 0.0; 
        }
        else { callstartupRoutine = false; } // Exit startup routine
        
        // Move forward until minimum laser distance is 50cm or less
        if (front_distance_ > minimumFrontDistance_)
        {
            RCLCPP_INFO(this -> get_logger(),"drive forwards");
            linear_ = 0.25;
            angular_ = 0.0;
        }
        else { callstartupRoutine = false; } // Exit startup routine
 
        return;
    }

    void wallFollowingRoutine()
    {
        /* Wall following routine to follow the right wall at a set distance */
        RCLCPP_INFO(this -> get_logger(),"front_distance: %.2f , left_distance: %.2f, right_distance: %.2f", front_distance_, left_distance_, right_distance_);

        // Determine if robot is close to an obstacle in front and needs to turn
        bool _turnClockwise = false; // true = turn right, false = turn left
        if (front_distance_ <= 0.5 && !isTurning)
        {
            if (left_distance_ <= 0.5 && right_distance_ > 0.01) { _turnClockwise = true; } // turn right if left wall is also close
            isTurning = true;
        }
        
        if (!isTurning) // robot is clear ahead
        {
            linear_ = 0.2;
            angular_ = 0.0;
        }
        else { turnRobot(_turnClockwise); } // robot is turning

        return; 
    }

    void randomRoutine()
    {
        // enter routine
        // check if obstalce present
        // if yes, set random target angle, and turn towards until reached
        // if no, move forward
        
        //RCLCPP_INFO(this->get_logger(), "Randomly turning to angle: %f radians", random_angle);

        if (front_distance_ > 0.5 && !randomTurn) 
        {
            linear_ = 0.2;
            angular_ = 0.0;
            return;
        }
        else
        {
            static std::default_random_engine generator;
            static std::uniform_real_distribution<double> distribution(-M_PI, M_PI);
            double random_angle = distribution(generator);
            target_yaw = normalizeAngle(yaw_  + random_angle);
            randomTurn = true;
        }

        if (randomTurn)
        {
            angular_ = (normalizeAngle(target_yaw - yaw_) > 0) ? 0.2 : -0.2; // Set angular velocity to turn towards the target yaw
            linear_ = 0.0;

            if (abs(normalizeAngle(target_yaw - yaw_)) < 0.1) 
            {
                angular_ = 0.0;
                randomTurn = false;
            }
        }
    }

    #pragma endregion Routines

    #pragma region Utilities

    // functions that are called in the routines to perform specific tasks (e.g. turning robot, handling bumper)
   

    void turnRobot(bool _turnClockwise)
    {
        // Turn robot in specified direction until min side laser distance matches min laser distance from filteredLaserRange_
        RCLCPP_INFO(this->get_logger(), "Turning robot %s", _turnClockwise ? "clockwise" : "counter-clockwise");

        // Find minimum distance from filtered laser ranges
        float min_element = *std::min_element(begin(filteredLaserRange_), end(filteredLaserRange_)); // find minimum distance from filtered laser ranges
        float distanceDifference = _turnClockwise ? abs(left_distance_ - min_element) : abs(right_distance_ - min_element); // calculate distance difference based on turn direction

        if (distanceDifference > 0.01)  // if not yet reached target rotation
        { 
            linear_ = 0.0;
            angular_ = _turnClockwise ? -0.1 : 0.1; // turn in specified direction
        }
        else // reached target rotation
        {
            RCLCPP_INFO(this->get_logger(), "Reached 90 degrees, moving to final stop"); 
            linear_ = 0.2;
            angular_ = 0.0; 
            isTurning = false;
        }

        return;
    }

    void bumperPressedHandling() 
    {
        // Handling bumper pressed event
        RCLCPP_INFO(this -> get_logger(),"Bumper pressed! Handling...");

        // you want the robot to move back until the distance traveled is greater than a certain threshold.
        //      To achieve this, we will use a backup boolean set to true when we enter the bumper handling routine, and then set to false once we have backed up enough
        //      Because we don't want to boolean to be overwritten with every iteration, we should make it a global variable. In moveBack(), we can use this bool instead of the state_ variable.
        //      Once we have backed up enough, backup is set to false (in moveBack()). Then, we enter a turning routine that executes until the robot has turned 45 degrees away from obstacle.
        //      If we want to be more specific with the turning, we can choose the angle we turn by based on which bumper was pressed, but I don't think it's neccessary.
        //      Once the robot has turned 45 degrees, we exit the bumper handling routine and return to whatever routine we we in before (startup, wallFollowing or random)

        if (backup)
        {
            moveBack();
            RCLCPP_INFO(this -> get_logger(),"backup...");
        }
        else if (abs(yaw_ - start_yaw_) < deg2rad(90)) // Turn the robot 45 degrees away from obstacle
        {
            //RCLCPP_INFO(this -> get_logger(),"yaw_ = %f, start_yaw_ = %f", yaw_, start_yaw_);
            if(pressed_bumper == "bump_front_left" || pressed_bumper == "bump_left" || pressed_bumper == "bump_front_center")
            {
                RCLCPP_INFO(this -> get_logger(),"Left side or front bumper was pressed - turn right");
                linear_ = 0.0;
                angular_ = -0.2; // turn right
            }
            else // if (pressed_bumper == "bump_right" or "bump_front_right") (only remaining options)
            {
                RCLCPP_INFO(this -> get_logger(),"Right side bumper was pressed - turn left");
                linear_ = 0.0;
                angular_ = 0.2; // turn left
            }
        }
        else // if robot has turned 45 degrees away from obstacle, exit bumper handling routine and return to startup/wall following routine
        {
            RCLCPP_INFO(this -> get_logger(),"yaw_ = %f, start_yaw_ = %f", yaw_, start_yaw_);
            RCLCPP_INFO(this -> get_logger(),"Finished turning, exiting bumper handling routine");
            enterBumperHandling = false; // exit bumper handling routine
        }
        return;
    }

    void moveBack()
    {
        // Move robot back until it has traveled a certain distance from the position where the bumper was pressed
        //     note that the position where the bumper was pressed is saved as start_pos_x_ and start_pos_y_ when setCurrentPositions() is called in bumperPressedHandling() 
        //     - this is updated/set at the moment the bumper is pressed, and is not updated until the next time a bumper is pressed, so it will save the position of the robot at the moment the bumper was pressed
        
        double distance_traveled = std::sqrt(
            std::pow(pos_x_ - start_pos_x_, 2) + std::pow(pos_y_ - start_pos_y_, 2)
        ); // calculate distance traveled from the position where the bumper was pressed

        if (distance_traveled < 0.15) // if distance traveled is less than 15cm, keep backing up **NOTE: changed to 15cm because 5cm seemed too small
        {
            linear_ = -0.1; // back up
            angular_ = 0.0;
            RCLCPP_INFO(this -> get_logger(),"Backing up (beep!), distance traveled: %.2f", distance_traveled);
        }
        else // if distance traveled is greater than or equal to 15cm, stop backing up and exit moveBack()
        {
            RCLCPP_INFO(this -> get_logger(),"Finished backing up, distance traveled: %.2f", distance_traveled);
            linear_ = 0.0;
            angular_ = 0.0;
            backup = false; // exit bumper handling routine
        }
    }

    void obstacleAvoidance()
    {
        // adding obstacle avoidance function to be called in the control loop
        //     If the robot is within a certain distance threshold of an obstacle, it will turn away from the obstacle until it is not longer within the distance threshold.
        //     what we know - the robot is too close to an obstacle if minLaserDist_ is less than minimumObstacleDistance
        //     what we want to do - if the robot is too close to an obstacle, we want it to turn away from the obstacle until it is no longer too close (using is_obstacle_on_right/left bools to choose direction).
        //     NOTE: i removed logic for front distance becauce that handling is included in wall following routine.

        if (obstacle_on_right_) 
        {
            RCLCPP_INFO(this->get_logger(), "Obstacle on the right, turning left");
            linear_ = 0.1; // CODE COMMENT: made this greater than 0 to try to prevent the robot from getting stuck turning in place if it gets too close to an obstacle on the right, since we were having some issues with that.
            angular_ = 0.2; // turn left
        }
        else if (obstacle_on_left_)
        {
            RCLCPP_INFO(this->get_logger(), "Obstacle on the left, turning right");
            linear_ = 0.1; // CODE COMMENT: made this greater than 0 to try to prevent the robot from getting stuck turning in place if it gets too close to an obstacle on the left, since we were having some issues with that.
            angular_ = -0.2; // turn right
        }
    }

    // functions that are called in the control loop to check conditions and set flags for which routine to enter (e.g. check if any bumper is pressed, check if robot is stuck, check if robot is too close to an obstacle)

    bool isBumpersPressed()
    {
        // Check if any bumper is pressed
        bool any_bumper_pressed = false;
        for (const auto& [key, val] : bumpers_) {
            if (val) {
                any_bumper_pressed = true;
                //determine specifically which bumper was pressed 
                pressed_bumper = key; // store the key of the pressed bumper
                RCLCPP_INFO(this-> get_logger(),"Pressed bumper: %s", pressed_bumper.c_str());
                break;
            }
          
        } //(should be addressed) CODE COMMENT: bumpers will not always have been pressed when we enter this function, so if we want to save which specific bumper was pressed, we should make a global variable and probably set it in isBumperPressed(),/n
        //      so that it is set when the bumper is first pressed, and not overwritten with every iteration. 

        return any_bumper_pressed;
    }

    bool isStuck()
    {
        if (!stuck_timer_active_) 
        {
            stuck_start_time = this->now();
            stuck_start_x_ = pos_x_;
            stuck_start_y_ = pos_y_;
            stuck_start_yaw_ = yaw_;
            stuck_timer_active_ = true;
            return false;
        }

        double dist = std::hypot(pos_x_ - stuck_start_x_, pos_y_ - stuck_start_y_);
        double yaw_change = std::abs(normalizeAngle(yaw_ - stuck_start_yaw_));
        double time_elapsed = (this->now() - stuck_start_time).seconds();

        // Reset timer if robot is clearly moving forward
        if (dist > STUCK_DISTANCE_THRESH) 
        {
            stuck_timer_active_ = false;
            return false;
        }

        // If turning in place for too long → stuck
        if (yaw_change > STUCK_YAW_THRESH && time_elapsed > STUCK_TIME_THRESH) {
            RCLCPP_WARN(this->get_logger(),
                "Robot appears stuck (%.2fm moved, %.1f deg turned over %.1fs)",
                dist, rad2deg(yaw_change), time_elapsed);
            stuck_timer_active_ = false;
            return true;
        }

        return false;
    }

    bool tooCloseToObstacle()
    {
        // check through idices in the front half of the laser range data (90 degrees on either side of the front center laser measurement)

        if (minLaserDist_ < minimumObstacleDistance) 
        {
            RCLCPP_WARN(this->get_logger(), "Too close to obstacle! Executing avoidance maneuver.");
            if (obstacle_idx_ < front_idx_) // obstacle is on the right
            {
                obstacle_on_right_ = true;
                obstacle_on_left_ = false;
            }
            else if (obstacle_idx_ > front_idx_) // obstacle is on the left
            {
                obstacle_on_right_ = false;
                obstacle_on_left_ = true;
            }
            return true;
        }
        return false;
    }

    // functions that are called in the control loop to perform specific calculations (e.g. normalize angle, create filtered laser range array, set current positions for bumper handling routine)

    double normalizeAngle(double angle)   
    {
        // Normalize angle in radian so it stars within pi to -pi. Used during YawChange calc - while turning a corner
        while (angle > M_PI)
            angle -= 2.0 * M_PI;

        while (angle < -M_PI)
            angle += 2.0 * M_PI;

        return angle;
    }
    
    void createLasersArray()
    {
        // Create filteredLaserRange_ array with laser distances above 20cm
        for (const auto& range : laserRange_) {
            if (range >= 0.2f) {
                filteredLaserRange_.push_back(range);
            }
        }
    }

    void setCurrentPositions()
    {
        // Set current positions and yaw for bumper handling routine
        start_pos_x_ = pos_x_;
        start_pos_y_ = pos_y_;
        start_yaw_ = yaw_;
    }
    
    #pragma endregion Utilities

    void controlLoop()
    {
        RCLCPP_INFO(this->get_logger(), "in control loop");
        // Calculate elapsed time
        auto current_time = this->now();
        double seconds_elapsed = (current_time - start_time_).seconds();

        //RCLCPP_INFO(this -> get_logger(),"Position: (%.2f, %.2f), orientation: %f rad or %f deg", pos_x_, pos_y_, yaw_, rad2deg(yaw_));

        // Check if 480 seconds (8 minutes) have elapsed
        if (seconds_elapsed >= 480.0) {
            RCLCPP_INFO(this->get_logger(), "Contest time completed (480 seconds). Stopping robot.");

            // Stop the robot
            geometry_msgs::msg::TwistStamped vel;
            vel.header.stamp = this->now();
            vel.twist.linear.x = 0.0;
            vel.twist.angular.z = 0.0;
            vel_pub_->publish(vel);

            // Shutdown the node
            rclcpp::shutdown();
            return;
        }

        // Our exploration code below this point ////////////////////////////////////////////////////////////////////////////////////////////////////////
        
        // SET UP VARIABLES AND CONDITIONS //

        // check if robot is stuck, and if yes, call isStuck and reset handling so that robot goes back to startup routine
        if (isStuck()) 
        {
            RCLCPP_WARN(this->get_logger(), "Resetting to startup behavior");
            startup = true;
            callstartupRoutine = true;
            isTurning = false;
            enterBumperHandling = false;
        }

        // Check if any bumper is pressed
        anyBumperPressed = isBumpersPressed(); // checks if any bumper has been pressed (updated every iteration) - if yes, saves which specific bumper was pressed in pressed_bumper variable

        // Set up conditional handling for bumper pressed or startup
        if (anyBumperPressed) // if true, set flag to enter bumper handling routine and define current positions
        {
            enterBumperHandling = true; // set flag to enter bumper handling routine (true until bumper handling is complete)   
            backup = true; // set flag to enter backup routine within bumper handling
            setCurrentPositions(); // saves the yaw and position of the robot at the moment the bumper was pressed
        }
        else if (startup) // if true (first loop), set initial goal yaw and define current positions and set to false
        {       
            if (filteredLaserRange_.empty()) {return; }     
            // Find index of maximum distance from filtered laser ranges
            max_element = *std::max_element(begin(filteredLaserRange_), end(filteredLaserRange_)); // find maximum distance from filtered laser ranges
            max_element_idx = std::distance(filteredLaserRange_.begin(), std::max_element(filteredLaserRange_.begin(), filteredLaserRange_.end())); // find index of maximum distance

            // Determine turn direction based on index of maximum distance
            turnClockwise = max_element_idx < (filteredLaserRange_.size() / 2); // true = turn right, false = turn left
            startup = false;
            callstartupRoutine = true; // set flag to call startup routine in control loop
        }
        
        // ENTER ROUTINE BASED ON CONDITIONS //
        if (enterBumperHandling)  { bumperPressedHandling(); } // handle bumper pressed event
        else if (tooCloseToObstacle()) { obstacleAvoidance(); } // handle obstacle avoidance if robot is too close to an obstacle //CODE COMMENT: i moved the code from the earlier statements here (split between tooCloseToObstacle() - which includes the checks - and obstacleAvoidance() - which includes the linear_ and angular_ instructions) to make it more organized and clear
        else if (callstartupRoutine) { startupRoutine(); } // perform startup routine (orient robot to face furthest wall & approach)
        else if (seconds_elapsed > 360) { randomRoutine(); } // perform random search algorithm after 6 minutes have elapsed
        else { wallFollowingRoutine(); } // perform wall following routine

        // SET AND PUBLISH VELOCITY COMMAND //
        geometry_msgs::msg::TwistStamped vel;
        vel.header.stamp = this->now();
        vel.twist.linear.x = linear_;
        vel.twist.angular.z = angular_;
        vel_pub_->publish(vel);
        return;
    }


    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<irobot_create_msgs::msg::HazardDetectionVector>::SharedPtr hazard_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time start_time_;

    // Velocity commands
    float angular_;
    float linear_;

    // Robot position state
    double pos_x_;
    double pos_y_;
    double yaw_;

    // Bumper states
    std::map<std::string, bool> bumpers_;

    float minLaserDist_;
    int minLaserDist_idx_;
    int32_t nLasers_;
    int32_t desiredNLasers_;
    int32_t desiredAngle_;
    std::vector<float> laserRange_;
    std::vector<float> filteredLaserRange_;

    bool startup;
    bool callstartupRoutine;

    bool enterBumperHandling;
    bool anyBumperPressed;
    bool backup; // boolean to indicate whether robot should be in backup routine within bumper handling

    bool isTurning;
    float minimumObstacleDistance;
    float minimumFrontDistance_;

    float max_element;
    int max_element_idx;
    bool turnClockwise;

    float front_distance_;
    float right_distance_;
    float left_distance_;
    float back_distance_; 
    int front_idx_;
    int back_idx_;
    int right_idx_;
    int left_idx_;

    float obstacle_idx_;
    bool obstacle_on_right_;
    bool obstacle_on_left_;

    float target_yaw; // used in random routine to set a random target yaw to turn towards when the robot is too close to an obstacle
    bool randomTurn; // boolean to indicate whether robot should be in random turn routine


    float start_pos_x_;
    float start_pos_y_;
    float start_yaw_;
    float distance_traveled;
    std::string pressed_bumper;

    // Stuck detection
    rclcpp::Time stuck_start_time;
    double stuck_start_x_;
    double stuck_start_y_;
    double stuck_start_yaw_;
    bool stuck_timer_active_ = false;

    // Tunables
    const double STUCK_DISTANCE_THRESH = 0.15;   // meters
    const double STUCK_YAW_THRESH = deg2rad(90); // radians
    const double STUCK_TIME_THRESH = 8.0;        // seconds

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Contest1Node>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}