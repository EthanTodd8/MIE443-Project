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
        nLasers_ = 0; //initilaize counter for number of laser data points
        desiredNLasers_ = 0; // part of the default code: for setting up minLaserDistance in laserCallback - sets up number of laser measurements to consider on either side of the center front laser measurement for minimum laser distance calculation (set as however many lasers are within the desiredAngle range)
        desiredAngle_ = 5; // part of the default code: for setting up minLaserDistance in laserCallback - sets up angle range to consider for minimum laser distance calculation - 5 degrees on either side of the center front laser measurement
        minimumFrontDistance = 0.5; //minimum distance the robot can have to an obstacle in front
        isTurning = false; //boolean to indicate whether the robot is currently turning
        enterBumperHandling = false; //boolean to indicate whether robot should enter bumper handling routine
        currentYaw = 0.0; // set to current yaw when setPositions() is called - used for some handler routines (e.g. startup routine)
        currentX = 0.0; // set to current X position when setPositions() is called - used for some handler routines (e.g. startup routine)
        currentY = 0.0; // set to current Y position when setPositions() is called - used for some handler routines (e.g. startup routine)
        anyBumperPressed = false; //boolean to indicate whether any bumper is pressed
        front_distance_ = std::numeric_limits<float>::infinity(); //initiaze front distance to infinity, all other measurements will be smaller
        right_distance_ = std::numeric_limits<float>::infinity(); //initiaze right distance to infinity, all other measurements will be smaller
        left_distance_ = std::numeric_limits<float>::infinity(); //initiaze left distance to infinity, all other measurements will be smaller
        back_distance_ = std::numeric_limits<float>::infinity();// initiaze back distance to infinity, all other measurements will be smaller

        // Initialize bumper states to false 
        bumpers_["bump_from_left"] = false;
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

        // Find minimum laser distance within +/- desiredAngle from front center
        float laser_offset = deg2rad(-90.0);
        uint32_t front_idx = (laser_offset - scan->angle_min) / scan->angle_increment;   

        int front_idx_ = nLasers_ / 4; //find index of front obstacle distance measurement
        int back_idx_ = nLasers_ * 3/4 ; //find index of back obstacle distance measurement
        int left_idx_ = nLasers_ / 2; //find index of left obstacle distance measurement
        int right_idx_ = 0; //find index of right obstacle distance measurement

        minLaserDist_ = std::numeric_limits<float>::infinity(); //find the closest obstacle to the current position of the robot
        if (deg2rad(desiredAngle_) < scan->angle_max &&deg2rad(desiredAngle_) > scan->angle_min) {
            for (uint32_t laser_idx = front_idx - desiredNLasers_; laser_idx < front_idx + desiredNLasers_; ++laser_idx) {
                minLaserDist_ = std::min(minLaserDist_, laserRange_[laser_idx]);
            }
        } else {
            for (uint32_t laser_idx = 0; laser_idx < nLasers_; ++laser_idx) {
                minLaserDist_ = std::min(minLaserDist_, laserRange_[laser_idx]);
            }
        }

        front_distance_ = laserRange_[front_idx_];  //extract distance of obstacle in front of robot
        right_distance_ = laserRange_[right_idx_];  //extract distance of obstacle to right of robot
        left_distance_ = laserRange_[left_idx_];  //extract distance of obstacle to left of robot
        back_distance_ = laserRange_[back_idx_]; //extract distance of obstacle behind robot

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
        if (front_distance_ > minimumFrontDistance)
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

    #pragma endregion Routines

    #pragma region Utilities

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
            angular_ = _turnClockwise ? 0.1 : -0.1; // turn in specified direction
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

    void bumperPressedHandling()
    {
        // Handling bumper pressed event
        RCLCPP_INFO(this -> get_logger(),"Bumper pressed! Handling...");

        // CODE COMMENT: Everything is called over and over again as the control loop is executed. This means that the last linear_ and angular_ values that are set in each loop will overwrite any previous logic. /n
        //         So if you set linear_ as a negative value in moveBack(), when you exit moveBack(), linear_ and angular_ will be set to the values following the moveBack() function, and the logic from moveBack() is overwritten.

        //the desired logic here is that our code enters this function and moves back/executes this movement until the robot is far enough and is safely able to continue
        //do we need a boolean to be able to enter this function and not exit until the robot is far enough back? or can we just use the distance traveled to determine whether we are far enough back to exit the function and continue with the rest of the control loop logic?
        if(pressed_bumper == "bump_from_left" || pressed_bumper == "bump_left")
        {
            RCLCPP_INFO(this -> get_logger(),"Left bumper was pressed - turn right");
            moveBack(0); // CODE COMMENT: input to moveBack() should probably be a global variable
            linear_ = 0.0;
            angular_ = -0.2; // turn right
        }
        else if (pressed_bumper == "bump_front_center" || pressed_bumper == "bump_front_right" )
        {
            RCLCPP_INFO(this -> get_logger(),"Front bumper was pressed - back up");
            moveBack(0);
            linear_ = -0.1; // back up
            angular_ = 0.0;
        }
        else if (pressed_bumper == "bump_right")
        {
            RCLCPP_INFO(this -> get_logger(),"Right bumper was pressed - turn left");
            moveBack(0);
            linear_ = 0.0;
            angular_ = 0.2; // turn left
        }
        return;
    }
    
    void moveBack(int state_){
        if (state_ ==0){
            if(start_pos_x ==0.0 && start_pos_y_ ==0.0) { 
                start_pos_x = currentX;
                start_pos_y_ = currentY;
                //  (addressed?) CODE COMMENT
                // start_pos_x and start_pos_y_ are initialized to 0 with every iteration of the control loop, so this condition will always be true, and therefore when /n
                //   these variables are accessed later, they will always equal pos_x_ and pos_y_. If you want to access the position of the robot at the moment the bumper was /n
                //   pressed without updating every time, you can use currentX and currentY, which are set to the position of the robot at the moment the bumper is pressed in /n
                //   the setCurrentPositions() function (check line 472), and are not updated until the next time a bumper is pressed. Otherwise, you could just used pos_x_ and pos_y_ directly.
            }
        }

        double distance_traveled = std::sqrt(
            std::pow(pos_x_ - start_pos_x, 2) + std::pow(pos_y_ - start_pos_y_, 2)
        );

        if(distance_traveled <target_distance) //CODE COMMENT: this is a global variable but it's only used here. If we leave it as a global variable, we should set it at the top with the other global variables, but since it's only used here, to just use 0.05 and not make it a variable, or make it local.
        {
            linear_ = -0.1; // back up
            angular_ = 0.0;
            RCLCPP_INFO(this -> get_logger(),"Backing up (beep!), distance traveled: %.2f", distance_traveled);
        }
        else {
            RCLCPP_INFO(this -> get_logger(),"Finished backing up, distance traveled: %.2f", distance_traveled);
            state_ = 1; // CODE COMMENT: this variable is always 0 because moveBack is always called with 0 as an argument. This won't update the value. We can make a global variable for state if we want to keep track of it across function calls.
            linear_ = 0.0;
            angular_ = 0.0;
        }

    }

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
        // CODE COMMENT: NOT USED RIGHT NOW - MAY BE USEFUL FOR BUMPER HANDLING
        // Set current positions and yaw for bumper handling routine
        currentX = pos_x_;
        currentY = pos_y_;
        currentYaw = yaw_;
    }

    void randomSearchAlgorithm() 
    {
        // Implement a random search algorithm for exploration
        static std::default_random_engine generator;
        static std::uniform_real_distribution<double> distribution(-M_PI, M_PI);

        // Generate a random number
        double random_angle = distribution(generator);
        RCLCPP_INFO(this->get_logger(), "Randomly turning to angle: %f radians", random_angle);
        
        static bool RandomTurn = false; // CODE COMMENT: this variable is always false because it's defined in the function. If we want to keep track of it across function calls, we should make it a global variable.
        static double target_yaw = 0.0; // CODE COMMENT: this variable is always 0 because it's defined in the function. If we want to keep track of it across function calls, we should make it a global variable.
        
        if (front_distance <= 0.5 && !RandomTurn) {
            linear_ = -0.1; // back up if too close to obstacle // CODE COMMENT: this is overwritten in the next iterations a few miliseconds later, so might as well be 0
            angular_ = 0.0;
            RandomTurn = true;
            
        }else {
            linear_ = 0.2;
            angular_ = 0.0;
        }

        if (!RandomTurn) {
            target_yaw = normalizeAngle(currentYaw  + random_angle);
        }else{
            angular_ = (normalizeAngle(target_yaw - currentYaw) > 0) ? 0.2 : -0.2; // Set angular velocity to turn towards the target yaw
            linear_ = 0.0;

            if (abs(normalizeAngle(target_yaw - currentYaw)) < 0.1) {
                angular_ = 0.0;
                RandomTurn = false;
            }
        }

    }

    bool isStuck()
    {
        if (!stuck_timer_active_) 
        {
            stuck_start_time_ = this->now();
            stuck_start_x_ = pos_x_;
            stuck_start_y_ = pos_y_;
            stuck_start_yaw_ = yaw_;
            stuck_timer_active_ = true;
            return false;
        }

        double dist = std::hypot(pos_x_ - stuck_start_x_, pos_y_ - stuck_start_y_);
        double yaw_change = std::abs(normalizeAngle(yaw_ - stuck_start_yaw_));
        double time_elapsed = (this->now() - stuck_start_time_).seconds();

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
        // Define filteredLaserRange array with values from laserRange_ above 20cm
        createLasersArray(); 

        // check if robot is stuck, and if yes, call isStuck and reset handling so that robot goes back to startup routine
        if (isStuck()) 
        {
            RCLCPP_WARN(this->get_logger(), "Resetting to startup behavior");
            startup = true;
            callstartupRoutine = true;
            isTurning = false;
            enterBumperHandling = false;

            filteredLaserRange_.clear();
        }
        //set start x and y position equal to 0
        start_pos_x = 0.0;
        start_pos_y_ = 0.0;
        target_distance = 0.05; // set target distance to move back when bumper is pressed

        // Check if any bumper is pressed
        anyBumperPressed = isBumpersPressed();

        // Set up conditional handling for bumper pressed or startup
        if (anyBumperPressed) // if true, set flag to enter bumper handling routine and define current positions
        {
            enterBumperHandling = true;   
            setCurrentPositions();
        }
        // CODE COMMENT: this logic should be implemented somewhere else in the code. The linear and angular velocities will be overwritten before they are published. 
        //GOOD CATCH. the intention here is to have a check that the robot isn't too close to anything before it starts moving (since we were having issues with this not being checked)
        //any suggestions on where/how to fix this? a boolean called tooClose that avoids the remaining logic triggering/re-publishing velocity commands if the robot is too close to an obstacle? 
        else if (min_element(filteredLaserRange_.begin(), filteredLaserRange_.end()) >= minLaserDist_) //checks to see where the closest obstacle is located, and orients the robot away from it if within a certain distance threshold
        {
            obstacle_idx_ = min_element(filteredLaserRange_.begin(), filteredLaserRange_.end()) - filteredLaserRange_.begin();
            if(obstacle_idx_ < front_idx_ )
            {
                angular_=-0.2; //turn to the left because the obstacle is on the right
                linear_ = 0.0;
            }
            else if (obstacle_idx_ > front_idx_)
            {
                angular_= 0.2; //turn to the right because the obstacle is on the left
                linear_ = 0.0;
            }
            else if (obstacle_idx_ == front_idx_)
            {
                angular_ = 0.0;
                linear_ = -0.1; //back up if the obstacle is directly in front
            }
            
        }
        else if (startup) // if true (first loop), set initial goal yaw and define current positions and set to false
        {            
            // Find index of maximum distance from filtered laser ranges
            max_element = *std::max_element(begin(filteredLaserRange_), end(filteredLaserRange_)); // find maximum distance from filtered laser ranges
            max_element_idx = std::distance(filteredLaserRange_.begin(), std::max_element(filteredLaserRange_.begin(), filteredLaserRange_.end())); // find index of maximum distance

            // Determine turn direction based on index of maximum distance
            turnClockwise = max_element_idx < (filteredLaserRange_.size() / 2); // true = turn right, false = turn left
            startup = false;
        }

        // ENTER ROUTINE BASED ON CONDITIONS //
        if (enterBumperHandling)  { bumperPressedHandling(); } // handle bumper pressed event
        else if (callstartupRoutine) { startupRoutine(); } // perform startup routine (orient robot to face furthest wall & approach)
        else if (seconds_elasped > 360) { randomSearchAlgorithm(); } // perform random search algorithm after 6 minutes have elapsed
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
    int32_t nLasers_;
    int32_t desiredNLasers_;
    int32_t desiredAngle_;
    std::vector<float> laserRange_;
    std::vector<float> filteredLaserRange_;

    bool startup;
    bool callstartupRoutine;

    bool enterBumperHandling;
    bool anyBumperPressed;

    bool isTurning;
    float minimumFrontDistance;

    float currentYaw;
    float currentX;
    float currentY;
    float max_element;
    int max_element_idx;
    bool turnClockwise;

    float front_distance_;
    float right_distance_;
    float left_distance_;
    float back_distance_; 

    float obstacle_idx_;
    float start_pos_x;
    float start_pos_y_;
    float distance_traveled;
    float target_distance; // distance to move back when bumper is pressed
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