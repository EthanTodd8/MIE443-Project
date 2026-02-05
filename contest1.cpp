#include <chrono>
#include <memory>
#include <cmath>
#include <map>
#include <vector>
#include <algorithm>

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

        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&Contest1Node::laserCallback, this, std::placeholders::_1));

        

        hazard_sub_ = this->create_subscription<irobot_create_msgs::msg::HazardDetectionVector>(
            "/hazard_detection", rclcpp::SensorDataQoS(),
            std::bind(&Contest1Node::hazardCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::SensorDataQoS(),
            std::bind(&Contest1Node::odomCallback, this, std::placeholders::_1));

        // Timer for main control loop at 10 Hz
        timer_ = this->create_wall_timer(
            100ms, std::bind(&Contest1Node::controlLoop, this));

        // Initialize variables
        start_time_ = this->now();
        startup = true;
        callstartupRoutine = true;
        angular_ = 0.0;
        linear_ = 0.0;
        pos_x_ = 0.0;
        pos_y_ = 0.0;
        yaw_ = 0.0;
        minLaserDist_ = std::numeric_limits<float>::infinity();
        nLasers_ = 0;
        desiredNLasers_ = 0;
        desiredAngle_ = 5;
        minimumFrontDistance = 0.5;
        isTurning = false;
        initialGoalYaw = 0.0;
        enterBumperHandling = false;
        currentYaw = 0.0;
        currentX = 0.0;
        currentY = 0.0;
        anyBumperPressed = false;
        front_distance_ = std::numeric_limits<float>::infinity();
        right_distance_ = std::numeric_limits<float>::infinity();
        left_distance_ = std::numeric_limits<float>::infinity();
        back_distance_ = std::numeric_limits<float>::infinity();

        // Initialize bumper states
        bumpers_["bump_from_left"] = false;
        bumpers_["bump_front_center"] = false;
        bumpers_["bump_front_right"] = false;
        bumpers_["bump_left"] = false;
        bumpers_["bump_right"] = false;
        
        // Initializes node - should be last
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

        minLaserDist_ = std::numeric_limits<float>::infinity();
        if (deg2rad(desiredAngle_) < scan->angle_max &&deg2rad(desiredAngle_) > scan->angle_min) {
            for (uint32_t laser_idx = front_idx - desiredNLasers_; laser_idx < front_idx + desiredNLasers_; ++laser_idx) {
                minLaserDist_ = std::min(minLaserDist_, laserRange_[laser_idx]);
            }
        } else {
            for (uint32_t laser_idx = 0; laser_idx < nLasers_; ++laser_idx) {
                minLaserDist_ = std::min(minLaserDist_, laserRange_[laser_idx]);
            }
        }

        int front_idx_from_scan_ = static_cast<int>(
            (0.0 - scan->angle_min) / scan->angle_increment
        );

        //front_idx_ = front_idx_from_scan_;

        int front_idx_ = nLasers_ / 4;
        int back_idx_ = nLasers_ * 3/4 ;
        int left_idx_ = nLasers_ / 2;
        int right_idx_ = 0;

        front_distance_ = laserRange_[front_idx_];
        right_distance_ = laserRange_[right_idx_];
        left_distance_ = laserRange_[left_idx_];
        back_distance_ = laserRange_[back_idx_];

        RCLCPP_INFO(this->get_logger(), "front_distance_: %f, right_distance_: %f, left_distance: %f, back_distance: %f" , front_distance_, right_distance_, left_distance_, back_distance_);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
    {
        // implement your code here
        //extract position

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
        // Startup routine to orient robot towards direction of maximum laser distance and drive to within 50cm of an obstacle
        RCLCPP_INFO(this -> get_logger(),"WE'RE IN STARTUP ROUTINE");
        
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
        bool turnClockwise = false; // true = turn right, false = turn left
        if (front_distance_ <= 0.5 && !isTurning)
        {
            if (left_distance_ <= 0.5 && right_distance_ > 0.01) { turnClockwise = true; } // turn right if left wall is also close
            isTurning = true;
        }
        
        if (!isTurning) // robot is clear ahead
        {
            linear_ = 0.2;
            angular_ = 0.0;
        }
        else { turnRobot(turnClockwise); } // robot is turning
            
        return; 
    }

    #pragma endregion Routines

    #pragma region Utilities

    void turnRobot(bool turnClockwise)
    {
        // Turn robot in specified direction until min side laser distance matches min laser distance from filteredLaserRange_
        RCLCPP_INFO(this->get_logger(), "Turning robot %s", turnClockwise ? "clockwise" : "counter-clockwise");

        // Find minimum distance from filtered laser ranges
        float min_element = *std::min_element(begin(filteredLaserRange_), end(filteredLaserRange_)); // find minimum distance from filtered laser ranges
        float distanceDifference = turnClockwise ? abs(left_distance_ - min_element) : abs(right_distance_ - min_element); // calculate distance difference based on turn direction

        if (distanceDifference > 0.01)  // if not yet reached target rotation
        { 
            linear_ = 0.0;
            angular_ = turnClockwise ? 0.1 : -0.1; // turn in specified direction
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
                break;
            }
        }
        return any_bumper_pressed;
    }

    void bumperPressedHandling()
    {
        // Handling bumper pressed event
        RCLCPP_INFO(this -> get_logger(),"Bumper pressed! Handling...");
        
            RCLCPP_INFO(this -> get_logger(),"bumper was pressed");
            if (abs(pos_x_-currentX) < 0.2)
            {
                RCLCPP_INFO(this -> get_logger(),"bumper was pressed - backup");
                linear_ = -0.1; // back up
                angular_ = 0.0;
            }
            else if (abs(normalizeAngle(yaw_ - currentYaw)) < deg2rad(20.0))
            {
                RCLCPP_INFO(this -> get_logger(),"bumper was pressed - turn");
                linear_ = 0.0;
                angular_ = 0.25; // turn left
            }
            else 
            {
                RCLCPP_INFO(this -> get_logger(),"bumper was pressed - exit handling");
                enterBumperHandling = false; // exit bumper handling once backed up and turned ~90 degrees
                linear_ = 0.0;
                angular_ = 0.0;
            }

        return;
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
        // Set current positions and yaw for bumper handling routine
        currentX = pos_x_;
        currentY = pos_y_;
        currentYaw = yaw_;
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

        // Check if any bumper is pressed
        anyBumperPressed = isBumpersPressed();

        // Set up conditional handling for bumper pressed or startup
        if (anyBumperPressed) // if true, set flag to enter bumper handling routine and define current positions
        {
            enterBumperHandling = true;   
            setCurrentPositions();
        }
        else if (startup) // if true (first loop), set initial goal yaw and define current positions and set to false
        {
            initialGoalYaw = yaw_;
            setCurrentPositions();
            startup = false;
        }

        // ENTER ROUTINE BASED ON CONDITIONS //
        if (enterBumperHandling)  { bumperPressedHandling(); } // handle bumper pressed event
        else if (callstartupRoutine) { startupRoutine(); } // perform startup routine (orient robot to face furthest wall & approach)
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
    float initialGoalYaw;

    bool enterBumperHandling;
    bool anyBumperPressed;

    bool isTurning;
    float minimumFrontDistance;

    float currentYaw;
    float currentX;
    float currentY;

    float front_distance_;
    float right_distance_;
    float left_distance_;
    float back_distance_; 

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Contest1Node>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}