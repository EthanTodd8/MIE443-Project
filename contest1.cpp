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
        angular_ = 0.0;
        linear_ = 0.0;
        pos_x_ = 0.0;
        pos_y_ = 0.0;
        yaw_ = 0.0;
        minLaserDist_ = std::numeric_limits<float>::infinity();
        minRightLaserDist_ = std::numeric_limits<float>::infinity();
        minLeftLaserDist_ = std::numeric_limits<float>::infinity();
        nLasers_ = 0;
        desiredNLasers_ = 0;
        desiredAngle_ = 5;
        desiredAngle_2 = 180;
        frontTooClose = 0.2;
        rightWallMax = 1.0;
        isTurning = false;
        startYaw = 0.0;
        
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
        RCLCPP_INFO(this->get_logger(), "Size of laser scan array: %d, and size of offset: %d", nLasers_, desiredNLasers_);
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

        // EDITED CODE BELOW THIS LINE
        // Find minimum laser distance on the right side within +/- desiredAngle from right center
        uint32_t right_idx = (laser_offset + deg2rad(90.0) - scan->angle_min) / scan->angle_increment;
        minRightLaserDist_ = std::numeric_limits<float>::infinity();
        if (deg2rad(desiredAngle_) < scan->angle_max &&deg2rad(desiredAngle_) > scan->angle_min) {
            for (uint32_t laser_idx = right_idx - desiredNLasers_; laser_idx < right_idx + desiredNLasers_; ++laser_idx) {
                minRightLaserDist_ = std::min(minRightLaserDist_, laserRange_[laser_idx]);
            }
        } else {
            for (uint32_t laser_idx = 0; laser_idx < nLasers_; ++laser_idx) {
                minRightLaserDist_ = std::min(minRightLaserDist_, laserRange_[laser_idx]);
            }
        }

        // Find minimum laser distance on the left side within +/- desiredAngle from left center
        uint32_t left_idx = (laser_offset - deg2rad(90.0) - scan->angle_min) / scan->angle_increment;
        minLeftLaserDist_ = std::numeric_limits<float>::infinity();
        if (deg2rad(desiredAngle_) < scan->angle_max &&deg2rad(desiredAngle_) > scan->angle_min) {
            for (uint32_t laser_idx = left_idx - desiredNLasers_; laser_idx < left_idx + desiredNLasers_; ++laser_idx) {
                minLeftLaserDist_ = std::min(minLeftLaserDist_, laserRange_[laser_idx]);
            }
        } else {
            for (uint32_t laser_idx = 0; laser_idx < nLasers_; ++laser_idx) {
                minLeftLaserDist_ = std::min(minLeftLaserDist_, laserRange_[laser_idx]);
            }
        }        

        // Record min and max laser distances & associated yaw angles for full 180 degree scan
        desiredNLasers_ = deg2rad(desiredAngle_2) / scan->angle_increment;
        laser_offset = deg2rad(-180.0);
        front_idx = (laser_offset - scan->angle_min) / scan->angle_increment;
        
        minLaserDistPosition[0] = std::numeric_limits<float>::infinity(); // min distance
        maxLaserDistPosition[0] = 0.0;                                    // max distance
        minLaserDistPosition[1] = 0.0;                                    // yaw associated with min distance
        maxLaserDistPosition[1] = 0.0;                                    // yaw associated with max distance

        // for each laser_idx, save max distance and associated yaw & min distance and associated yaw
        if (deg2rad(desiredAngle_2) <= scan->angle_max &&deg2rad(desiredAngle_2) >= scan->angle_min) {
            for (uint32_t laser_idx = front_idx - desiredNLasers_; laser_idx < front_idx + desiredNLasers_; ++laser_idx) {
                if (laserRange_[laser_idx] < minLaserDistPosition[0])
                {
                    minLaserDistPosition[0] = laserRange_[laser_idx];
                    minLaserDistPosition[1] = scan->angle_min + laser_idx * scan->angle_increment;
                }
                else if (laserRange_[laser_idx] > maxLaserDistPosition[0])
                {
                    maxLaserDistPosition[0] = laserRange_[laser_idx];
                    maxLaserDistPosition[1] = scan->angle_min + laser_idx * scan->angle_increment;
                }
            }
        } 
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
        """
        startupRoutine to orient robot towards direction of maximum laser distance and drive to within 20cm of an obstacle
        """
        RCLCPP_INFO(this -> get_logger(),"current yaw_: %.2f, desired yaw_: %.2f", yaw_, maxLaserDistPosition[1]);
        bool any_bumper_pressed = checkBumpers();
           
        // Save initial position
        initialPosition[0] = pos_x_;
        initialPosition[1] = yaw_;

        // Continuously checking max distance and rotating until we're within +- ~2.5 degrees of the max distance direction
        while (abs(maxLaserDistPosition[1]) > 0.04 && !any_bumper_pressed) 
        {
            linear_ = 0.0;
            angular_ = 0.25;
            velocityPublish();
            any_bumper_pressed = checkBumpers();
        }
        
        // Move forward until minimum laser distance is 20cm or less
        while (minLaserDist_ > 0.2 && !any_bumper_pressed)
        {
            linear_ = 0.25;
            angular_ = 0.0;
            velocityPublish();
            any_bumper_pressed = checkBumpers();
        }

        if (any_bumper_pressed)
        {
            linear_ = 0.0;
            angular_ = 0.0;
            velocityPublish();
            bumperPressedHandling();
        }

        startup = false;
    }

    void wallFollowingRoutine()
    {
        """
        Wall following routine to follow the right wall at a set distance
        """
        bool any_bumper_pressed = checkBumpers();

        if (any_bumper_pressed)
        {
            linear_ = 0.0;
            angular_ = 0.0;
            velocityPublish();
            bumperPressedHandling();
        }

        else if (minLaserDist_ < frontTooClose) // front obstacle too close
        {
            if (minRightLaserDist_ < rightWallMax) // wall on right side too
            {
                // turn 90 degrees left
                cornerHandling(); 
            }
            else // no wall on right side
            {
                // turn 15 degrees right to find wall
                cornerHandling(-deg2rad(15.0));
            }
        }

        else
        {
            // adjust to maintain right wall distance
            // WALL FOLLOWING LOGIC TO BE IMPLEMENTED HERE

            // Default code for now:
            if (minRightLaserDist_ < 0.5 * rightWallMax) // too close to right wall
            {
                linear_ = 0.2;
                angular_ = 0.15; // turn left slightly
                velocityPublish();
            }
            else if (minRightLaserDist_ > 1.5 * rightWallMax) // too far from right wall
            {
                linear_ = 0.2;
                angular_ = -0.15; // turn right slightly
                velocityPublish();
            }
            else // within acceptable range of right wall
            {
                linear_ = 0.2;
                angular_ = 0.0; // go straight
                velocityPublish();
            }
        }
    }

    #pragma endregion Routines

    #pragma region Handlers

    void cornerHandling(float turnAngle = M_PI/2.0)
    {
        """
        Does something when a corner is detected, then returns to wall following
        """
        bool any_bumper_pressed = checkBumpers();
        currentYaw = yaw_;
        targetYaw = normalizeAngle(currentYaw + turnAngle);
        while (yaw_ < targetYaw && !any_bumper_pressed)
        {
            linear_ = 0.0;
            angular_ = 0.25;
            velocityPublish();
            any_bumper_pressed = checkBumpers();
        }
    }

    void bumperPressedHandling()
    {
        """
        Does something when a bumper is pressed, then returns to what it was doing before
        """
        bool any_bumper_pressed = false;
        float initialYaw = yaw_;
        float initialX = pos_x_;

        // Rotate 90 degrees to the left
        while (yaw_ < initialYaw + M_PI/2.0)
        {
            linear_ = 0.0;
            angular_ = 0.25;
            velocityPublish();
        }

        // Move forward 0.5m
        while (pos_x_ < initialX + 0.5 && !any_bumper_pressed && minLaserDist_ > 0.25)
        {
            linear_ = 0.25;
            angular_ = 0.0;
            velocityPublish();
            any_bumper_pressed = checkBumpers();    
        }

        if (any_bumper_pressed) // Recursively call bumperPressedHandling again
        {
            linear_ = 0.0;
            angular_ = 0.0;
            velocityPublish();
            bumperPressedHandling();
        }

        else // Resume previous routine
        { 
            if (startup) { startupRoutine(); }
            else { wallFollowingRoutine(); }
        }
        
    }
    
    #pragma endregion Handlers

    #pragma region Utilities

    bool checkBumpers()
    {
        """ Check if any bumper is pressed """
        bool any_bumper_pressed = false;
        for (const auto& [key, val] : bumpers_) {
            if (val) {
                any_bumper_pressed = true;
                break;
            }
        }
        return any_bumper_pressed;
    }

    double normalizeAngle(double angle)   
    {
        """ Normalize angle in radian so it stars within pi to -pi. Used during YawChange calc - while turning a corner """
        while (angle > M_PI)
            angle -= 2.0 * M_PI;

        while (angle < -M_PI)
            angle += 2.0 * M_PI;

        return angle;
    }

    void velocityPublish()
    {
        """Setting/publishing velocity commands"""

        // Set velocity command
        geometry_msgs::msg::TwistStamped vel;
        vel.header.stamp = this->now();
        vel.twist.linear.x = linear_;
        vel.twist.angular.z = angular_;

        // Publish velocity command
        vel_pub_->publish(vel);
    }
    
    #pragma endregion Utilities

    void controlLoop()
    {
        // Calculate elapsed time
        auto current_time = this->now();
        double seconds_elapsed = (current_time - start_time_).seconds();

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

        RCLCPP_INFO(this -> get_logger(),"Position: (%.2f, %.2f), orientation: %f rad or %f deg", pos_x_, pos_y_, yaw_, rad2deg(yaw_));

        // Implement your exploration code here
        bool any_bumper_pressed = checkBumpers();
        if (any_bumper_pressed == true)
        {
            linear_ = 0.0;
            angular_ = 0.0;
            velocityPublish();
            bumperPressedHandling();
        }

        // Startup routine to orient robot towards direction of maximum laser distance and drive to within 20cm of an obstacle
        if (startup == true) { startupRoutine(); }
        wallFollowingRoutine();

        velocityPublish();
    }


    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<irobot_create_msgs::msg::HazardDetectionVector>::SharedPtr hazard_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time start_time_;
    float angular_;
    float linear_;

    double pos_x_;
    double pos_y_;
    double yaw_;

    std::map<std::string, bool> bumpers_;

    float minLaserDist_;
    float minRightLaserDist_;
    float minLeftLaserDist_;
    int32_t nLasers_;
    int32_t desiredNLasers_;
    int32_t desiredAngle_;
    int32_t desiredAngle_2;
    std::vector<float> laserRange_;
    bool startup;

    // min&max laser dist & associated yaw
    double minLaserDistPosition[2] = {0.0};
    double maxLaserDistPosition[2] = {0.0};

    // initial position
    double initialPosition[2] = {0.0};

    // index and interpret front, left, and right regions of LIDAR scan data
    int frontIndex;
    int leftIndex;
    int rightIndex;

    // corresponding distances 
    float frontDist;
    float rightDist;
    float leftDist;

    // Distance (in m) at which front wall is too close
    float frontTooClose;

    // Maximum distance (in m) at which right wall is considered to be present
    float rightWallMax;

    // turning state
    bool isTurning;

    // start yaw 
    double startYaw;

    // change in yaw, due to rotation, calc based on the startYaw
    double yawChange;

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Contest1Node>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}