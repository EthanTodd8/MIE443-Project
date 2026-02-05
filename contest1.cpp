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
        minRightLaserDist_ = std::numeric_limits<float>::infinity();
        minLeftLaserDist_ = std::numeric_limits<float>::infinity();
        nLasers_ = 0;
        desiredNLasers_ = 0;
        desiredAngle_ = 5;
        desiredAngle_2 = 180;
        frontTooClose = 0.5;
        rightWallMax = 0.5;
        isTurning = false;
        startYaw = 0.0;
        initialGoalYaw = 0.0;
        enterBumperHandling = false;
        
        front_idx_ = 0;
        right_idx_ = 0;
        left_idx_ = 0;
        front_distance_ = 0.0;
        right_distance_ = 0.0;
        left_distance_ = 0.0;
        
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

        front_idx_ = nLasers_ / 2;
        right_idx_ = nLasers_ / 4;
        left_idx_ = 3 * nLasers_ / 4;

        front_distance_ = laserRange_[front_idx_];
        right_distance_ = laserRange_[right_idx_];
        left_distance_ = laserRange_[left_idx_];
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
        """startupRoutine to orient robot towards direction of maximum laser distance and drive to within 20cm of an obstacle""";
        RCLCPP_INFO(this -> get_logger(),"WE'RE IN STARTUP ROUTINE");

        // Continuously checking max distance and rotating until we're within +- ~2.5 degrees of the max distance direction
        // if (abs(initialGoalYaw - yaw_) > 0.04) 
        // {   
        //     RCLCPP_INFO(this -> get_logger(),"initialGoalYaw: %f, current yaw_: %f", initialGoalYaw, yaw_);
        //     linear_ = 0.0;
        //     angular_ = 0.25;
        // }
        
        // Move forward until minimum laser distance is 20cm or less
        if (minLaserDist_ > frontTooClose)
        {
            linear_ = 0.25;
            angular_ = 0.0;
        }

        else
        {
            callstartupRoutine = false; // Exit startup routine
        }

        return;
    }

    void wallFollowingRoutine()
    {

        // int center_index = nLasers_ / 2;
        // int right_index = nLasers_ / 4;
        // int left_index = 3 * nLasers_ / 4;

        // float center_distance = laserRange_[center_index];
        // float right_distance = laserRange_[right_index];      
        // float left_distance = laserRange_[left_index];

        /* Wall following routine to follow the right wall at a set distance */
        RCLCPP_INFO(this -> get_logger(),"WE'RE IN WALL FOLLOWING ROUTINE");

        if (minLaserDist_ < frontTooClose)
        {
            RCLCPP_INFO(this -> get_logger(),"front too close");
            linear_ = 0.0;
            angular_ = 0.25; // turn left to avoid front obstacle
        }
        else 
        {
            // proceed forward while adjusting to maintain right wall distance
            if (right_distance_ < rightWallMax*0.9) // too close to right wall, with 20cm buffer
            {
                RCLCPP_INFO(this -> get_logger(),"right wall too close");
                linear_ = 0.0;
                angular_ = 0.25; // turn left slightly
            }
            else if (right_distance_ > rightWallMax*1.1) // too far from right wall
            {
                RCLCPP_INFO(this -> get_logger(),"right wall too far");
                linear_ = 0.1;
                angular_ = -0.25; // turn right slightly
            }
            else // within acceptable range of right wall
            {
                linear_ = 0.2;
                angular_ = 0.0; // go straight
            }
        }




        // bool wallonRight = isWallOnRight();
        
        // if (!wallonRight) // front obstacle too close
        // {
        //     linear_ = 0.0;
        //     angular_ = 0.25; // turn left to avoid front obstacle
        //     // if (minRightLaserDist_ < rightWallMax) // wall on right side too
        //     // {
        //     //     RCLCPP_INFO(this -> get_logger(),"WE'RE IN WALL FOLLOWING ROUTINE");
        //     //     linear_ = 0.0;
        //     //     angular_ = 0.25; // turn left to avoid front obstacle
        //     //     return;
        //     // }
        //     // else // no wall on right side
        //     // {
        //     //     linear_ = 0.0;
        //     //     angular_ = -0.25; // turn right to find wall
        //     //     return;
        //     // }
        // } 

        // else if (minLaserDist_ > frontTooClose)
        // {
        //     linear_ = 0.2;
        //     angular_ = 0.0; // go straight
        // }

        // else
        // {
        //     RCLCPP_INFO(this -> get_logger(),"CLOSE IN FRONT, WALL ON RIGHT");
        //     linear_ = 0.0;
        //     angular_ = 0.25; // turn left to avoid front obstacle
        // }
        // else if (minRightLaserDist_ > rightWallMax*2) // clear in front but no wall on right side
        // {
        //     RCLCPP_INFO(this -> get_logger(),"CLEAR IN FRONT, NO WALL ON RIGHT");
        //     angular_ = -0.25; // turn right to find wall
        //     linear_ = 0.0;

        // } 

        // else  // clear in front and wall on right side
        // {         
        //     RCLCPP_INFO(this -> get_logger(),"CLEAR IN FRONT, WALL ON RIGHT");   
        //     // proceed forward while adjusting to maintain right wall distance
        //     if (minRightLaserDist_ < rightWallMax*0.5) // too close to right wall, with 20cm buffer
        //     {
        //         linear_ = 0.2;
        //         angular_ = -0.25; // turn left slightly
        //     }
        //     else if (minRightLaserDist_ > rightWallMax*1.5) // too far from right wall
        //     {
        //         linear_ = 0.2;
        //         angular_ = 0.25; // turn right slightly
        //     }
        //     else // within acceptable range of right wall
        //     {
        //         linear_ = 0.2;
        //         angular_ = 0.0; // go straight
        //     }
        // }

        return;
        
    }

    #pragma endregion Routines

    #pragma region Utilities

    bool isWallOnRight()
    {
        """ Check if there is a wall on the right side within the defined maximum distance """;
        int right_idx = nLasers_ / 4; // Right side index in laser scan array
        int center_idx = nLasers_ / 2; // straight ahead index in laser scan array

        int min_idx = center_idx;
        float min_distance = laserRange_[min_idx];

        for (int32_t laser_idx = center_idx; laser_idx <= right_idx; ++laser_idx) {
            if (laserRange_[laser_idx] < min_distance) {
                min_idx = laser_idx;
                min_distance = laserRange_[laser_idx];
            }
        }

        RCLCPP_INFO(this -> get_logger(),"min_idx: %d, right_idx: %d", min_idx, right_idx);
        return min_idx == right_idx;
    }
    
    bool isBumpersPressed()
    {
        """ Check if any bumper is pressed """;
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
        """ Normalize angle in radian so it stars within pi to -pi. Used during YawChange calc - while turning a corner """;
        while (angle > M_PI)
            angle -= 2.0 * M_PI;

        while (angle < -M_PI)
            angle += 2.0 * M_PI;

        return angle;
    }

    void velocityPublish()
    {
        """Setting/publishing velocity commands""";

        // Set velocity command
        geometry_msgs::msg::TwistStamped vel;
        vel.header.stamp = this->now();
        vel.twist.linear.x = linear_;
        vel.twist.angular.z = angular_;

        // Publish velocity command
        vel_pub_->publish(vel);

        return;
    }
    
    void bumperPressedHandling()
    {
        """ Handling bumper pressed event """;
        RCLCPP_INFO(this -> get_logger(),"Bumper pressed! Handling...");

        // Stop the robot immediately
        
        return;
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

        // // Implement your exploration code here
        // if (enterBumperHandling == false) 
        // {
        //     enterBumperHandling = isBumpersPressed();
        // }

        // if (enterBumperHandling) 
        // {
        //     bumperPressedHandling();
        //     return; // Skip rest of control loop while handling bumper press
        // }

        bool anyBumperPressed = isBumpersPressed();
        if (anyBumperPressed)
        {
            // shutdown robot
            linear_ = 0.0;
            angular_ = 0.0;
            velocityPublish();
            rclcpp::shutdown();
            return;
        }

        // Startup routine to orient robot towards direction of maximum laser distance and drive to within 20cm of an obstacle
        if (startup) {
            // Save initial position
            initialPosition[0] = pos_x_;
            initialPosition[1] = yaw_;
            // initialGoalYaw = maxLaserDistPosition[1];
            // initialGoalYaw = minLaserDistPosition[1];
            initialGoalYaw = yaw_;
            startup = false;
        }
        
        if (callstartupRoutine) { startupRoutine(); }
        else { wallFollowingRoutine(); }

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
    bool callstartupRoutine;
    float initialGoalYaw;
    bool enterBumperHandling;

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



    // front, right and left indices
    int front_idx_;
    int right_idx_;
    int left_idx_;

    float front_distance_;
    float right_distance_;
    float left_distance_;

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Contest1Node>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}