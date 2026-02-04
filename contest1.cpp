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
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        // implement your code here
        nLasers_ = (scan->angle_max - scan-> angle_min) / scan->angle_increment;
        laserRange_ = scan->ranges;
        desiredNLasers_ = deg2rad(desiredAngle_) / scan->angle_increment; // number of lasers to consider on each side
        RCLCPP_INFO(this->get_logger(), "Size of laser scan array: %d, and size of offset: %d", nLasers_, desiredNLasers_);
        //RCLCPP_INFO(this->get_logger(), "angle max %.2f, angle min %.2f, range_min %.2f, range_max %.2f", scan->angle_max, scan->angle_min, scan->range_min, scan->range_max);


        float laser_offset = deg2rad(-90.0);
        uint32_t front_idx = (laser_offset - scan->angle_min) / scan->angle_increment;
        
        minLaserDist_ = std::numeric_limits<float>::infinity();

        // Find minimum laser distance within +/- desiredAngle from front center
        if (deg2rad(desiredAngle_) < scan->angle_max &&deg2rad(desiredAngle_) > scan->angle_min) {
            for (uint32_t laser_idx = front_idx - desiredNLasers_; laser_idx < front_idx + desiredNLasers_; ++laser_idx) {
                minLaserDist_ = std::min(minLaserDist_, laserRange_[laser_idx]);
            }
        } else {
            for (uint32_t laser_idx = 0; laser_idx < nLasers_; ++laser_idx) {
                minLaserDist_ = std::min(minLaserDist_, laserRange_[laser_idx]);
            }
        }

        // laserCallback_2 uses a 180 degree offset instead of 90 degree offset, with desiredAngle_ at 180 degrees instead of 5 degrees.
        // Using this method, we are able to consider all laser measurements in the scan and record the minimum and maximum distance to obstacles
        // along with the associated yaw angles.
        desiredNLasers_ = deg2rad(desiredAngle_2) / scan->angle_increment;
        laser_offset = deg2rad(-180.0);
        front_idx = (laser_offset - scan->angle_min) / scan->angle_increment;
        
        minLaserDistPosition[0] = std::numeric_limits<float>::infinity();
        maxLaserDistPosition[0] = 0.0;
        minLaserDistPosition[1] = 0.0;
        maxLaserDistPosition[1] = 0.0;

        // for each laser_idx, save max distance and associated yaw & min distance and associated yaw
        // Find minimum laser distance within +/- desiredAngle from front center
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
        
        // Update bum[er states based on current detections
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

    /* function to normalise the angle in radian so it stays withing pi to -pi
    used during YawChange calc - while turning a corner */

    double normalizeAngle(double angle)   
    {
        while (angle > M_PI)
            angle -= 2.0 * M_PI;

        while (angle < -M_PI)
            angle += 2.0 * M_PI;

        return angle;
    }

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
        bool any_bumper_pressed = false;
        for (const auto& [key, val] : bumpers_) {
            if (val) {
                any_bumper_pressed = true;
                break;
            }
        }

        // Startup routine to orient robot towards direction of maximum laser distance and drive to within 20cm of an obstacle
        if (startup == true)
        {
            RCLCPP_INFO(this -> get_logger(),"current yaw_: %.2f, desired yaw_: %.2f", yaw_, maxLaserDistPosition[1]);
           
            // Save initial position
            initialPosition[0] = pos_x_;
            initialPosition[1] = yaw_;

            // Continuously checking max distance and rotating until we're within +- ~2.5 degrees of the max distance direction
            while (abs(maxLaserDistPosition[1] > 0.04)) 
            {
                linear_ = 0.0;
                angular_ = 0.25;
                geometry_msgs::msg::TwistStamped vel;
                vel.header.stamp = this->now();
                vel.twist.linear.x = linear_;
                vel.twist.angular.z = angular_;
                vel_pub_->publish(vel);
            }
            // Move forward until minimum laser distance is 20cm or less
            while (minLaserDist_ > 0.2)
            {
                linear_ = 0.25;
                angular_ = 0.0;
                geometry_msgs::msg::TwistStamped vel;
                vel.header.stamp = this->now();
                vel.twist.linear.x = linear_;
                vel.twist.angular.z = angular_;
                vel_pub_->publish(vel);
            }
            startup = false;
        }

        // Corner sub-routine, detects when the front wall is less than 0.2m away from robot & run simultaneously with Right Wall follow behaviour

        // checking to ensure laserRange_ and nLasers_ are not empty/invalid which can cause code to CRASH

        if (laserRange_.empty() || nLasers_ <= 0)
        {
            linear_ = 0.0;
            angular_ = 0.0;
        }
        else
        {
        frontIndex = nLasers_/ 2;     //straight
        rightIndex = nLasers_ /4;     //90 right
        leftIndex = nLasers_ * 3/4;   //90 left - NOT USED

        frontDist = laserRange_[frontIndex];
        rightDist = laserRange_[rightIndex];
        leftDist = laserRange_[leftIndex];           // NOT USED
        }

        if (isTurning == false)                                           // not turning - intialise turning if a corner is detected
        {
            if ((frontDist < frontTooClose) && (rightDist<rightWallMax))  //corner detected
            {
                //start turning 90 degree left

                isTurning = true;
                startYaw = yaw_;

                linear_ = 0.0;
                angular_ = 0.25;                                         // set angular speed in rad/s
            }

            else                                                         // no corner detected - keep driving forward
            {
                linear_ = 0.25;
                angular_ = 0.0;

            }

        }

        else                                                          // it is turning
        {
            yawChange = normalizeAngle(yaw_- startYaw);              // calc the angle it has rotated in radians (pi to -pi) -> func defined above

            if (std::abs(yawChange)<(M_PI/2.0))                      // absolute value of rotation less than pi/2 or 90 deg
            {
                linear_ = 0.0;
                angular_ = 0.25;                                     // keeps rotating
            }

            else                                                    // absolute value of rotation equal or greater than pi/2 - 90 deg
            {
                isTurning = false;                                  // stops rotating, change the state of isTurning = false
                linear_ = 0.25;
                angular_ = 0.0;
            }
        }
        

        //PUT MAIN LOGIC HERE

        if (minLaserDist_ > 0.5 && !any_bumper_pressed)
        {
            linear_ = 0.2;
            angular_ = 0.0;
        }
        else 
        {
            angular_ = 0.0;
            linear_ = 0.0;
            rclcpp::shutdown();
            return;
        }   
        


        // Set velocity command
        geometry_msgs::msg::TwistStamped vel;
        vel.header.stamp = this->now();
        vel.twist.linear.x = linear_;
        vel.twist.angular.z = angular_;

        // Publish velocity command
        vel_pub_->publish(vel);
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