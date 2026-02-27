#include "mie443_contest2/boxes.h"
#include "mie443_contest2/navigation.h"
#include "mie443_contest2/robot_pose.h"
#include "mie443_contest2/yoloInterface.h"
#include "mie443_contest2/arm_controller.h"
#include "mie443_contest2/apriltag_detector.h"
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <thread>
#include <fstream>
#include <sstream>

void startup()
{
    //move the arm to location 1 - pickup the object
    armController.openGripper();
    std::this_thread::sleep_for(std::chrono::seconds(2));

    RCLCPP_INFO(note->get_logger(), "Moving arm to grab unknown object");
    //doing the movement in two steps, once to hover above the object and the second to get close enough to grasp
    armController.moveToCartesianPose(0.043, 0.199, 0.313, -0.471, -0.557, 0.564, -0.387); //need to change this pose pending simulation testing
    armController.moveToCartesianPose(0.043, 0.199, 0.313, -0.471, -0.557, 0.564, -0.387); //need to change this pose pending simulation testing
    
    //close the gripper to grab the object
    RCLCPP_INFO(node-.get_logger(), "Grabbing the unknown object");
    armController.closeGripper();
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    //move the arm to location 2 - in frame for the wrist camera
    RCLCPP_INFO(note->get_logger(), "Moving arm to position for wrist camera object detection");
    armController.moveToCartesianPose(0.043, 0.199, 0.313, -0.471, -0.557, 0.564, -0.387); //need to change this pose pending simulation testing

    //object detection using the wrist camera and determine and save class
    captureAndDetect("Wrist", true);
    detectedClass = latest_class_name_; //NEED TO UPDATE THIS LINE WITH KEY-VALUE SYNTAX
    
    startup = false;

}

void putInBin(){

    //move the arm to location 3 - above the box
    RCLCPP_INFO(node->get_logger(), "Moving arm to position above box");
    armController.moveToCartesianPose(0.043, 0.199, 0.313, -0.471, -0.557, 0.564, -0.387); //need to change this pose pending simulation testing

    //move the arm to location 4 - drop object into box
    RCLCPP_INFO(node->get_logger(), "Moving arm to drop object into box");
    armController.moveToCartesianPose(0.043, 0.199, 0.313, -0.471, -0.557, 0.564, -0.387); //need to change this pose pending simulation testing

    //open gripper to drop object into box
    RCLCPP_INFO(node->get_logger(), "Releasing object into box");
    armController.openGripper();
    std::this_thread::sleep_for(std::chrono::seconds(2));

    //move the arm back up to location 3
    RCLCPP_INFO(node->get_logger(), "Moving arm back up after dropping object into box");
    armController.moveToCartesianPose(0.043, 0.199, 0.313, -0.471, -0.557, 0.564, -0.387); //need to change this pose pending simulation testing

    shouldPutInBin = false;
}

int main(int argc, char** argv) {
    // Setup ROS 2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("contest2");

    // Load the arm URDF and SRDF directly as node parameters so that
    // MoveGroupInterface builds the SO-ARM101 model
    {
        std::string desc_dir = ament_index_cpp::get_package_share_directory("lerobot_description");
        std::ifstream urdf_file(desc_dir + "/urdf/so101.urdf");
        if (urdf_file.is_open()) {
            std::stringstream ss;
            ss << urdf_file.rdbuf();
            node->declare_parameter("robot_description", ss.str());
        } else {
            RCLCPP_ERROR(node->get_logger(), "Could not open arm URDF file");
        }

        std::string moveit_dir = ament_index_cpp::get_package_share_directory("lerobot_moveit");
        std::ifstream srdf_file(moveit_dir + "/config/so101.srdf");
        if (srdf_file.is_open()) {
            std::stringstream ss;
            ss << srdf_file.rdbuf();
            node->declare_parameter("robot_description_semantic", ss.str());
        } else {
            RCLCPP_ERROR(node->get_logger(), "Could not open arm SRDF file");
        }
    }

    RCLCPP_INFO(node->get_logger(), "Contest 2 node started");

    // Robot pose object + subscriber
    RobotPose robotPose(0, 0, 0);
    auto amclSub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose",
        10,
        std::bind(&RobotPose::poseCallback, &robotPose, std::placeholders::_1)
    );

    // Initialize box coordinates
    Boxes boxes;
    if(!boxes.load_coords()) {
        RCLCPP_ERROR(node->get_logger(), "ERROR: could not load box coordinates");
        return -1;
    }

    for(size_t i = 0; i < boxes.coords.size(); ++i) {
        RCLCPP_INFO(node->get_logger(), "Box %zu coordinates: x=%.2f, y=%.2f, phi=%.2f",
                    i, boxes.coords[i][0], boxes.coords[i][1], boxes.coords[i][2]);
    }
    
    //Initialize arm controller
    ArmController armController(node);

    // Contest countdown timer
    auto start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    RCLCPP_INFO(node->get_logger(), "Starting contest - 300 seconds timer begins now!");

    //initialize variable values
    startup = true; 
    armSuccess = false;
    gripSuccess = false;

    // Execute strategy
    while(rclcpp::ok() && secondsElapsed <= 300) {
        rclcpp::spin_some(node);

        // Calculate elapsed time
        auto now = std::chrono::system_clock::now();
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();

        /***YOUR CODE HERE***/

        //Enter routine based on conditions
        //startup (pickup and detect our object
        if(startup){startup();}
        
        //calculate path to box
        //navigate to box
        //orient to april tag
        //drop into box
        if(putInBin){putInBin();} //we may want to move this inside of another routine that shifts the position of the robot relative to the april tag
        //return to start position



        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (secondsElapsed > 300) {
        RCLCPP_WARN(node->get_logger(), "Contest time limit reached!");
    }

    RCLCPP_INFO(node->get_logger(), "Contest 2 node shutting down");
    rclcpp::shutdown();
    return 0;

    bool startup;
    bool armSuccess;
    bool gripSuccess;
    char detectedClass;

}
