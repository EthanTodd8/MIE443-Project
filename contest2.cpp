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
    //bring the arm to a start position, orienting the wrist camera downwards towards the object of interest
    RCLCPP_INFO(note->get_logger(), "orientin");
    orientForPickup();

    //call a function that captures an image from the wrist camera/adjust until object is detected and arm is above
    RCLCPP_INFO(note->get_logger(), "detecting unknown ");
    //add that in here

    //call a function that moves the arm to the location of the object
    RCLCPP_INFO(note->get_logger(), "grabbin");
    grab();

    startup = false;

}

void orientForPickup()
{
    bool checkedLeft = false;
    bool checkedRight = false;
    bool checkedFront = false;

    //move arm to the starting position
     armController.moveToCartesianPose(0.043, 0.199, 0.313, -0.471, -0.557, 0.564, -0.387); //need to change this pose pending simulation testing

     //object detection using the wrist camera and determine and save class
    captureAndDetect("Wrist", true);
    detectedClass = latest_class_name_;

    //evaluate if the object is detected in the wrist camera, if not adjust the arm pose     
        while(detectedClass == '0') {
            RCLCPP_INFO(note->get_logger(), "Object not detected in wrist camera, adjusting arm pose");
            //add in code to adjust the arm pose here, maybe a for loop that iterates through a set of poses until the object is detected?

            if (detectedClass == '0' && !checkedLeft) {
                //adjust arm pose to the left
                RCLCPP_INFO(note->get_logger(), "Object not detected, adjusting arm pose to the left");
                //add in code to adjust the arm pose to the left here
                checkedLeft = true;
            }
            else if (detectedClass == '0' && !checkedRight) {
                //adjust arm pose to the right
                RCLCPP_INFO(note->get_logger(), "Object not detected, adjusting arm pose to the right");
                //add in code to adjust the arm pose to the right here
                checkedRight = true;
            }
            else if (detectedClass == '0' && !checkedFront) {
                //adjust arm pose forward
                RCLCPP_INFO(note->get_logger(), "Object not detected, adjusting arm pose forward");
                //add in code to adjust the arm pose forward here
                checkedFront = true;
            }

            captureAndDetect("Wrist", true);
            detectedClass = latest_class_name_;
        }

    //calculate or save the pose the arm needs to go to for object pickup
    //check what type of object it is and adjust the arm pose accordingly if needed
    //update this section based on how we need to adjust the arm pose based on the object class
    /*if (detectedClass == "bottle"){
        startupArmPose = {0.0, 0.0, 0.0, 0.0, 0.0}; //need to change this pose based on how we calculate position?
    }
    else if (detectedCLass =="") {
    
    startupArmPose = {0.0, 0.0, 0.0, 0.0, 0.0}; //need to change this pose based on how we calculate position?

    }
    else if (detectedCLass =="") {
    
    startupArmPose = {0.0, 0.0, 0.0, 0.0, 0.0}; //need to change this pose based on how we calculate position?
    
    }
    else if (detectedCLass =="") {
    
    startupArmPose = {0.0, 0.0, 0.0, 0.0, 0.0}; //need to change this pose based on how we calculate position?
    
    }
    else if (detectedCLass =="") {
    
    startupArmPose = {0.0, 0.0, 0.0, 0.0, 0.0}; //need to change this pose based on how we calculate position?
    
    }
    */
    
}

void grab() {

    //move the arm to location 1 - pickup the object
    armController.openGripper();
    std::this_thread::sleep_for(std::chrono::seconds(2));

    RCLCPP_INFO(note->get_logger(), "Moving arm to grab unknown object");
    //doing the movement in two steps, once to hover above the object and the second to get close enough to grasp
    armController.moveToCartesianPose(startupArmPose[0], startupArmPose[1], startupArmPose[2], startupArmPose[3], startupArmPose[4], startupArmPose[5], startupArmPose[6]); //need to change this pose pending simulation testing
    armController.moveToCartesianPose(startupArmPose[0], startupArmPose[1], startupArmPose[2], startupArmPose[3], startupArmPose[4], startupArmPose[5], startupArmPose[6]); //need to change this pose pending simulation testing
    
    //close the gripper to grab the object
    RCLCPP_INFO(node-.get_logger(), "Grabbing the unknown object");
    armController.closeGripper();
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    //move the arm to location 2 to pick it up and orient to later drop it in
    RCLCPP_INFO(note->get_logger(), "Moving arm to position for wrist camera object detection");
    //change the line below to adjust specifically what coordinates it is that we need to change
    armController.moveToCartesianPose(0.043, 0.199, 0.313, -0.471, -0.557, 0.564, -0.387); //need to change this pose pending simulation testing

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
    startupArmPose = {0.0, 0.0, 0.0, 0.0, 0.0}; ///initialize the arm pose for locating and grabbing the object as a 'neutral' pose


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
    float array startupArmPose[7];

}
