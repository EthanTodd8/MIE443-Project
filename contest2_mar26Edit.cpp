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
#include <map>
#include <array>
#include <string>
#include <iostream>

#include <optional>
                          
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "sensor_msgs/msg/joint_state.hpp" // ADDED: joint command publishing

#include <Eigen/Dense>
#include <Eigen/Geometry>

AprilTagDetector* tagDetector   = nullptr;
std::vector<int>  candidateTags = {0, 1, 2, 3, 4};

ArmController* armController = nullptr;

// ADDED: global joint command publisher
rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointCommandPub;

rclcpp::Node::SharedPtr node;
std::unique_ptr<YoloInterface> yoloDetector;

std::string detectedClass = "";
float startupObjectPose[7] = {};

bool aprilTagDetected(float secondsElapsed); 
std::optional<geometry_msgs::msg::Pose> getBinTagPose();
void orientForPickup();
void grab();
void putInBin();
bool isTargetObject(std::string name);
std::string startupArm();


double getYawFromQuaternion(const geometry_msgs::msg::Quaternion &q)
{
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
    return yaw;
}

std::string yoloDetectionOutput(std::string cameraName, float secondsElapsed, bool saveImage = false)
{
    static uint64_t lastYoloTime = 0;

    if (secondsElapsed >= lastYoloTime + 2) {
        lastYoloTime = secondsElapsed;
        std::string detected = yoloDetector->getObjectName(CameraSource::OAKD, saveImage);

        RCLCPP_INFO(node->get_logger(), "--- YOLO Detection (OAK-D Camera) ---");
        if (cameraName == "wrist") {
            detected = yoloDetector->getObjectName(CameraSource::WRIST, saveImage);
        }

        if(!detected.empty()) {
        float confidence = yoloDetector->getConfidence();
        RCLCPP_INFO(node->get_logger(), "Detected: %s (Confidence: %.2f)",
                detected.c_str(), confidence);
        } else{
        RCLCPP_INFO(node->get_logger(), "No object detected");
        }
        return detected;

    }
    return "";
}


bool aprilTagDetected(float secondsElapsed)
{
    static uint64_t lastPrintTime = 0;
    if (!tagDetector) return false;

    std::optional<geometry_msgs::msg::Pose> tagPose;

    if (secondsElapsed > lastPrintTime) {
        lastPrintTime = secondsElapsed;

        auto visible_tags = tagDetector->getVisibleTags(candidateTags);

        if (!visible_tags.empty()) {
            for (int tag_id : visible_tags) {
                auto tagPose = tagDetector->getTagPose(tag_id);

                if (tagPose.has_value()) {
                    RCLCPP_INFO(node->get_logger(),
                        "%s -> tag%d: pos(%.3f, %.3f, %.3f) ori(%.3f, %.3f, %.3f, %.3f)",
                        tagDetector->getReferenceFrame().c_str(), tag_id,
                        tagPose->position.x, tagPose->position.y, tagPose->position.z,
                        tagPose->orientation.x, tagPose->orientation.y, tagPose->orientation.z, tagPose->orientation.w);
                    return true;
                }
            }
        }
        else {
            RCLCPP_INFO(node->get_logger(), "No tags visible");
        }
    }
    return false;
}

std::optional<geometry_msgs::msg::Pose> getBinTagPose()
{
    if (!tagDetector) return std::nullopt;

    auto visible = tagDetector->getVisibleTags(candidateTags);

    for (int tag_id : visible) {
        auto pose = tagDetector->getTagPose(tag_id);

        if (pose.has_value()) {
            RCLCPP_INFO(node->get_logger(),
                "getBinTagPose: using tag%d at pos(%.3f, %.3f, %.3f)",
                tag_id, pose->position.x, pose->position.y, pose->position.z);
            return pose;
        }
    }
    return std::nullopt;
}

bool isTargetObject(std::string name)
{
    if (name == "bottle" ||
        name == "cup" ||
        name == "clock" ||
        name == "plant" ||
        name == "motorcycle")
    {
        return true;
    }

    return false;
}


std::vector<int> solveTSP(const std::vector<std::vector<double>> &distances, std::vector<int> order)
{
    double best_distance = std::numeric_limits<double>::max();
    std::vector<int> best_order;

    do
    {
        double total = 0.0;
        int current = 0;

        for (int next : order)
        {
            total += distances[current][next];
            current = next;
        }

        total += distances[current][0];

        if (total < best_distance)
        {
            best_distance = total;
            best_order = order;
        }

    } while (std::next_permutation(order.begin(), order.end()));

    return best_order;
}

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>("contest2");

    yoloDetector = std::make_unique<YoloInterface>(node);
    
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

    RobotPose robotPose(0, 0, 0);
    auto amclSub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose",
        10,
        std::bind(&RobotPose::poseCallback, &robotPose, std::placeholders::_1)
    );

    Boxes boxes;
    if(!boxes.load_coords()) {
        RCLCPP_ERROR(node->get_logger(), "ERROR: could not load box coordinates");
        return -1;
    }

    for(size_t i = 0; i < boxes.coords.size(); ++i) {
        RCLCPP_INFO(node->get_logger(), "Box %zu coordinates: x=%.2f, y=%.2f, phi=%.2f",
                    i, boxes.coords[i][0], boxes.coords[i][1], boxes.coords[i][2]);
    }

    ArmController armController_address(node); 
    armController = &armController_address;

    Navigation navigator(node);
    navigator.moveToGoal(robotPose.x, robotPose.y, robotPose.phi-M_PI); // spin in half circle to establish good localization
    
    AprilTagDetector aprilDetector(node);
    tagDetector = &aprilDetector;

    RCLCPP_INFO(node->get_logger(), "AprilTagDetector initialised (ref frame: %s, candidate tags: 0-4)",
            aprilDetector.getReferenceFrame().c_str());

    auto start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    RCLCPP_INFO(node->get_logger(), "Starting contest - 300 seconds timer begins now!");

    #pragma region PATH PLANNING TO BOXES - Isabelle
    int num_boxes = boxes.coords.size();
    int num_nodes = num_boxes + 1;
    std::vector<std::vector<double>> box_distances(num_nodes, std::vector<double>(num_nodes, 0.0));
    
    std::vector<std::array<double,3>> nodes;
    nodes.push_back({robotPose.x, robotPose.y, robotPose.phi});
    for (auto &box : boxes.coords) { nodes.push_back({box[0], box[1], box[2]}); }

    for (int i = 1; i < num_nodes; i++)
    {
        double buffer = 0.15;
        nodes[i][0] -= buffer * cos(nodes[i][2] + M_PI);
        nodes[i][1] -= buffer * sin(nodes[i][2] + M_PI);
        nodes[i][2] = nodes[i][2] + M_PI;
    }
    
    for (int start = 0; start < num_nodes; start++)
    {
        for (int goal = 0; goal < num_nodes; goal++)
        {
            if (start == goal)
                continue;
            double start_x = nodes[start][0];
            double start_y = nodes[start][1];
            double goal_x = nodes[goal][0];
            double goal_y = nodes[goal][1];
            double total_distance = sqrt(pow(goal_x - start_x, 2) + pow(goal_y - start_y, 2));
            box_distances[start][goal] = total_distance;;
            RCLCPP_INFO(node->get_logger(), "Path %d -> %d has %.2f distance", start, goal, box_distances[start][goal]);        
        }
    }

    std::vector<int> order;
    for (int i = 1; i <= num_boxes; i++) { order.push_back(i); } 
    std::vector<int> optimal_order = solveTSP(box_distances, order);
    std::vector<int> full_route;
    full_route.push_back(0);

    for (int box : optimal_order)
    {
        full_route.push_back(box);
    }

    full_route.push_back(0);

    RCLCPP_INFO(node->get_logger(), "Best route:");

    int current = 0;
    double total_distance = 0.0;

    for (int box : optimal_order)
    {
        RCLCPP_INFO(node->get_logger(), " %d -> %d  (%.2f m)", current, box, box_distances[current][box]);
        total_distance += box_distances[current][box];
        current = box;
    }

    RCLCPP_INFO(node->get_logger(), " %d -> %d  (%.2f m)  [RETURN]", current, 0, box_distances[current][0]);
    total_distance += box_distances[current][0];
    RCLCPP_INFO(node->get_logger(), "Total route length: %.2f m", total_distance);

    #pragma endregion END PATH PLANNING TO BOXES

    bool startup = true; 
    bool startYolo = false;
    bool foundAprilTag = false;
    bool startPickup = false;
    bool startTravelling = false;
    bool armSuccess = false;
    bool gripSuccess = false;
    startupObjectPose[0] = 0.099;
    startupObjectPose[1] = -0.009;
    startupObjectPose[2] = 0.155;

    int currentBoxIndex = 0;
    bool arrivedAtGoal = false;
    bool objectFound = false;
    bool detectedObjectFromList = false;


    double start_x = robotPose.x;
    double start_y = robotPose.y;
    double start_phi = robotPose.phi;

    std::string objectName = "";

    std::map<std::string, std::array<double, 3>> boxItemCoordinates;
    boxItemCoordinates["cup"] = {0.0, 0.0, 0.0};
    boxItemCoordinates["plant"] = {0.0, 0.0, 0.0};
    boxItemCoordinates["motorcycle"] = {0.0, 0.0, 0.0};
    boxItemCoordinates["bottle"] = {0.0, 0.0, 0.0};
    boxItemCoordinates["clock"] = {0.0, 0.0, 0.0};

    std::map<std::string, float> boxItemConfidences;
    boxItemConfidences["cup"] = 0.0;
    boxItemConfidences["plant"] = 0.0;
    boxItemConfidences["motorcycle"] = 0.0;
    boxItemConfidences["bottle"] = 0.0;
    boxItemConfidences["clock"] = 0.0;


    double goal_x, goal_y, goal_phi;

    std::vector<std::array<double,3>> item_locations;

    //std::string detected = (yoloDetectionOutput("oakd", secondsElapsed, true));
    uint64_t startAprilTagSearch = 0.0;
    uint64_t startYoloSearchTime = 0.0;

    std::string detected = "";

    #pragma region WHILE LOOP
    while(rclcpp::ok() && secondsElapsed <= 300) {
        RCLCPP_INFO(node->get_logger(), "Main loop - seconds elapsed: %d", secondsElapsed);
        rclcpp::spin_some(node);

        auto now = std::chrono::system_clock::now();
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();
        
        if (!startTravelling) {
            
            if(startup) {
                RCLCPP_INFO(node->get_logger(), "Starting arm sequence to pick up object");
                //objectName = startupArm();
                armController->openGripper();
                armController->moveToCartesianPose(0.125, -0.022, 0.221, -0.095, 0.217, 2.742);
                armController->moveToCartesianPose(0.124, -0.021, 0.222, -0.239, -0.016, 1.524); 
                armController->moveToCartesianPose(0.175, -0.006, 0.178, 0.319, -0.023, 1.749);  

                startup = false;
                startYolo = true;
                startYoloSearchTime = secondsElapsed;
            }
            else if (startYolo && secondsElapsed - startYoloSearchTime < 20.0) {
                RCLCPP_INFO(node->get_logger(), "Starting YOLO detection loop to identify object in gripper");
                detected = yoloDetectionOutput("wrist", secondsElapsed, true);
                RCLCPP_INFO(node->get_logger(), "Wrist camera detection: %s", detected.c_str());
                if (!detected.empty() && isTargetObject(detected)) {
                    startYolo = false;
                    startPickup = true;
                    objectName = detected;
                    RCLCPP_INFO(node->get_logger(), "Identified object in gripper as %s, proceeding to pickup sequence", objectName.c_str());

                }
            }
            else if ((startPickup || secondsElapsed - startYoloSearchTime <= 20.0) && startTravelling == false) {
                RCLCPP_INFO(node->get_logger(), "Starting pickup sequence for detected object: %s", detected.c_str());
                if (!detected.empty() && isTargetObject(detected)) {
                    armController->moveToCartesianPose(0.125, -0.022, 0.221, -0.095, 0.217, 2.742);
                    if (detected == "cup") {
                        // inside cup
                        armController->moveToCartesianPose(0.128, 0.014, 0.172, -0.084, -0.006, 1.835);
                        armController->closeGripper();
                    }
                    else {
                        // default grab
                        armController->moveToCartesianPose(0.149, -0.019, 0.154, 0.178, -0.017, 1.696); 
                        armController->closeGripper();
                    }
                    armController->moveToCartesianPose(0.125, -0.022, 0.221, -0.095, 0.217, 2.742);
                    
                }
                else {
                    // default grab
                    armController->moveToCartesianPose(0.149, -0.019, 0.154, 0.178, -0.017, 1.696);
                    armController->closeGripper();
                    armController->moveToCartesianPose(0.125, -0.022, 0.221, -0.095, 0.217, 2.742);
                }
                // hold position
                armController->moveToCartesianPose(0.021, -0.022, 0.287, -1.523, 0.094, 0.042);
                startPickup = false;
                startTravelling = true;
            }  
        }
        else if (startTravelling) {
            RCLCPP_INFO(node->get_logger(), "Starting navigation and box search sequence");

            if (currentBoxIndex < static_cast<int>(full_route.size()) - 1 && !arrivedAtGoal)  // if the location index is not the last (home position) and we have not arrived at a box
            {
                // NAVIGATION TO NEXT LOCATION
                int goal_node = full_route[currentBoxIndex + 1];

                goal_x = nodes[goal_node][0];
                goal_y = nodes[goal_node][1];
                goal_phi = nodes[goal_node][2];

                if (goal_node == 0) {
                    goal_x = start_x;
                    goal_y = start_y;
                    goal_phi = start_phi;
                } 

                RCLCPP_INFO(node->get_logger(), "Navigating to node %d at (%.2f, %.2f, %.2f)", goal_node, goal_x, goal_y, goal_phi);
                arrivedAtGoal = navigator.moveToGoal(goal_x, goal_y, goal_phi);
                RCLCPP_INFO(node->get_logger(), "Arrived at goal: %s", arrivedAtGoal ? "true" : "false");
                if (arrivedAtGoal) { // if we have arrived at the box location, start searching for april tag
                    currentBoxIndex++;
                    startAprilTagSearch = secondsElapsed;
                }
            }
            else if (arrivedAtGoal && !objectFound && currentBoxIndex < (int)full_route.size() - 1) // if we've arrived at the box but no object has been found yet, and we're not at the final location, start search for april tag and object
            {  
                RCLCPP_INFO(node->get_logger(), "Searching for AprilTag at box %d", full_route[currentBoxIndex]);
                
                // if we have been at the object for less than ten seconds and we haven't found the april tag yet, keep trying. After 10 seconds, try yolo detection whether it's found or not.
                if (!foundAprilTag && secondsElapsed - startAprilTagSearch < 10) {
                    if (aprilTagDetected(secondsElapsed)) // returns true if tag detected
                    {
                        auto tagPose = getBinTagPose();
                        if (tagPose.has_value()) // if we get the tag pose, adjust position of robot to be in optimal position for pickup based on tag pose and orientation
                        {
                            RCLCPP_INFO(node->get_logger(), "AprilTag detected at box %d, adjusting position for pickup", full_route[currentBoxIndex]);
                            double tag_x = tagPose->position.x;
                            double tag_y = tagPose->position.y;
                            double tag_yaw = getYawFromQuaternion(tagPose->orientation);
                            const double desired_distance = 0.5;

                            double target_x_robot = tag_x - desired_distance * cos(tag_yaw);
                            double target_y_robot = tag_y - desired_distance * sin(tag_yaw);
                            double target_yaw_robot = atan2(tag_y, tag_x);

                            double goal_x = robotPose.x + cos(robotPose.phi) * target_x_robot - sin(robotPose.phi) * target_y_robot;
                            double goal_y = robotPose.y + sin(robotPose.phi) * target_x_robot + cos(robotPose.phi) * target_y_robot;
                            double goal_phi = robotPose.phi + target_yaw_robot;
                            goal_phi = atan2(sin(goal_phi), cos(goal_phi));

                            RCLCPP_INFO(node->get_logger(), "Adjusting position near AprilTag: goal (%.2f, %.2f, %.2f)", goal_x, goal_y, goal_phi);
                            navigator.moveToGoal(goal_x, goal_y, goal_phi);

                            foundAprilTag = true;
                            startYoloSearchTime = secondsElapsed; // start yolo search after adjusting position based on april tag
                        }
                    }
                    else {
                        RCLCPP_INFO(node->get_logger(), "AprilTag not detected at goal pose, continuing search...");
                        startYoloSearchTime = secondsElapsed; // start yolo search after 10 seconds of searching for april tag, whether we found the tag or not
                    }
                }
                // EVEN IF THE APRIL TAG IS NOT DETECTED, WE SHOULD STILL TRY TO DETECT THE OBJECT WITH YOLO IN CASE THE APRIL TAG IS NOT VISIBLE BUT THE OBJECT IS
                else {
                    RCLCPP_INFO(node->get_logger(), "Attempting YOLO detection for object in case AprilTag is not visible", full_route[currentBoxIndex]);
                    // DEPENDING ON OUR VIEW OF THE OBJECT, WE MIGHT HAVE TO BACK UP A BIT ////////////////////////////////////////////////////////////////////////////////////////
                    // navigator.moveToGoal(goal_x - 0.1 * cos(goal_phi), goal_y - 0.1 * sin(goal_phi), goal_phi); // back up a bit to get better view of object for yolo detection
                    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                    // try yolo detection for 10 seconds of until we find an object that matched one of the detected class objects.
                    if (secondsElapsed - startYoloSearchTime < 10 && !detectedObjectFromList) {
                        std::string detected = yoloDetectionOutput("oakd", secondsElapsed, true);
                        RCLCPP_INFO(node->get_logger(), "YOLO detection at box %d: %s", full_route[currentBoxIndex], detected.c_str());

                        // check if whatever was detected is one of the target objects. If yes, add current coordinates to boxItemCoordiantes and confidence to boxItemConfidences.
                        if (!detected.empty() && isTargetObject(detected)) {
                            detectedObjectFromList = true; // if detected object is in the list, we will move on to the next box, whether or not it matched the object we picked up.
                            std::array<double, 3> coords = {goal_x, goal_y, goal_phi};
                            boxItemCoordinates[detected] = coords;
                            boxItemConfidences[detected] = yoloDetector->getConfidence(); 
                            
                            // WRITE BOX ITEM COORDINATES AND CONFIDENCES TO A FILE FOR FUTURE REFERENCE - WRITES EVERY ROUND JUST IN CASE ROBOT DIES PARTWAY THROUGH AND WE LOSE ALL THE COORDINATES
                            std::string filePath = "/home/turtlebot/ros2_ws/boxItems.txt";
                            std::ofstream outfile(filePath);

                            if (!outfile) {
                                std::cerr << "Error opening file!" << std::endl;
                                return;
                            }

                            outfile << "Object detected from wrist camera: " << objectName << std::endl;
                            outfile << "----------------------------------------" << std::endl;

                            for (const auto& pair : boxItemCoordinates) {
                                const std::string& name = pair.first;
                                const auto& coords = pair.second;

                                float confidence = 0.0f;
                                auto it = boxItemConfidences.find(name);
                                if (it != boxItemConfidences.end()) {
                                    confidence = it->second;
                                }

                                outfile << name << ": "
                                        << coords[0] << ", "
                                        << coords[1] << ", "
                                        << coords[2]
                                        << " | confidence: "
                                        << confidence
                                        << std::endl;
                            }

                            outfile.close();

                            // determine if detected object matches object we picked up
                            RCLCPP_INFO(node->get_logger(), "Detected object %s at box %d, saving coordinates (%.2f, %.2f, %.2f)", detected.c_str(), full_route[currentBoxIndex], coords[0], coords[1], coords[2]);
                            if (detected == objectName) { 
                                objectFound = true; 
                            }
                        }    
                    }
                    else {
                        if (objectFound || currentBoxIndex == (int)full_route.size() - 2) { // if object was found or we are at the last box, drop the object in the bin at current location
                            if (!objectFound) {
                                RCLCPP_WARN(node->get_logger(), "Object not found with YOLO at box %d after 10 seconds, but we are at the last box so we will drop the object here.", full_route[currentBoxIndex]);
                            }
                            RCLCPP_INFO(node->get_logger(), "Object in gripper matches detected object at box %d! Dropping object.", full_route[currentBoxIndex]);
                            armController->moveToCartesianPose(0.041, -0.308, 0.275, -1.236, 0.095, 0.103); // stick out arm
                            armController->openGripper(); // drop object
                            std::this_thread::sleep_for(std::chrono::seconds(2));
                            armController->moveToCartesianPose(0.021, -0.022, 0.287, -1.523, 0.094, 0.042); // move arm back in 
                            objectFound = false; // reset object found for next box
                        }

                        detectedObjectFromList = false; // reset detected object from list for next box
                    }                    
                }        

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    #pragma endregion

    if (secondsElapsed > 300) {
        RCLCPP_WARN(node->get_logger(), "Contest time limit reached!");
    }

    RCLCPP_INFO(node->get_logger(), "Contest 2 node shutting down");
    rclcpp::shutdown();
    return 0;
}