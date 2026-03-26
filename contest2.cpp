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

#pragma region HELPER FUNCTIONS FOR CONVERTING FROM JOINT ANGLES TO CARTESIAN POSE - ISABELLE

struct CartesianPose
{
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};

Eigen::Matrix4d Txyz(double x, double y, double z)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T(0, 3) = x;
    T(1, 3) = y;
    T(2, 3) = z;
    return T;
}

Eigen::Matrix4d Rx(double a)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    double c = std::cos(a), s = std::sin(a);
    T(1,1) = c;  T(1,2) = -s;
    T(2,1) = s;  T(2,2) =  c;
    return T;
}

Eigen::Matrix4d Ry(double a)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    double c = std::cos(a), s = std::sin(a);
    T(0,0) =  c; T(0,2) = s;
    T(2,0) = -s; T(2,2) = c;
    return T;
}

Eigen::Matrix4d Rz(double a)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    double c = std::cos(a), s = std::sin(a);
    T(0,0) = c;  T(0,1) = -s;
    T(1,0) = s;  T(1,1) =  c;
    return T;
}

Eigen::Matrix4d Rrpy(double roll, double pitch, double yaw)
{
    return Rz(yaw) * Ry(pitch) * Rx(roll);
}

Eigen::Matrix4d jointT(double x, double y, double z,
                       double roll, double pitch, double yaw,
                       double theta)
{
    // URDF joint transform:
    // T(origin xyz) * R(origin rpy) * Rz(theta)
    return Txyz(x, y, z) * Rrpy(roll, pitch, yaw) * Rz(theta);
}

CartesianPose matrixToPoseRPY(const Eigen::Matrix4d& T)
{
    CartesianPose pose;
    pose.x = T(0,3);
    pose.y = T(1,3);
    pose.z = T(2,3);

    Eigen::Matrix3d R = T.block<3,3>(0,0);

    // ZYX / getRPY-compatible extraction
    pose.yaw   = std::atan2(R(1,0), R(0,0));
    pose.pitch = std::asin(-R(2,0));
    pose.roll  = std::atan2(R(2,1), R(2,2));

    return pose;
}

CartesianPose fkSo101Gripper(const std::array<double, 5>& q)
{
    const double q1 = q[0];
    const double q2 = q[1];
    const double q3 = q[2];
    const double q4 = q[3];
    const double q5 = q[4];

    Eigen::Matrix4d T1 = jointT(
        0.0207909, -0.0230745, 0.0948817,
        -3.14159,  6.03684e-16, 1.5708,
        q1
    );

    Eigen::Matrix4d T2 = jointT(
        -0.0303992, -0.0182778, -0.0542,
        -1.5708, -1.5708, 0.0,
        q2
    );

    Eigen::Matrix4d T3 = jointT(
        -0.11257, -0.028, 2.46331e-16,
        -1.22818e-15, 5.75928e-16, 1.5708,
        q3
    );

    Eigen::Matrix4d T4 = jointT(
        -0.1349, 0.0052, 1.65232e-16,
        3.2474e-15, 2.86219e-15, -1.5708,
        q4
    );

    Eigen::Matrix4d T5 = jointT(
        0.0, -0.0611, 0.0181,
        1.5708, -9.38083e-08, 3.14159,
        q5
    );

    Eigen::Matrix4d T = T1 * T2 * T3 * T4 * T5;
    return matrixToPoseRPY(T);
}

CartesianPose fkSo101Jaw(const std::array<double, 6>& q)
{
    const double q1 = q[0];
    const double q2 = q[1];
    const double q3 = q[2];
    const double q4 = q[3];
    const double q5 = q[4];
    const double q6 = q[5];

    Eigen::Matrix4d T1 = jointT(
        0.0207909, -0.0230745, 0.0948817,
        -3.14159,  6.03684e-16, 1.5708,
        q1
    );

    Eigen::Matrix4d T2 = jointT(
        -0.0303992, -0.0182778, -0.0542,
        -1.5708, -1.5708, 0.0,
        q2
    );

    Eigen::Matrix4d T3 = jointT(
        -0.11257, -0.028, 2.46331e-16,
        -1.22818e-15, 5.75928e-16, 1.5708,
        q3
    );

    Eigen::Matrix4d T4 = jointT(
        -0.1349, 0.0052, 1.65232e-16,
        3.2474e-15, 2.86219e-15, -1.5708,
        q4
    );

    Eigen::Matrix4d T5 = jointT(
        0.0, -0.0611, 0.0181,
        1.5708, -9.38083e-08, 3.14159,
        q5
    );

    Eigen::Matrix4d T6 = jointT(
        0.0202, 0.0188, -0.0234,
        1.5708, -5.14108e-17, -1.38655e-14,
        q6
    );

    Eigen::Matrix4d T = T1 * T2 * T3 * T4 * T5 * T6;
    return matrixToPoseRPY(T);
}

#pragma endregion

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

std::string startupArm()
{
    RCLCPP_INFO(node->get_logger(), "orienting arm for pickup");
    orientForPickup();

    detectedClass = yoloDetectionOutput("wrist", 0.0, true);

    //RCLCPP_INFO(node->get_logger(), "grabbing object");
    //grab();

    return detectedClass;
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

void orientForPickup()
{
    armController->openGripper();
    armController->moveToCartesianPose(0.114, -0.022, 0.174, 0.015, -0.035, 2.749); //position direction above object for better YOLO detection
    
    armController->moveToCartesianPose(0.155, -0.030, 0.164, 0.445, 0.005, 1.506); //this pose reliably detects cup

    //float scanZ = 0.10;

    // float armPose[3][6] = {{startupObjectPose[0], startupObjectPose[1], scanZ, -0.006, -0.000, 1.658},
    //                             {startupObjectPose[0]- rotation, startupObjectPose[1] + rotation, scanZ,-0.006, -0.000, 1.658 },
    //                             {startupObjectPose[0] + rotation, startupObjectPose[1] - rotation, scanZ, -0.006, -0.000, 1.658  }};

    //armController->moveToCartesianPose(startupObjectPose[0], startupObjectPose[1], startupObjectPose[2]+0.244, -0.471, -0.557,  0.564, -0.387);
    //armController->moveToCartesianPose(startupObjectPose[0], startupObjectPose[1], startupObjectPose[2]+0.170, -0.471, -0.557,  0.564, -0.387);
    //armController->moveToCartesianPose(startupObjectPose[0], startupObjectPose[1], startupObjectPose[2]+0.100, -0.471, -0.557,  0.564, -0.387);
    // armController->moveToCartesianPose(armPose[0][0], armPose[0][1], armPose[0][2], armPose[0][3], armPose[0][4], armPose[0][5]);

    // for (int i=1; i<3; i++){    
    //     detectedClass = yoloDetector->getObjectName(CameraSource::WRIST, true);
    //     float confidence = yoloDetector->getConfidence();

    //     if (!isTargetObject(detectedClass)&&confidence > 0.5){
    //         armController->moveToCartesianPose(armPose[i][0], armPose[i][1], armPose[i][2], armPose[i][3], armPose[i][4], armPose[i][5]); 
    //     }
    //     else break;
    // }
    detectedClass = yoloDetector->getObjectName(CameraSource::WRIST, true);
    return;
}

void grab() {

    float scanZ     = 0.10;
    float approachZ = 0.14;

    armController->openGripper();

    armController->moveToCartesianPose(0.114, -0.022, 0.174, 0.015, -0.035, 2.749); //position direction above object for better YOLO detection


    // RCLCPP_INFO(node->get_logger(), "Moving arm to grab unknown object");

    // if (detectedClass == "bottle"){
    // armController->moveToCartesianPose(startupObjectPose[0], startupObjectPose[1], scanZ, -0.006, -0.000, 1.658);
    // armController->moveToCartesianPose(startupObjectPose[0], startupObjectPose[1], startupObjectPose[2], -0.006, -0.000, 1.658);
    // }
    // else if (detectedClass =="plant") {
    // armController->moveToCartesianPose(startupObjectPose[0], startupObjectPose[1], scanZ, -0.006, -0.000, 1.658);
    // armController->moveToCartesianPose(startupObjectPose[0], startupObjectPose[1], startupObjectPose[2], -0.006, -0.000, 1.658);    
    // }
    // else if (detectedClass =="motorcycle") {
    // armController->moveToCartesianPose(startupObjectPose[0], startupObjectPose[1], scanZ, -0.006, -0.000, 1.658);
    // armController->moveToCartesianPose(startupObjectPose[0], startupObjectPose[1], startupObjectPose[2], -0.006, -0.000, 1.658);
    // }
    // else if (detectedClass =="clock") {
    // armController->moveToCartesianPose(startupObjectPose[0], startupObjectPose[1], scanZ, -0.006, -0.000, 1.658);
    // armController->moveToCartesianPose(startupObjectPose[0], startupObjectPose[1], startupObjectPose[2], -0.006, -0.000, 1.658);
    // }
    // else if (detectedClass =="cup") {
    //armController->moveToCartesianPose(startupObjectPose[0], startupObjectPose[1], startupObjectPose[2], startupObjectPose[3], startupObjectPose[4], startupObjectPose[5], startupObjectPose[6]);
    // }

    if (detectedClass == "cup"){
        armController->moveToCartesianPose(0.108, -0.001, 0.169, -0.205, -0.002, 1.813); 
        armController->closeGripper();
        armController->moveToCartesianPose(0.114, -0.022, 0.174, 0.015, -0.035, 2.749);
    }
    else {
        armController->moveToCartesianPose(0.114, -0.031, 0.168, -0.016, -0.018, 1.266); 
        armController->closeGripper();
        armController->moveToCartesianPose(0.114, -0.022, 0.174, 0.015, -0.035, 2.749);
    }

    // RCLCPP_INFO(node->get_logger(), "Moving arm to position to later drop in bin");
    // armController->moveToCartesianPose(0.043, 0.199, 0.313, -0.471, -0.557,  0.564, -0.387);
}

void putInBin()
{
    RCLCPP_INFO(node->get_logger(), "putInBin: locating bin via AprilTag...");

    auto tagPose = getBinTagPose();

    if (!tagPose.has_value()) {
        RCLCPP_WARN(node->get_logger(), "putInBin: no AprilTag visible — dropping at fallback pose");
        armController->moveToCartesianPose(0.300, 0.000, 0.400, -0.471, -0.557, 0.564, -0.387);
        armController->openGripper();
        std::this_thread::sleep_for(std::chrono::seconds(2));
        armController->moveToCartesianPose(0.043, 0.199, 0.313, -0.471, -0.557, 0.564, -0.387);
        return;
    }

    constexpr double HOVER_Z      = 0.15;
    constexpr double BIN_X_OFFSET = 0.05;

    double target_x = tagPose->position.x + BIN_X_OFFSET;
    double target_y = tagPose->position.y;
    double target_z = tagPose->position.z + HOVER_Z;

    constexpr double ORI_X = -0.657;
    constexpr double ORI_Y = 0.000;
    constexpr double ORI_Z =  -0.000;
    constexpr double ORI_W = 0.754;

    RCLCPP_INFO(node->get_logger(), "putInBin: moving above bin at (%.3f, %.3f, %.3f)", target_x, target_y, target_z);

    bool success = armController->moveToCartesianPose(target_x, target_y, target_z, ORI_X, ORI_Y, ORI_Z, ORI_W);

    if (!success) {
        RCLCPP_ERROR(node->get_logger(), "putInBin: hover pose unreachable — aborting drop");
        return;
    }

    double drop_z = target_z - 0.05;
    armController->moveToCartesianPose(target_x, target_y, drop_z, ORI_X, ORI_Y, ORI_Z, ORI_W);

    RCLCPP_INFO(node->get_logger(), "putInBin: releasing object");
    armController->openGripper();
    std::this_thread::sleep_for(std::chrono::seconds(2));

    RCLCPP_INFO(node->get_logger(), "putInBin: retracting arm");
    armController->moveToCartesianPose(target_x, target_y, target_z, ORI_X, ORI_Y, ORI_Z, ORI_W);
    armController->moveToCartesianPose(0.043, 0.199, 0.313, -0.471, -0.557, 0.564, -0.387);

    RCLCPP_INFO(node->get_logger(), "putInBin: complete");
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

    // ADDED: initialize joint command publisher
    //jointCommandPub = node->create_publisher<sensor_msgs::msg::JointState>("/joint_commands", 10);
    
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
        double buffer = 0.25;
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
    bool armSuccess = false;
    bool gripSuccess = false;
    startupObjectPose[0] = 0.099;
    startupObjectPose[1] = -0.009;
    startupObjectPose[2] = 0.155;

    int currentBoxIndex = 0;
    bool arrivedAtGoal = false;
    bool objectFound = false;


    double start_x = robotPose.x;
    double start_y = robotPose.y;
    double start_phi = robotPose.phi;

    std::string objectName = "";
    std::map<std::string, std::array<double, 3>> boxItemCoordinates;
    boxItemCoordinates["bottle"] = {0.0, 0.0, 0.0};
    boxItemCoordinates["plant"] = {0.0, 0.0, 0.0};
    boxItemCoordinates["motorcycle"] = {0.0, 0.0, 0.0};
    boxItemCoordinates["clock"] = {0.0, 0.0, 0.0};
    boxItemCoordinates["clock"] = {0.0, 0.0, 0.0};

    double goal_x, goal_y, goal_phi;

    std::vector<std::array<double,3>> item_locations;

    //std::string detected = (yoloDetectionOutput("oakd", secondsElapsed, true));
    uint64_t startAprilTagSearch = 0.0;

    #pragma region WHILE LOOP
    while(rclcpp::ok() && secondsElapsed <= 300) {
        //RCLCPP_INFO(node->get_logger(), "Main loop - seconds elapsed: %d", secondsElapsed);
        rclcpp::spin_some(node);

        auto now = std::chrono::system_clock::now();
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();

        //armController->moveToCartesianPose(0.300, 0.000, 0.400, -0.471, -0.557, 0.564, -0.387);

        //joints: [-1.5933, -0.8909, 0.7, 1.600, 2.7448, -0.0774]
        //xyz: 0.114, -0.022, 0.174
        //rpy: 0.015, -0.035, 2.749

        ////////////////////////////////////////////////////////////////////////
        /////////////////////////TEST THE FOLLOWING CODE////////////////////////
        // example using of joint angles converter to cartesian pose;
        // std::array<double, 6> joints = {-1.5933, -0.8909, 0.7, 1.600, 2.7448, -0.0774};

        // CartesianPose jawPose = fkSo101Jaw(joints);

        // RCLCPP_INFO(node->get_logger(),
        //     "Jaw pose -> x: %.3f y: %.3f z: %.3f roll: %.3f pitch: %.3f yaw: %.3f",
        //     jawPose.x, jawPose.y, jawPose.z,
        //     jawPose.roll, jawPose.pitch, jawPose.yaw);

        // moveToCartesianPose(jawPose.x, jawPose.y, jawPose.z, jawPose.roll, jawPose.pitch, jawPose.yaw);
        ////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////

        //joints: [-1.5933, -0.8909, 0.6, 1.600, 2.7448, -0.0774]
        //xyz: 0.124, -0.023, 0.224
        //xyz: 0.124, -0.023, 0.224

        //  TEST CODE //
        // call yolo from wrist
        // std::string detected = yoloDetectionOutput("wrist", secondsElapsed, true);

        
        if(startup) {
            objectName = startupArm();
            startup = false;
        }

        
        
        else if (currentBoxIndex < static_cast<int>(full_route.size()) - 1 && !arrivedAtGoal && !objectFound) 
        {
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
            if (arrivedAtGoal) {
                currentBoxIndex++;
                startAprilTagSearch = secondsElapsed;
            }
        }
        else if (arrivedAtGoal && !objectFound && currentBoxIndex < (int)full_route.size() - 1) 
        {  
            RCLCPP_INFO(node->get_logger(), "Searching for AprilTag at box %d", full_route[currentBoxIndex]);
            if (aprilTagDetected(secondsElapsed))
            {
                auto tagPose = getBinTagPose();

                if (tagPose.has_value())
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
                }

                RCLCPP_INFO(node->get_logger(), "Bin AprilTag confirmed — preparing to drop object");

                std::string detected = (yoloDetectionOutput("oakd", secondsElapsed, true));
                if (!detected.empty()) {
                    if (boxItemCoordinates.find(detected) != boxItemCoordinates.end()) {
                        std::array<double, 3> coords = {goal_x, goal_y, goal_phi};
                        boxItemCoordinates[detected] = coords;
                        RCLCPP_INFO(node->get_logger(), "Updated coordinates for %s: (%.2f, %.2f, %.2f)", detected.c_str(), coords[0], coords[1], coords[2]);
                        if (detected == objectName) { objectFound = true; }
                    }
                    else {
                        std::array<double, 3> coords = {goal_x, goal_y, goal_phi};
                        boxItemCoordinates[detected] = coords;
                        RCLCPP_INFO(node->get_logger(), "Detected object %s not in boxItemCoordinates list, but saving coordinates (%.2f, %.2f, %.2f) for future reference", detected.c_str(), coords[0], coords[1], coords[2]);
                    }

                    std::ofstream outfile("boxItemCoordinates.txt");
                    for (const auto& pair : boxItemCoordinates) {
                        outfile << pair.first << ": " << pair.second[0] << ", " <<pair.second[1] << ", " << pair.second[2] << std::endl;
                    }
                    outfile.close();
                }


            }
            else 
            {
                RCLCPP_INFO(node->get_logger(), "AprilTag not detected at goal pose, moving on to next box");
                objectFound = false;
                if (secondsElapsed - startAprilTagSearch > 20) {
                    RCLCPP_WARN(node->get_logger(), "AprilTag search timeout at current box, moving to next box");
                    arrivedAtGoal = false;
                }
            }
        }
        else if (objectFound) {
             putInBin();
             objectFound = false;
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