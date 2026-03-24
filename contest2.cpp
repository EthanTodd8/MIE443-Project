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

#include <optional>      /* std::optional lets getBinTagPose() return either a real Pose
                          or "nothing found" without using raw pointers */

#include "nav2_msgs/action/compute_path_to_pose.hpp" // added - isabelle
#include "rclcpp_action/rclcpp_action.hpp"           // added - isabelle
#include "nav_msgs/msg/path.hpp"                     // added for path message type - isabelle
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"   // added for tf2::toMsg conversion
#include "sensor_msgs/msg/joint_state.hpp"           // added for joint command publishing

// ─── AprilTag globals ────────────────────────────────────────────────────────
AprilTagDetector* tagDetector   = nullptr;
std::vector<int>  candidateTags = {0, 1, 2, 3, 4};

// ─── Arm controller global ───────────────────────────────────────────────────
ArmController* armController = nullptr;

// ─── Joint command publisher global ─────────────────────────────────────────
rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointCommandPub;

rclcpp::Node::SharedPtr node;
std::unique_ptr<YoloInterface> yoloDetector;

std::string detectedClass = "";
float startupObjectPose[7] = {};

// ─── Forward declarations ────────────────────────────────────────────────────
bool aprilTagDetected(float secondsElapsed);
std::optional<geometry_msgs::msg::Pose> getBinTagPose();
void orientForPickup();
void grab();
void putInBin();
bool isTargetObject(std::string name);
std::string startupArm();

// ─── Helper: publish a set of joint positions to /joint_commands ─────────────
// Equivalent to: ros2 topic pub --once /joint_commands sensor_msgs/msg/JointState
//   "{name: ['1','2','3','4','5','6'], position: [...]}"
void moveToJointPositions(const std::vector<double>& positions)
{
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = node->now();
    msg.name         = {"1", "2", "3", "4", "5", "6"};
    msg.position     = positions;
    jointCommandPub->publish(msg);

    // Allow the arm time to reach the target pose before the next command
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
}

// ─── Utility ─────────────────────────────────────────────────────────────────
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

        if (!detected.empty()) {
            float confidence = yoloDetector->getConfidence();
            RCLCPP_INFO(node->get_logger(), "Detected: %s (Confidence: %.2f)",
                    detected.c_str(), confidence);
        } else {
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
                        tagPose->orientation.x, tagPose->orientation.y,
                        tagPose->orientation.z, tagPose->orientation.w);
                    return true;
                }
            }
        } else {
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

    RCLCPP_INFO(node->get_logger(), "grabbing object");
    grab();

    return detectedClass;
}

bool isTargetObject(std::string name)
{
    return (name == "bottle"     ||
            name == "cup"        ||
            name == "clock"      ||
            name == "plant"      ||
            name == "motorcycle");
}

void orientForPickup()
{
    // ── Phase 1: move to pre-defined joint-space poses so the wrist camera
    //    is positioned above the object area before the Cartesian scan begins.
    //    These match the three CLI commands from the command line:
    //      pose 1 — arm extended, camera looking down at object
    //      pose 2 — slight retraction / tilt adjustment
    //      pose 3 — approach angle for pickup
    RCLCPP_INFO(node->get_logger(), "orientForPickup: joint pose 1 — camera over object");
    moveToJointPositions({-1.5933, -0.8909,   0.8583,  1.600,  2.7448, -0.0774});

    RCLCPP_INFO(node->get_logger(), "orientForPickup: joint pose 2 — tilt adjustment");
    moveToJointPositions({-1.5933, -0.58519, -0.23257, 1.6061, 2.7448, -0.0774});

    RCLCPP_INFO(node->get_logger(), "orientForPickup: joint pose 3 — approach angle");
    moveToJointPositions({-0.4208, -0.5222,  -0.5366,  0.8286, 2.7448, -0.0774});

    // ── Phase 2: existing Cartesian scan logic ────────────────────────────────
    float scanZ = 0.20; // height to hold camera while scanning

    float armPose[3][6] = {
        {startupObjectPose[0],            startupObjectPose[1],            scanZ, -0.006, -0.000, 1.658},
        {startupObjectPose[0] - rotation, startupObjectPose[1] + rotation, scanZ, -0.006, -0.000, 1.658},
        {startupObjectPose[0] + rotation, startupObjectPose[1] - rotation, scanZ, -0.006, -0.000, 1.658}
    };

    // Move arm to starting scan pose using confirmed reachable orientation as base
    armController->moveToCartesianPose(startupObjectPose[0], startupObjectPose[1], startupObjectPose[2] + 0.244, -0.471, -0.557,  0.564, -0.387);
    armController->moveToCartesianPose(startupObjectPose[0], startupObjectPose[1], startupObjectPose[2] + 0.170, -0.471, -0.557,  0.564, -0.387);
    armController->moveToCartesianPose(startupObjectPose[0], startupObjectPose[1], startupObjectPose[2] + 0.100, -0.471, -0.557,  0.564, -0.387);
    armController->moveToCartesianPose(armPose[0][0], armPose[0][1], armPose[0][2], armPose[0][3], armPose[0][4], armPose[0][5]);

    for (int i = 1; i < 3; i++) {
        // Object detection using the wrist camera — determine and save class
        detectedClass = yoloDetector->getObjectName(CameraSource::WRIST, true);
        float confidence = yoloDetector->getConfidence();

        if (!isTargetObject(detectedClass) && confidence > 0.5) {
            // Nothing useful seen yet — try the next scan position
            armController->moveToCartesianPose(
                armPose[i][0], armPose[i][1], armPose[i][2],
                armPose[i][3], armPose[i][4], armPose[i][5]);
        } else {
            break;
        }
    }
}

void grab()
{
    float scanZ     = 0.20; // same height used during scanning
    float approachZ = 0.14; // intermediate approach height

    // Open gripper
    armController->openGripper();

    RCLCPP_INFO(node->get_logger(), "Moving arm to grab unknown object");

    // Check object type and adjust arm pose accordingly if needed
    if (detectedClass == "waterbottle") {
        armController->moveToCartesianPose(startupObjectPose[0], startupObjectPose[1], scanZ,                  -0.006, -0.000, 1.658);
        armController->moveToCartesianPose(startupObjectPose[0], startupObjectPose[1], startupObjectPose[2],   -0.006, -0.000, 1.658);
    } else if (detectedClass == "plant") {
        armController->moveToCartesianPose(startupObjectPose[0], startupObjectPose[1], scanZ,                  -0.006, -0.000, 1.658);
        armController->moveToCartesianPose(startupObjectPose[0], startupObjectPose[1], startupObjectPose[2],   -0.006, -0.000, 1.658);
    } else if (detectedClass == "motorcycle") {
        armController->moveToCartesianPose(startupObjectPose[0], startupObjectPose[1], scanZ,                  -0.006, -0.000, 1.658);
        armController->moveToCartesianPose(startupObjectPose[0], startupObjectPose[1], startupObjectPose[2],   -0.006, -0.000, 1.658);
    } else if (detectedClass == "clock") {
        armController->moveToCartesianPose(startupObjectPose[0], startupObjectPose[1], scanZ,                  -0.006, -0.000, 1.658);
        armController->moveToCartesianPose(startupObjectPose[0], startupObjectPose[1], startupObjectPose[2],   -0.006, -0.000, 1.658);
    } else if (detectedClass == "coffee_cup") {
        // pose TBD pending simulation testing
    }

    // Move arm to carry position ready for bin drop
    RCLCPP_INFO(node->get_logger(), "Moving arm to position to later drop in bin");
    armController->moveToCartesianPose(0.043, 0.199, 0.313, -0.471, -0.557, 0.564, -0.387);
}

void putInBin()
{
    RCLCPP_INFO(node->get_logger(), "putInBin: locating bin via AprilTag...");

    auto tagPose = getBinTagPose();

    if (!tagPose.has_value()) {
        RCLCPP_WARN(node->get_logger(),
            "putInBin: no AprilTag visible — dropping at fallback pose");
        armController->moveToCartesianPose(
            0.300, 0.000, 0.400, -0.471, -0.557, 0.564, -0.387);
        armController->openGripper();
        std::this_thread::sleep_for(std::chrono::seconds(2));
        armController->moveToCartesianPose(
            0.043, 0.199, 0.313, -0.471, -0.557, 0.564, -0.387);
        return;
    }

    constexpr double HOVER_Z      = 0.15; // metres above tag — tune to your bin rim height
    constexpr double BIN_X_OFFSET = 0.05; // metres forward into bin past the tag face

    double target_x = tagPose->position.x + BIN_X_OFFSET;
    double target_y = tagPose->position.y;
    double target_z = tagPose->position.z + HOVER_Z;

    constexpr double ORI_X = -0.657;
    constexpr double ORI_Y =  0.000;
    constexpr double ORI_Z = -0.000;
    constexpr double ORI_W =  0.754;

    RCLCPP_INFO(node->get_logger(),
        "putInBin: moving above bin at (%.3f, %.3f, %.3f)",
        target_x, target_y, target_z);

    bool success = armController->moveToCartesianPose(
        target_x, target_y, target_z, ORI_X, ORI_Y, ORI_Z, ORI_W);

    if (!success) {
        RCLCPP_ERROR(node->get_logger(),
            "putInBin: hover pose unreachable — aborting drop");
        return;
    }

    double drop_z = target_z - 0.05;
    armController->moveToCartesianPose(
        target_x, target_y, drop_z, ORI_X, ORI_Y, ORI_Z, ORI_W);

    RCLCPP_INFO(node->get_logger(), "putInBin: releasing object");
    armController->openGripper();
    std::this_thread::sleep_for(std::chrono::seconds(2));

    RCLCPP_INFO(node->get_logger(), "putInBin: retracting arm");
    armController->moveToCartesianPose(
        target_x, target_y, target_z, ORI_X, ORI_Y, ORI_Z, ORI_W);

    armController->moveToCartesianPose(
        0.043, 0.199, 0.313, -0.471, -0.557, 0.564, -0.387);

    RCLCPP_INFO(node->get_logger(), "putInBin: complete");
}

std::vector<int> solveTSP(const std::vector<std::vector<double>> &distances, std::vector<int> order)
{
    double best_distance = std::numeric_limits<double>::max();
    std::vector<int> best_order;

    do {
        double total  = 0.0;
        int    current = 0; // start at robot

        for (int next : order) {
            total += distances[current][next];
            current = next;
        }
        total += distances[current][0]; // return to robot

        if (total < best_distance) {
            best_distance = total;
            best_order    = order;
        }
    } while (std::next_permutation(order.begin(), order.end()));

    return best_order;
}

int main(int argc, char** argv)
{
    // ── ROS 2 setup ───────────────────────────────────────────────────────────
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>("contest2");

    yoloDetector = std::make_unique<YoloInterface>(node);

    // ── Joint command publisher ───────────────────────────────────────────────
    jointCommandPub = node->create_publisher<sensor_msgs::msg::JointState>("/joint_commands", 10);

    // ── Load arm URDF / SRDF ──────────────────────────────────────────────────
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

    // ── Robot pose subscriber ─────────────────────────────────────────────────
    RobotPose robotPose(0, 0, 0);
    auto amclSub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose",
        10,
        std::bind(&RobotPose::poseCallback, &robotPose, std::placeholders::_1)
    );

    // ── Box coordinates ───────────────────────────────────────────────────────
    Boxes boxes;
    if (!boxes.load_coords()) {
        RCLCPP_ERROR(node->get_logger(), "ERROR: could not load box coordinates");
        return -1;
    }

    for (size_t i = 0; i < boxes.coords.size(); ++i) {
        RCLCPP_INFO(node->get_logger(), "Box %zu coordinates: x=%.2f, y=%.2f, phi=%.2f",
                    i, boxes.coords[i][0], boxes.coords[i][1], boxes.coords[i][2]);
    }

    // ── Subsystem initialisation ──────────────────────────────────────────────
    ArmController armController_address(node);
    armController = &armController_address;

    Navigation navigator(node);

    AprilTagDetector aprilDetector(node);
    tagDetector = &aprilDetector;

    RCLCPP_INFO(node->get_logger(),
        "AprilTagDetector initialised (ref frame: %s, candidate tags: 0-4)",
        aprilDetector.getReferenceFrame().c_str());

    // ── Contest timer ─────────────────────────────────────────────────────────
    auto     start          = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;
    RCLCPP_INFO(node->get_logger(), "Starting contest - 300 seconds timer begins now!");

    // ─────────────────────────────────────────────────────────────────────────
    #pragma region PATH PLANNING TO BOXES - Isabelle

    int num_boxes = boxes.coords.size();
    int num_nodes = num_boxes + 1;
    std::vector<std::vector<double>> box_distances(num_nodes, std::vector<double>(num_nodes, 0.0));

    std::vector<std::array<double, 3>> nodes;
    nodes.push_back({robotPose.x, robotPose.y, robotPose.phi});
    for (auto &box : boxes.coords) { nodes.push_back({box[0], box[1], box[2]}); }

    // Adjust coordinates so the robot goal is 50 cm in front of each box
    for (int i = 1; i < num_nodes; i++) {
        double buffer = 0.5;
        nodes[i][0] -= buffer * cos(nodes[i][2] + M_PI);
        nodes[i][1] -= buffer * sin(nodes[i][2] + M_PI);
        nodes[i][2]  = 2 * M_PI - nodes[i][2];
    }

    // Compute straight-line distances between all node pairs
    for (int s = 0; s < num_nodes; s++) {
        for (int g = 0; g < num_nodes; g++) {
            if (s == g) continue;
            double dx = nodes[g][0] - nodes[s][0];
            double dy = nodes[g][1] - nodes[s][1];
            box_distances[s][g] = sqrt(dx * dx + dy * dy);
            RCLCPP_INFO(node->get_logger(), "Path %d -> %d has %.2f distance", s, g, box_distances[s][g]);
        }
    }

    // Solve TSP
    std::vector<int> order;
    for (int i = 1; i <= num_boxes; i++) { order.push_back(i); }
    std::vector<int> optimal_order = solveTSP(box_distances, order);

    std::vector<int> full_route;
    full_route.push_back(0);
    for (int box : optimal_order) { full_route.push_back(box); }
    full_route.push_back(0);

    // Log optimal route
    RCLCPP_INFO(node->get_logger(), "Best route:");
    int    current_node    = 0;
    double total_dist      = 0.0;
    for (int box : optimal_order) {
        RCLCPP_INFO(node->get_logger(), " %d -> %d  (%.2f m)", current_node, box, box_distances[current_node][box]);
        total_dist  += box_distances[current_node][box];
        current_node = box;
    }
    RCLCPP_INFO(node->get_logger(), " %d -> %d  (%.2f m)  [RETURN]", current_node, 0, box_distances[current_node][0]);
    total_dist += box_distances[current_node][0];
    RCLCPP_INFO(node->get_logger(), "Total route length: %.2f m", total_dist);

    #pragma endregion END PATH PLANNING TO BOXES

    // ─────────────────────────────────────────────────────────────────────────
    // State variables
    bool startup        = true;
    bool armSuccess     = false;
    bool gripSuccess    = false;

    startupObjectPose[0] = 0.099;
    startupObjectPose[1] = -0.009;
    startupObjectPose[2] = 0.155;

    int  currentBoxIndex = 0;
    bool arrivedAtGoal   = false;
    bool objectFound     = false;

    double start_x   = robotPose.x;
    double start_y   = robotPose.y;
    double start_phi = robotPose.phi;

    std::string objectName = "";
    std::map<std::string, std::array<double, 3>> boxItemCoordinates;
    boxItemCoordinates["bottle"]     = {0.0, 0.0, 0.0};
    boxItemCoordinates["plant"]      = {0.0, 0.0, 0.0};
    boxItemCoordinates["motorcycle"] = {0.0, 0.0, 0.0};
    boxItemCoordinates["clock"]      = {0.0, 0.0, 0.0};

    double goal_x, goal_y, goal_phi;

    std::vector<std::array<double, 3>> item_locations;

    std::string detected = yoloDetectionOutput("oakd", secondsElapsed, true);

    // ── Main loop ─────────────────────────────────────────────────────────────
    while (rclcpp::ok() && secondsElapsed <= 300) {
        rclcpp::spin_some(node);

        auto now       = std::chrono::system_clock::now();
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();

        // ── State: startup — pick up object and identify it ───────────────────
        if (startup) {
            objectName = startupArm();
            startup    = false;
        }
        // ── State: navigate to next box ───────────────────────────────────────
        else if (currentBoxIndex < static_cast<int>(full_route.size()) - 1
                 && !arrivedAtGoal
                 && !objectFound)
        {
            int goal_node = full_route[currentBoxIndex + 1];

            goal_x   = nodes[goal_node][0];
            goal_y   = nodes[goal_node][1];
            goal_phi = nodes[goal_node][2];

            if (goal_node == 0) {
                goal_x   = start_x;
                goal_y   = start_y;
                goal_phi = start_phi;
            }

            RCLCPP_INFO(node->get_logger(),
                "Navigating to node %d at (%.2f, %.2f, %.2f)", goal_node, goal_x, goal_y, goal_phi);

            arrivedAtGoal = navigator.moveToGoal(goal_x, goal_y, goal_phi);
            RCLCPP_INFO(node->get_logger(), "Arrived at goal: %s", arrivedAtGoal ? "true" : "false");

            if (arrivedAtGoal) { currentBoxIndex++; }
        }
        // ── State: arrived — check AprilTag and YOLO ──────────────────────────
        else if (arrivedAtGoal && !objectFound
                 && currentBoxIndex < static_cast<int>(full_route.size()) - 1)
        {
            if (aprilTagDetected(secondsElapsed)) {
                auto tagPose = getBinTagPose();

                if (tagPose.has_value()) {
                    double tag_x   = tagPose->position.x;
                    double tag_y   = tagPose->position.y;
                    double tag_yaw = getYawFromQuaternion(tagPose->orientation);

                    const double desired_distance = 0.5;

                    double target_x_robot = tag_x - desired_distance * cos(tag_yaw);
                    double target_y_robot = tag_y - desired_distance * sin(tag_yaw);
                    double target_yaw_robot = atan2(tag_y, tag_x);

                    goal_x   = robotPose.x + cos(robotPose.phi) * target_x_robot - sin(robotPose.phi) * target_y_robot;
                    goal_y   = robotPose.y + sin(robotPose.phi) * target_x_robot + cos(robotPose.phi) * target_y_robot;
                    goal_phi = atan2(sin(robotPose.phi + target_yaw_robot), cos(robotPose.phi + target_yaw_robot));

                    RCLCPP_INFO(node->get_logger(),
                        "Adjusting position near AprilTag: goal (%.2f, %.2f, %.2f)",
                        goal_x, goal_y, goal_phi);

                    navigator.moveToGoal(goal_x, goal_y, goal_phi);
                }

                RCLCPP_INFO(node->get_logger(), "Bin AprilTag confirmed — preparing to drop object");

                std::string detected = yoloDetectionOutput("oakd", secondsElapsed, true);
                if (!detected.empty()) {
                    std::array<double, 3> coords = {goal_x, goal_y, goal_phi};
                    boxItemCoordinates[detected]  = coords;

                    if (boxItemCoordinates.find(detected) != boxItemCoordinates.end()) {
                        RCLCPP_INFO(node->get_logger(),
                            "Updated coordinates for %s: (%.2f, %.2f, %.2f)",
                            detected.c_str(), coords[0], coords[1], coords[2]);
                    } else {
                        RCLCPP_INFO(node->get_logger(),
                            "Detected object %s not in boxItemCoordinates list, saving anyway (%.2f, %.2f, %.2f)",
                            detected.c_str(), coords[0], coords[1], coords[2]);
                    }

                    if (detected == objectName) { objectFound = true; }
                }

                arrivedAtGoal = false;
            } else {
                RCLCPP_INFO(node->get_logger(),
                    "AprilTag not detected at goal pose, moving on to next box");
                objectFound   = false;
                arrivedAtGoal = false;
            }
        }
        // ── State: correct bin found — drop object ────────────────────────────
        else if (objectFound) {
            putInBin();
            objectFound = false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (secondsElapsed > 300) {
        RCLCPP_WARN(node->get_logger(), "Contest time limit reached!");
    }

    RCLCPP_INFO(node->get_logger(), "Contest 2 node shutting down");
    rclcpp::shutdown();
    return 0;
}