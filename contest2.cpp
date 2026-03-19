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
#include "rclcpp_action/rclcpp_action.hpp" // added - isabelle
#include "nav_msgs/msg/path.hpp" // added for path message type - isabelle
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // added for tf2::toMsg conversion

//new AprilTag globals
AprilTagDetector* tagDetector   = nullptr;
std::vector<int>  candidateTags = {0, 1, 2, 3, 4};

// global arm controller
ArmController* armController = nullptr;

rclcpp::Node::SharedPtr node;
std::unique_ptr<YoloInterface> yoloDetector;

std::string detectedClass = "";
float startupObjectPose[7] = {};

/*new forward declarations for the two AprilTag functions. 
The compiler needs to see these before main() calls them inside the while loop.*/
bool aprilTagDetected(float secondsElapsed); 
std::optional<geometry_msgs::msg::Pose> getBinTagPose();
//std::string yoloDetectionOutput(std::string cameraName, float secondsElapsed, bool saveImage = false);
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
    if (!tagDetector) return false; //new addition

    std::optional<geometry_msgs::msg::Pose> tagPose;


    if (secondsElapsed > lastPrintTime) {
        lastPrintTime = secondsElapsed;  //new add

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
                    return true; //new add: immediately confirm detection
                }
            }
        }
        else {
            RCLCPP_INFO(node->get_logger(), "No tags visible");
        }
    }
    return false;  //new add, if no valid tag pose is found, return false
}

/* Called inside putInBin() to get a live 3D reading of the
       bin tag at the moment the arm is about to move. Returns the
       Pose of the first visible tag in base_link frame, or
       std::nullopt if nothing found (putInBin uses a hardcoded
       fallback in that case). A live reading here rather than a
       saved value from Phase 3 keeps the arm target accurate even
       if the robot drifted slightly after navigating*/

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
    //bring the arm to a start position, orienting the wrist camera downwards towards the object of interest and adjust until image is detected
    RCLCPP_INFO(node->get_logger(), "orienting arm for pickup");
    orientForPickup();

    //call a function that moves the arm to the location of the object
    RCLCPP_INFO(node->get_logger(), "grabbing object");
    grab();

    return detectedClass;
}

bool isTargetObject(std::string name) //check if the given name is in the list of objects of interest
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
    float scanZ = 0.20; // height to hold camera while scanning

//    // float armPose[3][6] = {{startupObjectPose[0], startupObjectPose[1], scanZ, -0.006, -0.000, 1.658},
//                                 {startupObjectPose[0]- rotation, startupObjectPose[1] + rotation, scanZ,-0.006, -0.000, 1.658 },
//                                 {startupObjectPose[0] + rotation, startupObjectPose[1] - rotation, scanZ, -0.006, -0.000, 1.658  }};

    //move arm to starting scan pose — using confirmed reachable orientation as base
    armController->moveToCartesianPose(startupObjectPose[0], startupObjectPose[1], startupObjectPose[2]+0.244, -0.471, -0.557,  0.564, -0.387);
    armController->moveToCartesianPose(startupObjectPose[0], startupObjectPose[1], startupObjectPose[2]+0.170, -0.471, -0.557,  0.564, -0.387);
    armController->moveToCartesianPose(startupObjectPose[0], startupObjectPose[1], startupObjectPose[2]+0.100, -0.471, -0.557,  0.564, -0.387);
    //armController->moveToCartesianPose(armPose[0][0], armPose[0][1], armPose[0][2], armPose[0][3], armPose[0][4], armPose[0][5]);

    // for (int i=1; i<3; i++){    
    //     //object detection using the wrist camera and determine and save class
    //     detectedClass = yoloDetector->getObjectName(CameraSource::WRIST, true);
    //     float confidence = yoloDetector->getConfidence();

    //     if (!isTargetObject(detectedClass)&&confidence > 0.5){ //if we don't see anything and the confidence is too low
    //         armController->moveToCartesianPose(armPose[i][0], armPose[i][1], armPose[i][2], armPose[i][3], armPose[i][4], armPose[i][5]); 
    //     }
    //     else break;
    // }
}

void grab() {

    float scanZ     = 0.20; // same height used during scanning
    float approachZ = 0.14; // intermediate approach height

    //open gripper
    armController->openGripper();

    RCLCPP_INFO(node->get_logger(), "Moving arm to grab unknown object");

    //check what type of object it is and adjust the arm pose accordingly if needed

    // if (detectedClass == "waterbottle"){
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
    // armController->moveToCartesianPose(startupObjectPose[0], startupObjectPose[1], startupObjectPose[2], -0.006, -0.000, 1.658);    }

    // else if (detectedClass =="coffee_cup") {
    // //armController->moveToCartesianPose(startupObjectPose[0], startupObjectPose[1], startupObjectPose[2], startupObjectPose[3], startupObjectPose[4], startupObjectPose[5], startupObjectPose[6]);
    
    // }

    // Phase 1: start from near the confirmed working pose, same orientation throughout
    armController->moveToCartesianPose(0.043,                        0.199,                       0.313,                 -0.471, -0.557,  0.564, -0.387);

    // Phase 2: shift toward object XY while descending to scan height
    armController->moveToCartesianPose(startupObjectPose[0],         startupObjectPose[1] + 0.05, scanZ,                 -0.471, -0.557,  0.564, -0.387);

    // Phase 3: intermediate descent
    armController->moveToCartesianPose(startupObjectPose[0],         startupObjectPose[1] + 0.02, approachZ,             -0.471, -0.557,  0.564, -0.387);

    // Phase 4: final descent to cup
    armController->moveToCartesianPose(startupObjectPose[0],         startupObjectPose[1],        startupObjectPose[2],  -0.471, -0.557,  0.564, -0.387);

    armController->closeGripper();
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Mirror approach path back up
    armController->moveToCartesianPose(startupObjectPose[0],         startupObjectPose[1] + 0.02, approachZ,             -0.471, -0.557,  0.564, -0.387);

    //lift back up before moving to carry pose
    armController->moveToCartesianPose(startupObjectPose[0],         startupObjectPose[1],        0.280,                 -0.471, -0.557,  0.564, -0.387);

    //move the arm to location 2 to pick it up and orient to later drop it in
    RCLCPP_INFO(node->get_logger(), "Moving arm to position to later drop in bin");
    armController->moveToCartesianPose(0.043, 0.199, 0.313, -0.471, -0.557,  0.564, -0.387); //need to change this pose pending simulation testing

}

/*// void putInBin(){

//     //move the arm to location 3 - above the box
//     RCLCPP_INFO(node->get_logger(), "Moving arm to position above box");
//     armController->moveToCartesianPose(0.043, 0.199, 0.313, -0.471, -0.557, 0.564, -0.387); //need to change this pose pending simulation testing

//     //move the arm to location 4 - drop object into box
//     RCLCPP_INFO(node->get_logger(), "Moving arm to drop object into box");
//     armController->moveToCartesianPose(0.043, 0.199, 0.313, -0.471, -0.557, 0.564, -0.387); //need to change this pose pending simulation testing

//     //open gripper to drop object into box
//     RCLCPP_INFO(node->get_logger(), "Releasing object into box");
//     armController->openGripper();
//     std::this_thread::sleep_for(std::chrono::seconds(2));

//     //move the arm back up to location 3
//     RCLCPP_INFO(node->get_logger(), "Moving arm back up after dropping object into box");
//     armController->moveToCartesianPose(0.043, 0.199, 0.313, -0.471, -0.557, 0.564, -0.387); //need to change this pose pending simulation testing

//     shouldPutInBin = false;
// }*/

// putInBin() rewrittem
//          1. Calls getBinTagPose() for the live 3D bin tag position.
//          2. Adds HOVER_Z upward and BIN_X_OFFSET forward to compute
//             the arm target above the bin opening. These two constants
//             are the only values to tune to match your physical bin.
//          3. Moves to hover, checks reachability, lowers 5 cm, opens
//             the gripper, retracts, returns to neutral pose.
//          4. Falls back to hardcoded coords if tag is not visible.
//          5. Sets shouldPutInBin = false when done so the state
//             machine moves on to the next box.
void putInBin()
{
    RCLCPP_INFO(node->get_logger(), "putInBin: locating bin via AprilTag...");

    auto tagPose = getBinTagPose();

    if (!tagPose.has_value()) {
        RCLCPP_WARN(node->get_logger(),
            "putInBin: no AprilTag visible — dropping at fallback pose");
        armController->moveToCartesianPose(
            0.300, 0.000, 0.400, -0.471, -0.557, 0.564, -0.387); // may need to be changed experimentally
        armController->openGripper();
        std::this_thread::sleep_for(std::chrono::seconds(2));
        armController->moveToCartesianPose(
            0.043, 0.199, 0.313, -0.471, -0.557, 0.564, -0.387); //may need to be changed experimentally
        return;
    }

    constexpr double HOVER_Z      = 0.15; // metres above tag — tune to your bin rim height
    constexpr double BIN_X_OFFSET = 0.05; // metres forward into bin past the tag face

    double target_x = tagPose->position.x + BIN_X_OFFSET;
    double target_y = tagPose->position.y;
    double target_z = tagPose->position.z + HOVER_Z;

    // same gripper quaternion used throughout the file
    constexpr double ORI_X = -0.657;
    constexpr double ORI_Y = 0.000;
    constexpr double ORI_Z =  -0.000;
    constexpr double ORI_W = 0.754;

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
    // returns the optimal order of visiting the boxes (not including the robot which is node 0)

    double best_distance = std::numeric_limits<double>::max();
    std::vector<int> best_order;

    do
    {
        double total = 0.0;

        int current = 0; // start at robot

        for (int next : order)
        {
            total += distances[current][next];
            current = next;
        }

        // ADD RETURN TO ROBOT
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

    // Setup ROS 2
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>("contest2");

    yoloDetector = std::make_unique<YoloInterface>(node);
    
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

    //Initialize YOLO object detector
    //YoloInterface yoloInterface(node);
    
    //Initialize arm controller
    ArmController armController_address(node); 
    armController = &armController_address;

    //Initialize navigation interface
    Navigation navigator(node);
    
    AprilTagDetector aprilDetector(node);
    tagDetector = &aprilDetector;

    RCLCPP_INFO(node->get_logger(), "AprilTagDetector initialised (ref frame: %s, candidate tags: 0-4)",
            aprilDetector.getReferenceFrame().c_str());
        // Optional: uncomment to use camera frame instead of base_link
        // aprilDetector.setReferenceFrame("oakd_rgb_camera_optical_frame");

    // Contest countdown timer
    auto start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    RCLCPP_INFO(node->get_logger(), "Starting contest - 300 seconds timer begins now!");


    #pragma region PATH PLANNING TO BOXES - Isabelle
    // Define vector of vectors to hold paths to each box - Isabelle
    int num_boxes = boxes.coords.size();
    int num_nodes = num_boxes + 1; // robot + boxes
    std::vector<std::vector<double>> box_distances(num_nodes, std::vector<double>(num_nodes, 0.0)); // 2D vector: holds path to each box for each order - box_paths[start][goal]
    
    // Get list of all locations (robot + boxes) - Isabelle
    std::vector<std::array<double,3>> nodes;
    nodes.push_back({robotPose.x, robotPose.y, robotPose.phi}); // robot position is the first node
    for (auto &box : boxes.coords) { nodes.push_back({box[0], box[1], box[2]}); }

    // adjust coordinates so that robot goal position is 20cm in front of box and facing phi direction of box
    for (int i = 1; i < num_nodes; i++)
    {
        double buffer = 0.5; // 20 cm buffer in front of box
        nodes[i][0] -= buffer * cos(nodes[i][2] + M_PI);
        nodes[i][1] -= buffer * sin(nodes[i][2] + M_PI);
        nodes[i][2] = 2*M_PI - nodes[i][2]; // adjust robot orientation to face the box
    }
    
    // iterate though nodes and compute path from each node to each other node using ComputePathToPose action - Isabelle
    for (int start = 0; start < num_nodes; start++)
    {
        for (int goal = 0; goal < num_nodes; goal++)
        {
            if (start == goal)
                continue;
            // calculate straight path from start to goal and save it in box_paths[start][goal]
            double start_x = nodes[start][0];
            double start_y = nodes[start][1];

            double goal_x = nodes[goal][0];
            double goal_y = nodes[goal][1];

            double total_distance = sqrt(pow(goal_x - start_x, 2) + pow(goal_y - start_y, 2));

            box_distances[start][goal] = total_distance;;

            RCLCPP_INFO(node->get_logger(), "Path %d -> %d has %.2f distance", start, goal, box_distances[start][goal]);        
        }
    }

    // solve TSP to get optimal order of visiting boxes - Isabelle
    // create list of boxes without robot (first node) to pass to TSP solver 
    std::vector<int> order;
    for (int i = 1; i <= num_boxes; i++) { order.push_back(i); } 
    std::vector<int> optimal_order = solveTSP(box_distances, order);
    std::vector<int> full_route;
    full_route.push_back(0);  // start at robot

    for (int box : optimal_order)
    {
        full_route.push_back(box);
    }

    full_route.push_back(0);  // return to start

    // Print optimal order and total distance - Isabelle
    RCLCPP_INFO(node->get_logger(), "Best route:");

    int current = 0;
    double total_distance = 0.0;

    for (int box : optimal_order)
    {
        RCLCPP_INFO(node->get_logger(),
            " %d -> %d  (%.2f m)",
            current,
            box,
            box_distances[current][box]);

        total_distance += box_distances[current][box];
        current = box;
    }

    // return to robot
    RCLCPP_INFO(node->get_logger(),
        " %d -> %d  (%.2f m)  [RETURN]",
        current,
        0,
        box_distances[current][0]);

    total_distance += box_distances[current][0];

    RCLCPP_INFO(node->get_logger(), "Total route length: %.2f m", total_distance);

    // Define list of box locations for navigation () - Isabelle
    // std::vector<std::array<double,3>> box_locations;
    // for (auto &box : boxes.coords) {
    //     box_locations.push_back({box[0], box[1], box[2]});
    // }

    #pragma endregion END PATH PLANNING TO BOXES

    //initialize variable values
    bool startup = true; 
    bool armSuccess = false;
    bool gripSuccess = false;
    startupObjectPose[0] = 0.099;
    startupObjectPose[1] = -0.009;
    startupObjectPose[2] = 0.155;

    

    int currentBoxIndex = 0; // index to keep track of which box we are navigating to
    bool arrivedAtGoal = false; // flag to indicate if we have arrived at the current goal
    bool objectFound = false; // flag to indicate if the object we picked up is the one we are supposed to put in the box

    // Define starting coordinates
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

    // create array for saving location of each ITEM associated with each box
    std::vector<std::array<double,3>> item_locations;

    std::string detected = (yoloDetectionOutput("oakd", secondsElapsed, true));


    // Execute strategy
    while(rclcpp::ok() && secondsElapsed <= 300) {
        rclcpp::spin_some(node);

        // Calculate elapsed time
        auto now = std::chrono::system_clock::now();
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();

        /***YOUR CODE HERE***/
        // TEMP CODE /////////////////////////////////////////
        // if (aprilTagDetected(secondsElapsed)) {
        //     auto tagPose = getBinTagPose();

        //     if (tagPose.has_value())
        //     {
        //         RCLCPP_INFO(node->get_logger(),
        //             "Main loop: AprilTag detected at pos(%.3f, %.3f, %.3f)",
        //             tagPose->position.x, tagPose->position.y, tagPose->position.z);
        //         armController->moveToCartesianPose(
        //             tagPose->position.x, tagPose->position.y, tagPose->position.z + 0.1, // hover 10 cm above the tag
        //             -0.657, 0.000, -0.000, 0.754); // same gripper orientation used throughout the file
        //     }
        //      else {
        //         RCLCPP_INFO(node->get_logger(),
        //             "Main loop: AprilTag detected but no valid pose found");  
        //      }      
        // }
        // else {
        //     RCLCPP_INFO(node->get_logger(),
        //         "Main loop: No AprilTag detected");
        // }


        //Enter routine based on conditions
        //startup (pickup and detect our object
        
        if(startup) {
            objectName = startupArm();
            startup = false;
        }
        
        else if (currentBoxIndex < static_cast<int>(full_route.size()) - 1 && !arrivedAtGoal && !objectFound) 
        { // check if there are more boxes to navigate to and if we are not currently in the process of putting a box in the bin`
            //navigate to box
            int goal_node = full_route[currentBoxIndex + 1];

            // determine final coordinates to navigate to (robot or box)
            goal_x = nodes[goal_node][0];
            goal_y = nodes[goal_node][1];
            goal_phi = nodes[goal_node][2];

            if (goal_node == 0) {
                goal_x = start_x;
                goal_y = start_y;
                goal_phi = start_phi;
            } 

            RCLCPP_INFO(node->get_logger(), "Navigating to node %d at (%.2f, %.2f, %.2f)", goal_node, goal_x, goal_y, goal_phi);
            arrivedAtGoal = navigator.moveToGoal(goal_x, goal_y, goal_phi); // returns true when robot reaches goal, false if navigation failed
            RCLCPP_INFO(node->get_logger(), "Arrived at goal: %s", arrivedAtGoal ? "true" : "false");
            if (arrivedAtGoal) {
                currentBoxIndex++;
            }
        }
        // //added the apriltag bits, replaced the old code for state 3 and 4
        else if (arrivedAtGoal && !objectFound && currentBoxIndex < (int)full_route.size() - 1) 
        {  
            if (aprilTagDetected(secondsElapsed)) // checks if april tag is detected
            {
                auto tagPose = getBinTagPose();

                if (tagPose.has_value())
                {
                    double tag_x = tagPose->position.x;
                    double tag_y = tagPose->position.y;

                    double tag_yaw = getYawFromQuaternion(tagPose->orientation);

                    const double desired_distance = 0.5; // 50 cm

                    // position 50 cm in front of the tag
                    double target_x_robot = tag_x - desired_distance * cos(tag_yaw);
                    double target_y_robot = tag_y - desired_distance * sin(tag_yaw);

                    // robot should face the tag
                    double target_yaw_robot = atan2(tag_y, tag_x);

                    double goal_x =
                        robotPose.x +
                        cos(robotPose.phi) * target_x_robot -
                        sin(robotPose.phi) * target_y_robot;

                    double goal_y =
                        robotPose.y +
                        sin(robotPose.phi) * target_x_robot +
                        cos(robotPose.phi) * target_y_robot;

                    double goal_phi = robotPose.phi + target_yaw_robot;
                    goal_phi = atan2(sin(goal_phi), cos(goal_phi));

                    RCLCPP_INFO(node->get_logger(),
                        "Adjusting position near AprilTag: goal (%.2f, %.2f, %.2f)",
                        goal_x, goal_y, goal_phi);

                    navigator.moveToGoal(goal_x, goal_y, goal_phi);
                }

                RCLCPP_INFO(node->get_logger(),
                    "Bin AprilTag confirmed — preparing to drop object");

                // if tag is detected, use OAKD camera to take picture of item and determine if it is the correct item using Yolo
                
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
                }

                arrivedAtGoal  = false;
            }
            else 
            {
                // if couldn't detect tag, go to next box
                RCLCPP_INFO(node->get_logger(), "AprilTag not detected at goal pose, moving on to next box");
                objectFound = false;
                arrivedAtGoal = false;
            }
        }
        else if (objectFound) {
             putInBin();
             objectFound = false;
        }
             
        
        

        // else {continue;}

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (secondsElapsed > 300) {
        RCLCPP_WARN(node->get_logger(), "Contest time limit reached!");
    }

    RCLCPP_INFO(node->get_logger(), "Contest 2 node shutting down");
    rclcpp::shutdown();
    return 0;



}