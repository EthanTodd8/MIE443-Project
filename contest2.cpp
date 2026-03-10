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

#include <optional>      /* std::optional lets getBinTagPose() return either a real Pose
                          or "nothing found" without using raw pointers */ 
                          
#include "nav2_msgs/action/compute_path_to_pose.hpp" // added - isabelle
#include "rclcpp_action/rclcpp_action.hpp" // added - isabelle
#include "nav_msgs/msg/path.hpp" // added for path message type - isabelle

rclcpp::Node::SharedPtr node;  // global - to make it accessible everywhere
//new AprilTag globals
AprilTagDetector* tagDetector   = nullptr;
std::vector<int>  candidateTags = {0, 1, 2, 3, 4};

/*new forward declarations for the two AprilTag functions. 
The compiler needs to see these before main() calls them inside the while loop.*/
bool aprilTagDetected();
std::optional<geometry_msgs::msg::Pose> getBinTagPose();

/*aprilTagDetected() -  Called in Phase 3 of the while loop after the robot arrives
       at a box location. Checks if any of the 5 bin tags are
       visible and have a solvable 3D pose. Returns true the moment
       one is confirmed, advancing the state machine to Phase 4.
       Returns false so the loop keeps waiting if nothing is visible*/
bool aprilTagDetected()
        {
            if (!tagDetector) return false;
            auto visible = tagDetector->getVisibleTags(candidateTags);
            if (visible.empty()) {
                RCLCPP_INFO(node->get_logger(), "AprilTag: no tags visible");
                return false;
            }
            for (int tag_id : visible) {
                auto pose = tagDetector->getTagPose(tag_id);
                if (pose.has_value()) {
                    RCLCPP_INFO(node->get_logger(),
                        "AprilTag detected: tag%d  pos(%.3f, %.3f, %.3f)"
                        "  ori(%.3f, %.3f, %.3f, %.3f)",
                        tag_id,
                        pose->position.x, pose->position.y, pose->position.z,
                        pose->orientation.x, pose->orientation.y,
                        pose->orientation.z, pose->orientation.w);
                    return true;
                }
            }
            return false;
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
                        tag_id,
                        pose->position.x, pose->position.y, pose->position.z);
                    return pose;
                }
            }
            return std::nullopt;
        }

void startupArm()
{
    //bring the arm to a start position, orienting the wrist camera downwards towards the object of interest
    RCLCPP_INFO(note->get_logger(), "orienting arm for pickup");
    orientForPickup();

    //call a function that captures an image from the wrist camera/adjust until object is detected and arm is above
    RCLCPP_INFO(note->get_logger(), "detecting unknown object");
    //add that in here

    //call a function that moves the arm to the location of the object
    RCLCPP_INFO(note->get_logger(), "grabbing object");
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
    RCLCPP_INFO(note->get_logger(), "Grabbing the unknown object");
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

/*putInBin() rewrittem
         1. Calls getBinTagPose() for the live 3D bin tag position.
         2. Adds HOVER_Z upward and BIN_X_OFFSET forward to compute
            the arm target above the bin opening. These two constants
            are the only values to tune to match your physical bin.
         3. Moves to hover, checks reachability, lowers 5 cm, opens
            the gripper, retracts, returns to neutral pose.
         4. Falls back to hardcoded coords if tag is not visible.
         5. Sets shouldPutInBin = false when done so the state
            machine moves on to the next box.
/* void putInBin()
                {
            RCLCPP_INFO(node->get_logger(), "putInBin: locating bin via AprilTag...");

            auto tagPose = getBinTagPose();

            if (!tagPose.has_value()) {
                RCLCPP_WARN(node->get_logger(),
                    "putInBin: no AprilTag visible — dropping at fallback pose");
                armController.moveToCartesianPose(
                    0.300, 0.000, 0.400, -0.471, -0.557, 0.564, -0.387);
                armController.openGripper();
                std::this_thread::sleep_for(std::chrono::seconds(2));
                armController.moveToCartesianPose(
                    0.043, 0.199, 0.313, -0.471, -0.557, 0.564, -0.387);
                shouldPutInBin = false;
                return;
            }

            constexpr double HOVER_Z      = 0.15; // metres above tag — tune to your bin rim height
            constexpr double BIN_X_OFFSET = 0.05; // metres forward into bin past the tag face

            double target_x = tagPose->position.x + BIN_X_OFFSET;
            double target_y = tagPose->position.y;
            double target_z = tagPose->position.z + HOVER_Z;

            // same gripper quaternion used throughout the file
            constexpr double ORI_X = -0.471;
            constexpr double ORI_Y = -0.557;
            constexpr double ORI_Z =  0.564;
            constexpr double ORI_W = -0.387;

            RCLCPP_INFO(node->get_logger(),
                "putInBin: moving above bin at (%.3f, %.3f, %.3f)",
                target_x, target_y, target_z);

            bool success = armController.moveToCartesianPose(
                target_x, target_y, target_z, ORI_X, ORI_Y, ORI_Z, ORI_W);

            if (!success) {
                RCLCPP_ERROR(node->get_logger(),
                    "putInBin: hover pose unreachable — aborting drop");
                shouldPutInBin = false;
                return;
            }

            double drop_z = target_z - 0.05;
            armController.moveToCartesianPose(
                target_x, target_y, drop_z, ORI_X, ORI_Y, ORI_Z, ORI_W);

            RCLCPP_INFO(node->get_logger(), "putInBin: releasing object");
            armController.openGripper();
            std::this_thread::sleep_for(std::chrono::seconds(2));

            RCLCPP_INFO(node->get_logger(), "putInBin: retracting arm");
            armController.moveToCartesianPose(
                target_x, target_y, target_z, ORI_X, ORI_Y, ORI_Z, ORI_W);

            armController.moveToCartesianPose(
                0.043, 0.199, 0.313, -0.471, -0.557, 0.564, -0.387);

            RCLCPP_INFO(node->get_logger(), "putInBin: complete");
            shouldPutInBin = false;
        }*/

double pathLength(const nav_msgs::msg::Path &path)
{
    // this just calculate the distance between the points - doesn't take into account path(?) - isabelle
    double length = 0.0;

    for (size_t i = 1; i < path.poses.size(); i++)
    {
        double dx = path.poses[i].pose.position.x -
                    path.poses[i-1].pose.position.x;

        double dy = path.poses[i].pose.position.y -
                    path.poses[i-1].pose.position.y;

        length += sqrt(dx*dx + dy*dy);
    }

    return length;
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
    auto node = std::make_shared<rclcpp::Node>("contest2");

    // Add path to client here so that we can use nav2 to compute paths to boxes - isabelle
    using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
    auto path_client = rclcpp_action::create_client<ComputePathToPose>(node, "compute_path_to_pose");

    if (!path_client->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(node->get_logger(), "ComputePathToPose server not available");
    } else {
        RCLCPP_INFO(node->get_logger(), "Connected to ComputePathToPose server");
    }

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
    std::vector<std::vector<nav_msgs::msg::Path>> box_paths(num_nodes, std::vector<nav_msgs::msg::Path>(num_nodes)); // 2D vector: holds path to each box for each order - box_paths[start][goal]
    
    // Get list of all locations (robot + boxes) - Isabelle
    std::vector<std::array<double,3>> nodes;
    nodes.push_back({robotPose.x, robotPose.y, robotPose.phi}); // robot position is the first node
    for (auto &box : boxes.coords) { nodes.push_back({box[0], box[1], box[2]}); }

    // Add buffer to coordinates so that robot doesn't try to drive directly to the center of the box and instead goes to a point slightly in front of it - Isabelle
    for (int i = 1; i < num_nodes; i++)
    {
        double buffer = 0.2; // 20 cm buffer in front of box
        double angle = nodes[i][2]; // orientation of box
        nodes[i][0] -= buffer * cos(angle); // adjust x coordinate
        nodes[i][1] -= buffer * sin(angle); // adjust y coordinate
    }
    
    // iterate though nodes and compute path from each node to each other node using ComputePathToPose action - Isabelle
    for (int start = 0; start < num_nodes; start++)
    {
        for (int goal = 0; goal < num_nodes; goal++)
        {
            if (start == goal)
                continue;

            auto goal_msg = ComputePathToPose::Goal();

            goal_msg.goal.header.frame_id = "map";
            goal_msg.goal.header.stamp = node->now();

            goal_msg.start.header.frame_id = "map";
            goal_msg.start.header.stamp = node->now();

            goal_msg.start.pose.position.x = nodes[start][0];
            goal_msg.start.pose.position.y = nodes[start][1];

            goal_msg.goal.pose.position.x = nodes[goal][0];
            goal_msg.goal.pose.position.y = nodes[goal][1];

            tf2::Quaternion q;
            q.setRPY(0,0,nodes[goal][2]);
            goal_msg.goal.pose.orientation = tf2::toMsg(q);

            auto goal_future = path_client->async_send_goal(goal_msg);

            rclcpp::spin_until_future_complete(node, goal_future);

            auto goal_handle = goal_future.get();

            auto result_future = path_client->async_get_result(goal_handle);

            rclcpp::spin_until_future_complete(node, result_future);

            auto result = result_future.get();

            box_paths[start][goal] = result.result->path;

            RCLCPP_INFO(node->get_logger(), "Path %d -> %d has %ld poses", start, goal, box_paths[start][goal].poses.size());
        }
    }

    // build distance matrix from path lengths for TSP solver - Isabelle
    std::vector<std::vector<double>> distances(num_nodes,std::vector<double>(num_nodes, 0.0)); // 2D vector to hold distances between nodes - distances[start][goal]
    for (int i = 0; i < num_nodes; i++)
    {
        for (int j = 0; j < num_nodes; j++)
        {
            if (i == j) continue;
            distances[i][j] = pathLength(box_paths[i][j]);
        }
    }

    // solve TSP to get optimal order of visiting boxes - Isabelle
    // create list of boxes without robot (first node) to pass to TSP solver 
    std::vector<int> order;
    for (int i = 1; i <= num_boxes; i++) { order.push_back(i); } 
    std::vector<int> optimal_order = solveTSP(distances, order);
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
            distances[current][box]);

        total_distance += distances[current][box];
        current = box;
    }

    // return to robot
    RCLCPP_INFO(node->get_logger(),
        " %d -> %d  (%.2f m)  [RETURN]",
        current,
        0,
        distances[current][0]);

    total_distance += distances[current][0];

    RCLCPP_INFO(node->get_logger(), "Total route length: %.2f m", total_distance);

    // Define list of box locations for navigation () - Isabelle
    std::vector<std::array<double,3>> box_locations;
    for (auto &box : boxes.coords) {
        box_locations.push_back({box[0], box[1], box[2]});
    }

    #pragma endregion END PATH PLANNING TO BOXES

    //initialize variable values
    bool startup = true; 
    bool armSuccess = false;
    char detectedClass;
    float startupArmPose[7];
    bool gripSuccess = false;
    ///initialize the arm pose for locating and grabbing the object as a 'neutral' pose
    for (int i = 0; i < startupArmPose.size(); i++) { // since startupArmPose is a C-style array, we can't use range-based for loop, so we have to use 7
        startupArmPose[i] = 0.0;
    }


    int currentBoxIndex = 0; // index to keep track of which box we are navigating to
    bool arrivedAtGoal = false; // flag to indicate if we have arrived at the current goal
    bool itemDetected = false; // flag to indicate if the item is detected in the wrist camera

    // Define starting coordinates
    double start_x = robotPose.x;
    double start_y = robotPose.y;
    double start_phi = robotPose.phi;

    // Execute strategy
    while(rclcpp::ok() && secondsElapsed <= 300) {
        rclcpp::spin_some(node);

        // Calculate elapsed time
        auto now = std::chrono::system_clock::now();
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();

        /***YOUR CODE HERE***/

        //Enter routine based on conditions
        //startup (pickup and detect our object
        if(startup) {
            startupArm();
        }
        else if (currentBoxIndex < full_route.size() - 1 && !arrivedAtGoal && !itemDetected) { // check if there are more boxes to navigate to and if we are not currently in the process of putting a box in the bin`
            //navigate to box
            int goal_node = full_route[currentBoxIndex + 1];

            // determine final coordinates to navigate to (robot or box)
            double goal_x, goal_y, goal_phi;
            if (goal_node == 0) {
                goal_x = start_x;
                goal_y = start_y;
                goal_phi = start_phi;
            } else {
                goal_x = box_locations[goal_node - 1][0];
                goal_y = box_locations[goal_node - 1][1];
                goal_phi = box_locations[goal_node - 1][2];
            }

            RCLCPP_INFO(node->get_logger(), "Navigating to node %d at (%.2f, %.2f, %.2f)", goal_node, goal_x, goal_y, goal_phi);
            arrivedAtGoal = moveToGoal(goal_x, goal_y, goal_phi); // returns true when robot reaches goal, false if navigation failed
            currentBoxIndex++;
        }
        //added the apriltag bits, replaced the old code for state 3 and 4
           else if (arrivedAtGoal && !itemDetected && currentBoxIndex < (int)full_route.size() - 1) {
              if (aprilTagDetected()) {
                  RCLCPP_INFO(node->get_logger(),
                      "Bin AprilTag confirmed — preparing to drop object");
                  itemDetected   = true;
                  shouldPutInBin = true;
                  arrivedAtGoal  = false;
              }
          }
          else if (itemDetected && shouldPutInBin) {
            putInBin();
            itemDetected = false;
        }

        else {continue;}

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (secondsElapsed > 300) {
        RCLCPP_WARN(node->get_logger(), "Contest time limit reached!");
    }

    RCLCPP_INFO(node->get_logger(), "Contest 2 node shutting down");
    rclcpp::shutdown();
    return 0;



}
