#include <iostream>
#include <string>
#include <queue>
#include <vector>
#include <map>
#include <set>
#include <algorithm>

#include <unordered_map>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

#include "osrf_gear/Order.h"
#include "osrf_gear/Shipment.h"
#include "osrf_gear/Product.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "osrf_gear/GetMaterialLocations.h"

#include "ros/ros.h"

#include "ik_service/PoseIK.h"

#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/TransformStamped.h"

// Include the Trigger service header from the std_srvs package
#include "std_srvs/Trigger.h"

// Include the SetBool service header from the std_srvs package
#include "std_srvs/SetBool.h"

#include "sensor_msgs/JointState.h"

// Structure to represent a detected part's pose and metadata
struct DetectedPart {
    geometry_msgs::Pose part_pose;    // The pose of the detected part
    std::string source_frame_id;      // The frame ID where the part was detected
    bool has_been_picked = false;     // Indicates if the part has been picked up
    bool marked_as_obsolete = false;  // Marks if the part is no longer detected
};

// Unordered map to store data from logical cameras
// Key: Camera name (e.g., "bin1", "agv1")
// Value: Latest image data from the corresponding logical camera
std::unordered_map<std::string, osrf_gear::LogicalCameraImage> cam_data;

// Global queue to store incoming orders
std::vector<osrf_gear::Order> order_queue;
std::map<std::string, std::vector<DetectedPart>> product_locations;

// Vector to hold the names of the robot's joints in order
std::vector<std::string> joint_names;

// Stores the real-time state of the robot's joints
sensor_msgs::JointState current_joint_states;

// Declare a transformation buffer to keep track of transforms
tf2_ros::Buffer tfBuffer;

// Instantiate a listener to update the transformation buffer
tf2_ros::TransformListener tfListener(tfBuffer);

// Function to start the ARIAC competition using the start_competition service
bool startCompetitionService(ros::NodeHandle n) {
    // Create a service client for the start_competition service
    ros::ServiceClient client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");

    // Declare the service request and response
    std_srvs::Trigger begin_comp;

    // Call the service and check the outcome
    if (client.call(begin_comp)) {
        if (begin_comp.response.success) {
            // Competition started successfully
            ROS_INFO("Competition successfully started: %s", begin_comp.response.message.c_str());
            return true;
        } else {
            // Service call succeeded but competition did not start
            ROS_WARN("Competition start failed: %s", begin_comp.response.message.c_str());
        }
    } else {
        // Service call failed
        ROS_ERROR("Failed to call start_competition service. Please check the service connection.");
    }
    return false;
}

std::mutex data_mutex;

// Callback function to handle incoming orders
void orderReceivedCallback(const osrf_gear::Order::ConstPtr& msg) {
    // Log the received order ID
    ROS_INFO("Order received: %s", msg->order_id.c_str());

    // Add the received order to the queue
    order_queue.push_back(*msg);
}

// Callback to process data from logical cameras
void processLogicalCameraData(const osrf_gear::LogicalCameraImage::ConstPtr& msg, const std::string& camera_name) {
    // Check if the camera has detected any models
    if (!msg->models.empty()) {
        // Log the number of detected objects
        ROS_INFO_STREAM("Logical camera [" << camera_name << "] detected " 
                        << msg->models.size() << " object(s).");

        // Update the global map with the latest camera data
        cam_data[camera_name] = *msg;
    } else {
        // Log that no objects were detected
        ROS_INFO_STREAM("Logical camera [" << camera_name << "] detected no objects.");
    }
}

// Function to transform a part's pose from the camera frame to the robot base frame
bool convertPoseToBaseFrame(const std::string& cameraFrame, const geometry_msgs::Pose& partPose, geometry_msgs::PoseStamped& transformedPose) {
    geometry_msgs::PoseStamped partPoseStamped;
    partPoseStamped.header.frame_id = cameraFrame;
    partPoseStamped.pose = partPose;

    try {
        geometry_msgs::TransformStamped transformStamped;
        transformStamped = tfBuffer.lookupTransform("arm1_base_link", cameraFrame, ros::Time(0.0), ros::Duration(1.0));
        tf2::doTransform(partPoseStamped, transformedPose, transformStamped);
        ROS_INFO("Transformed pose to arm frame: (%.2f, %.2f, %.2f)", 
            transformedPose.pose.position.x, transformedPose.pose.position.y, transformedPose.pose.position.z);
        return true;
    } catch (tf2::TransformException& ex) {
        ROS_ERROR("Transform error: %s", ex.what());
        return false;
    }
}

// Function to set up subscribers for all logical cameras
void initializeCameraSubscribers(ros::NodeHandle &n) {
    // List of logical camera topics to subscribe to
    const std::vector<std::string> cam_topics = {
        "/ariac/logical_camera_bin1",
        "/ariac/logical_camera_bin2",
        "/ariac/logical_camera_bin3",
        "/ariac/logical_camera_bin4",
        "/ariac/logical_camera_bin5",
        "/ariac/logical_camera_bin6",
        "/ariac/logical_camera_bin7",
        "/ariac/logical_camera_bin8",
        "/ariac/logical_camera_agv1",
        "/ariac/logical_camera_agv2",
        "/ariac/quality_control_sensor_1",
        "/ariac/quality_control_sensor_2"
    };

    // Subscribe to each camera topic with a bound callback
    for (const auto& topic : cam_topics) {
        ros::Subscriber sub = n.subscribe<osrf_gear::LogicalCameraImage>(
            topic, 
            10, 
            boost::bind(processLogicalCameraData, _1, topic)
        );

        // Log the successful subscription
        ROS_INFO_STREAM("Subscribed to logical camera topic: " << topic);
    }
}

// Function to get and validate joint names from the parameter server
bool retrieveJointNamesFromParamServer(std::vector<std::string>& joint_names) {
    // Retrieve joint names
    if (!ros::param::get("/ariac/arm1/arm/joints", joint_names)) {
        ROS_ERROR("Failed to retrieve joint names from the parameter server.");
        return false; // Return false if retrieval fails
    }

    // Log the retrieved joint names
    ROS_INFO("Retrieved joint names:");
    for (const auto& joint_name : joint_names) {
        ROS_INFO("  %s", joint_name.c_str());
    }

    // Validate the number of joints
    if (joint_names.size() != 7) {
        ROS_ERROR("Expected 7 joint names, but got %lu.", joint_names.size());
        return false; // Return false if validation fails
    }

    return true; // Return true if successful
}

// Function to wait until all joint_states are received
bool ensureJointStatesAvailable(const std::vector<std::string>& joint_names, int timeout_sec = 10) {
    ROS_INFO("Waiting for joint_states to be populated...");
    ros::Rate rate(10); // 10 Hz
    bool joint_states_received = false;
    ros::Time start_time = ros::Time::now();

    while (ros::ok() && !joint_states_received) {
        {
            std::lock_guard<std::mutex> lock(data_mutex);

            // Check if all required joints are in the joint_states message
            bool all_joints_found = true;
            for (const auto& joint_name : joint_names) {
                if (std::find(current_joint_states.name.begin(), current_joint_states.name.end(), joint_name) == current_joint_states.name.end()) {
                    all_joints_found = false;
                    break;
                }
            }

            // Verify positions are available for all joints
            if (all_joints_found && current_joint_states.position.size() >= joint_names.size()) {
                joint_states_received = true;
                ROS_INFO("All joint_states received. Proceeding with movement commands.");
                break;
            }
        }

        // Check for timeout
        if ((ros::Time::now() - start_time).toSec() > timeout_sec) {
            ROS_ERROR("Timeout reached while waiting for joint_states.");
            return false; // Return false if joint_states are not received in time
        }

        rate.sleep(); // Maintain loop rate
    }

    return joint_states_received;
}

// Function to ensure the IK service is available
bool ensureIKServiceReady(const std::string& service_name, int timeout_ms) {
    // Wait for the service with a specified timeout
    if (!ros::service::waitForService(service_name, ros::Duration(timeout_ms / 1000.0))) {
        ROS_WARN("%s service is not available after %d milliseconds.", service_name.c_str(), timeout_ms);
        return false; // Return false if service is not available
    }

    ROS_INFO("%s service is available.", service_name.c_str());
    return true; // Return true if service is available
}

// Callback function to update joint states
void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    // Lock mutex for thread-safe access
    std::lock_guard<std::mutex> lock(data_mutex);

    // Update the current joint states
    current_joint_states = *msg;

    // Throttled debug message
    ROS_DEBUG_THROTTLE(5, "Joint states updated.");
}

// Function to initialize service clients, publishers, subscribers, and the spinner
void initializeROSComponents(ros::NodeHandle& n, 
                              ros::ServiceClient& ik_client, 
                              ros::Publisher& trajectory_pub, 
                              ros::Subscriber& joint_states_sub) {
    // Initialize IK service client
    ik_client = n.serviceClient<ik_service::PoseIK>("pose_ik");
    ROS_INFO("Initialized IK service client for 'pose_ik'.");

    // Initialize publisher for joint trajectories
    trajectory_pub = n.advertise<trajectory_msgs::JointTrajectory>("/ariac/arm1/arm/command", 10);
    ROS_INFO("Initialized joint trajectory publisher for '/ariac/arm1/arm/command'.");

    // Initialize subscriber for joint states
    joint_states_sub = n.subscribe("/ariac/arm1/joint_states", 10, jointStatesCallback);
    ROS_INFO("Initialized joint states subscriber for '/ariac/arm1/joint_states'.");

    // Start the asynchronous spinner
    static ros::AsyncSpinner spinner(1); // Use 1 thread
    spinner.start();
    ROS_INFO("Async spinner started with 1 thread.");
}

// Function to request an IK solution with retries
bool requestIKSolution(ros::ServiceClient& ik_client, ik_service::PoseIK& ik_srv, int max_attempts = 5, double retry_delay = 1.0) {
    int attempts = 0;
    while (attempts < max_attempts) {
        if (ik_client.call(ik_srv) && ik_srv.response.num_sols > 0) {
            ROS_INFO("IK service call succeeded after %d attempt(s).", attempts + 1);
            return true;
        }
        ROS_WARN("IK service call failed on attempt %d. Retrying in %.1f seconds...", attempts + 1, retry_delay);
        ros::Duration(retry_delay).sleep();
        attempts++;
    }
    ROS_ERROR("IK service failed after %d attempts.", max_attempts);
    return false;
}

// Function to publish trajectory commands to the robot arm
void publishTrajectoryCommand(const std::vector<double>& joint_angles, ros::Publisher& trajectory_pub, const std::vector<std::string>& joint_names) {
    trajectory_msgs::JointTrajectory trajectory_msg;
    trajectory_msg.joint_names = joint_names;

    // Prepare the starting and target points
    trajectory_msgs::JointTrajectoryPoint start_point, target_point;

    {
        std::lock_guard<std::mutex> lock(data_mutex);

        // Set starting point to current joint states
        for (const auto& joint_name : joint_names) {
            auto it = std::find(current_joint_states.name.begin(), current_joint_states.name.end(), joint_name);
            if (it != current_joint_states.name.end()) {
                size_t index = std::distance(current_joint_states.name.begin(), it);
                start_point.positions.push_back(current_joint_states.position[index]);
            } else {
                ROS_ERROR("Joint name '%s' not found in joint states.", joint_name.c_str());
                return;
            }
        }
        start_point.time_from_start = ros::Duration(0.0);

        // Set target point to IK solution
        target_point.positions = joint_angles;
        target_point.time_from_start = ros::Duration(2.0); // 2 seconds to reach the target
    }

    trajectory_msg.points.push_back(start_point);
    trajectory_msg.points.push_back(target_point);
    trajectory_msg.header.stamp = ros::Time::now();

    // Publish the trajectory
    trajectory_pub.publish(trajectory_msg);
    ROS_INFO("Trajectory command published.");
}

// Function to create a list of predefined target poses
std::vector<geometry_msgs::Pose> creatSetPoses() {
    std::vector<geometry_msgs::Pose> poses;

    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;  // Default orientation
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;

    // Define target poses
    pose.position.x = 0.75; pose.position.y = 0.0; pose.position.z = 0.95; poses.push_back(pose);
    pose.position.x = 0.0;  pose.position.y = 0.75; pose.position.z = 0.95; poses.push_back(pose);
    pose.position.x = 0.25; pose.position.y = 0.75; pose.position.z = 0.95; poses.push_back(pose);
    pose.position.x = 0.75; pose.position.y = 0.25; pose.position.z = 0.95; poses.push_back(pose);
    pose.position.x = 0.75; pose.position.y = 0.45; pose.position.z = 0.75; poses.push_back(pose);

    return poses;
}

// Function to move the robot arm through a sequence of poses
void executeTargetPoseMovements(ros::NodeHandle& nh, tf2_ros::Buffer& tfBuffer, ros::Publisher& trajectory_pub) {
    // IK service client
    ros::ServiceClient ik_client = nh.serviceClient<ik_service::PoseIK>("pose_ik");

    // Ensure the IK service is available
    if (!ros::service::waitForService("pose_ik", ros::Duration(3.0))) {
        ROS_ERROR("IK service 'pose_ik' is not available. Exiting...");
        return;
    }

    // Retrieve joint names
    std::vector<std::string> joint_names;
    if (!ros::param::get("/ariac/arm1/arm/joints", joint_names)) {
        ROS_ERROR("Failed to retrieve joint names from the parameter server.");
        return;
    }

    // Create target poses
    std::vector<geometry_msgs::Pose> poses = creatSetPoses();

    // Iterate through each target pose
    for (const auto& target_pose : poses) {
        ik_service::PoseIK ik_srv;
        ik_srv.request.part_pose = target_pose;

        // Request IK solution with retries
        if (!requestIKSolution(ik_client, ik_srv)) {
            ROS_WARN("Skipping pose (%.2f, %.2f, %.2f) due to IK failure.", 
                     target_pose.position.x, target_pose.position.y, target_pose.position.z);
            continue;
        }

        // Use the first valid IK solution
        std::vector<double> joint_angles(ik_srv.response.joint_solutions[0].joint_angles.begin(),
                                  ik_srv.response.joint_solutions[0].joint_angles.end());

        // Publish the trajectory command
        publishTrajectoryCommand(joint_angles, trajectory_pub, joint_names);

        // Wait for the arm to reach the target pose
        ros::Rate rate(10); // 10 Hz
        while (ros::ok()) {
            {
                std::lock_guard<std::mutex> lock(data_mutex);
                if (std::all_of(current_joint_states.velocity.begin(), current_joint_states.velocity.end(),
                                [](double vel) { return std::abs(vel) < 0.01; })) {
                    ROS_INFO("Arm has reached pose (%.2f, %.2f, %.2f).", 
                             target_pose.position.x, target_pose.position.y, target_pose.position.z);
                    break;
                }
            }
            rate.sleep();
        }
    }

    ROS_INFO("Completed all target poses.");
}

// Function to retrieve joint names from the parameter server
bool retrieveJointNames(std::vector<std::string> &joint_names) {
    // Fetch joint names
    if (!ros::param::get("/ariac/arm1/arm/joints", joint_names)) {
        ROS_ERROR("Failed to retrieve joint names from the parameter server.");
        return false;
    }

    // Log the retrieved joint names
    ROS_INFO("Successfully retrieved joint names:");
    for (const auto &joint : joint_names) {
        ROS_DEBUG(" - %s", joint.c_str());
    }

    return true;
}

// Function to retrieve part locations using the material_locations service
void retrievePartLocation(ros::NodeHandle &nh, const std::string &product_type) {
    ros::ServiceClient client = nh.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
    osrf_gear::GetMaterialLocations srv;
    srv.request.material_type = product_type;

    if (!client.call(srv)) {
        ROS_ERROR("Failed to call material_locations service for product type: %s", product_type.c_str());
        return;
    }

    for (const auto &unit : srv.response.storage_units) {
        if (unit.unit_id == "belt") {
            ROS_WARN_STREAM("Skipping belt location for product type: " << product_type);
            continue;
        }

        ROS_INFO_STREAM("Product type: " << product_type << " located in unit: " << unit.unit_id);

        {
            std::lock_guard<std::mutex> lock(data_mutex);
            if (product_locations.count(product_type)) {
                for (const auto &part_location : product_locations[product_type]) {
                    const auto &pose = part_location.part_pose;
                    ROS_DEBUG_STREAM("Location details - x: " << pose.position.x
                                                             << ", y: " << pose.position.y
                                                             << ", z: " << pose.position.z);
                }
            } else {
                ROS_WARN_STREAM("No logical camera data found for product type: " << product_type);
            }
        }
    }
}

// Function to transform pose to the arm's base frame using tfBuffer
bool convertPoseToBaseFrame(const geometry_msgs::Pose &camera_pose, const std::string &source_frame, geometry_msgs::PoseStamped &arm_pose, tf2_ros::Buffer &tfBuffer) {
    try {
        // Obtain the transform from the source frame to the arm's base frame
        geometry_msgs::TransformStamped transform = tfBuffer.lookupTransform("arm1_base_link", source_frame, ros::Time(0), ros::Duration(1.0));

        // Create a stamped pose in the source frame
        geometry_msgs::PoseStamped stamped_camera_pose;
        stamped_camera_pose.pose = camera_pose;
        stamped_camera_pose.header.frame_id = source_frame;

        // Transform the pose to the arm's frame
        tf2::doTransform(stamped_camera_pose, arm_pose, transform);

        // Slightly raise the Z position to avoid collisions
        arm_pose.pose.position.z += 0.10;

        ROS_DEBUG_STREAM("Transformed pose to arm frame: x = " << arm_pose.pose.position.x
                                                               << ", y = " << arm_pose.pose.position.y
                                                               << ", z = " << arm_pose.pose.position.z);
        return true;
    } catch (tf2::TransformException &ex) {
        ROS_ERROR("Failed to transform pose from %s to arm1_base_link: %s", source_frame.c_str(), ex.what());
        return false;
    }
}

// Function to process incoming orders and perform tasks
void executeOrderProcessing(ros::NodeHandle &nh, ros::ServiceClient &ik_client, tf2_ros::Buffer &tfBuffer, ros::Publisher &trajectory_pub) {
    // Joint constraints to ensure safe IK solutions
    const double SHOULDER_LIFT_MIN = -M_PI / 4;     // -45 degrees
    const double SHOULDER_LIFT_MAX = M_PI / 4;      // +45 degrees
    const double WRIST_2_NEAR_PI_2 = M_PI / 2;      // Approximately π/2
    const double WRIST_2_NEAR_3PI_2 = 3 * M_PI / 2; // Approximately 3π/2
    const double WRIST_2_TOLERANCE = 0.1;           // Tolerance for wrist_2 joint

    // Retrieve joint names
    std::vector<std::string> joint_names;
    if (!retrieveJointNames(joint_names)) {
        ROS_ERROR("Failed to retrieve joint names. Aborting executeOrderProcessing.");
        return;
    }

    // Main loop to process orders
    while (ros::ok()) {
        // Check if there are any orders
        if (order_queue.empty()) {
            ROS_INFO_THROTTLE(10, "Waiting for orders...");
            ros::Duration(0.5).sleep();
            continue;
        }

        // Retrieve the current order
        osrf_gear::Order current_order;
        {
            std::lock_guard<std::mutex> lock(data_mutex);
            current_order = order_queue.front();
        }

        ROS_INFO("Processing order: %s", current_order.order_id.c_str());

        // Process each shipment in the order
        for (const auto &shipment : current_order.shipments) {
            ROS_INFO("Processing shipment type: %s", shipment.shipment_type.c_str());

            // Process each product in the shipment
            for (const auto &product : shipment.products) {
                ROS_INFO("Processing product type: %s", product.type.c_str());

                // Locate the part using cameras and material locations
                retrievePartLocation(nh, product.type);

                // Retrieve the part's pose from camera data
                geometry_msgs::Pose part_pose;
                std::string camera_frame;

                {
                    std::lock_guard<std::mutex> lock(data_mutex);
                    if (product_locations.find(product.type) == product_locations.end() || product_locations[product.type].empty()) {
                        ROS_WARN("No available parts of type: %s found in product_locations.", product.type.c_str());
                        continue;
                    }
                    // Use the first available part
                    part_pose = product_locations[product.type].front().part_pose;
                    camera_frame = product_locations[product.type].front().source_frame_id;
                }

                // Transform the part's pose to the robot arm frame
                geometry_msgs::PoseStamped goal_pose;
                convertPoseToBaseFrame(part_pose, camera_frame, goal_pose, tfBuffer);

                // Prepare IK service request
                ik_service::PoseIK ik_srv;
                ik_srv.request.part_pose = goal_pose.pose;

                // Request IK solution
                if (!requestIKSolution(ik_client, ik_srv)) {
                    ROS_ERROR("Failed to call IK service for product type: %s", product.type.c_str());
                    continue;
                }

                // Select a valid IK solution based on constraints
                std::vector<double> selected_solution;
                bool valid_solution_found = false;

                for (int i = 0; i < ik_srv.response.num_sols; ++i) {
                    const auto &solution = ik_srv.response.joint_solutions[i].joint_angles;

                    // Check joint constraints
                    double shoulder_lift_joint = solution[1]; // Index for shoulder_lift_joint
                    double wrist_2_joint = solution[4];       // Index for wrist_2_joint

                    if (shoulder_lift_joint < SHOULDER_LIFT_MIN || shoulder_lift_joint > SHOULDER_LIFT_MAX) {
                        continue; // Skip invalid solutions
                    }

                    if (std::abs(wrist_2_joint - WRIST_2_NEAR_PI_2) > WRIST_2_TOLERANCE &&
                        std::abs(wrist_2_joint - WRIST_2_NEAR_3PI_2) > WRIST_2_TOLERANCE) {
                        continue; // Skip invalid solutions
                    }

                    // Use the first valid solution
                    selected_solution = std::vector<double>(solution.begin(), solution.end());
                    valid_solution_found = true;
                    break;
                }

                if (!valid_solution_found) {
                    ROS_WARN("No valid IK solution found for product type: %s", product.type.c_str());
                    continue;
                }

                // Execute the IK solution by publishing a trajectory
                publishTrajectoryCommand(selected_solution, trajectory_pub, joint_names);

                // Wait for the robot arm to reach the target position
                ros::Rate rate(10); // 10 Hz
                while (ros::ok()) {
                    std::lock_guard<std::mutex> lock(data_mutex);
                    bool arm_at_rest = std::all_of(current_joint_states.velocity.begin(), current_joint_states.velocity.end(),
                                                   [](double vel) { return std::abs(vel) < 0.01; });

                    if (arm_at_rest) {
                        ROS_INFO("Robot arm has reached the target position for product type: %s", product.type.c_str());
                        break;
                    }
                    rate.sleep();
                }
            }
        }

        // Remove the processed order from the queue
        {
            std::lock_guard<std::mutex> lock(data_mutex);
            order_queue.erase(order_queue.begin());
        }

        ROS_INFO("Completed processing order: %s", current_order.order_id.c_str());
    }
}


void publishTrajectory(const sensor_msgs::JointState& joint_states, ros::Publisher& trajectory_pub) {
    trajectory_msgs::JointTrajectory joint_trajectory;

    // 定义关节的名称
    joint_trajectory.joint_names = {
        "linear_arm_actuator_joint", "shoulder_pan_joint", "shoulder_lift_joint",
        "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
    };

    // 配置中间点
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.resize(joint_trajectory.joint_names.size());

    // 从 joint_states 中设置初始位置
    for (size_t t_joint = 0; t_joint < joint_trajectory.joint_names.size(); ++t_joint) {
        for (size_t s_joint = 0; s_joint < joint_states.name.size(); ++s_joint) {
            if (joint_trajectory.joint_names[t_joint] == joint_states.name[s_joint]) {
                point.positions[t_joint] = joint_states.position[s_joint];
                break;
            }
        }
    }

    // 修改肘部关节（elbow_joint，第3个关节）的角度
    point.positions[3] += 0.1; // 调整角度增加 0.1 弧度

    // 设置线性执行器的位置
    point.positions[0] = joint_states.position[1];

    // 设置运动的持续时间
    point.time_from_start = ros::Duration(0.5);

    // 将点添加到轨迹中
    joint_trajectory.points.push_back(point);

    // 配置消息的头部
    static int count = 0;
    joint_trajectory.header.seq = count++;
    joint_trajectory.header.stamp = ros::Time::now();
    joint_trajectory.header.frame_id = "arm1_base_link";

    // 发布轨迹
    ROS_INFO("发布轨迹...");
    trajectory_pub.publish(joint_trajectory);
}


int main(int argc, char **argv){

    // Initialize ROS
    ros::init(argc, argv, "ariac_entry_node");

    // Create a node handle
    ros::NodeHandle n;

    // Initialize global data structures
    order_queue.clear();
    product_locations.clear();
    joint_names.clear();
    
    // Delay to ensure all systems are ready
    ros::Duration(1.0).sleep();

    // Retrieve and validate joint names
    if (!retrieveJointNamesFromParamServer(joint_names)) {
        ROS_ERROR("Joint name retrieval or validation failed. Exiting...");
        return 1;
    }

    // Wait for joint_states to be ready
    if (!ensureJointStatesAvailable(joint_names)) {
        ROS_ERROR("Failed to receive joint_states. Exiting...");
        return 1;
    }

    ROS_INFO("'/ariac/start_competition' service is starting please wait!");
    ros::service::waitForService("/ariac/start_competition");

    // Start the competition
    if (!startCompetitionService(n)) {
        ROS_ERROR("Unable to start '/ariac/start_competition'. Shutting down node.");
        ros::shutdown();
    }

    // Declare the transformation buffer
    tf2_ros::Buffer tfBuffer;
    // Instantiate the transformation listener
    tf2_ros::TransformListener tfListener(tfBuffer);

    // Subscribe to the /ariac/orders topic
    ros::Subscriber order_subscriber = n.subscribe("/ariac/orders", 10, orderReceivedCallback);

    // Initialize camera subscribers
    initializeCameraSubscribers(n);

    // Ensure the IK service is ready
    if (!ensureIKServiceReady("pose_ik", 10000)) {
        ROS_ERROR("IK service is unavailable. Exiting...");
        return 1; // Exit if the IK service is not ready
    }

    // Declare variables for ROS components
    ros::ServiceClient ik_client;
    ros::Publisher trajectory_pub;
    ros::Subscriber joint_states_sub;

    // Initialize ROS components
    initializeROSComponents(n, ik_client, trajectory_pub, joint_states_sub);

    ROS_INFO("Subscribed to /ariac/start_competition");

    // Start the asynchronous spinner
    ros::AsyncSpinner spinner(1); // Use 1 thread
    spinner.start();              // Start the spinner

    // Execute predefined target pose movements
    executeTargetPoseMovements(n, tfBuffer, trajectory_pub);    

    ros::spinOnce();

    // Process incoming orders
    executeOrderProcessing(n, ik_client, tfBuffer, trajectory_pub);

    return 0;

}

