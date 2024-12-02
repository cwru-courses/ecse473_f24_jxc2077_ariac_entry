#include "algorithm"
#include "vector"
#include "ros/ros.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/Proximity.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_srvs/Trigger.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "ariac_entry/Bins.h"
#include "ariac_entry/Bin.h"
#include "ik_service/PoseIK.h"
#include "ur_kinematics/ur_kin.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "control_msgs/FollowJointTrajectoryAction.h"

std::vector<osrf_gear::Order> order_vector;
std::vector<ariac_entry::Bin> bin_vector(6);
std::vector<ariac_entry::Bin> agv_vector(2);
std::vector<ariac_entry::Bin> fault_vector(2);
ariac_entry::Bins bin_contents;
ros::ServiceClient find_bin;
ros::ServiceClient ik_client;
ros::Publisher joint_trajectory_pub;
sensor_msgs::JointState joint_states;
tf2_ros::Buffer tfBuffer;


//actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> trajectory_ac("ariac/arm/follow_joint_trajectory", true);

bool sort_agv1 = false;
bool sort_agv2 = false;

bool sort_bin1 = false;
bool sort_bin2 = false;
bool sort_bin3 = false;
bool sort_bin4 = false;
bool sort_bin5 = false;
bool sort_bin6 = false;

bool sort_faulty1 = false;
bool sort_faulty2 = false;

//Starts the competition
void start_competition(ros::NodeHandle &node) {
  order_vector.clear();
  ros::ServiceClient begin_client = node.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  std_srvs::Trigger begin_comp;
  int service_call_succeeded;
  
  if(!begin_client.exists()) {
    ROS_INFO("Waiting for the competition to be ready...");
    begin_client.waitForExistence();
    ROS_INFO("Competition is now ready");
  }
  ROS_INFO("Request competition start...");
  service_call_succeeded = begin_client.call(begin_comp);
  if(!service_call_succeeded) {
    ROS_ERROR_STREAM("Failed to start the competion: " << begin_comp.response.message);
  } else {
    ROS_INFO("Competition started succesfully: %s", begin_comp.response.message.c_str());
  }
}

//Identifies the bin of the first product in the first shipment of the first order
void get_bin_of_first(ros::NodeHandle & node) {
  ros::ServiceClient get_bin_of_first_client = node.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
  osrf_gear::GetMaterialLocations get_bin;
  get_bin.request.material_type = order_vector.front().shipments.front().products.front().type.c_str();
  get_bin_of_first_client.call(get_bin);

  for(const osrf_gear::StorageUnit& storage_unit : get_bin.response.storage_units) {
    std::string bin ("bin");
    std::string unit_id = storage_unit.unit_id.c_str();
    std::string material_type = get_bin.request.material_type.c_str();
    if(unit_id.find(bin) != std::string::npos) {
      ROS_INFO("The first product of the first shipment in the first order is '%s' and is found in '%s'", get_bin.request.material_type.c_str(), unit_id.c_str());
    } 
  }
}

//Prints Each product's type, where that product type is located, and the pose for every product in a received order
void order_callback(const osrf_gear::Order::ConstPtr& order_msg) {
  ROS_INFO_STREAM("Received order: " << *order_msg);
  order_vector.push_back(*order_msg);

  osrf_gear::Order current_order = *order_msg;
  osrf_gear::GetMaterialLocations get_bin;

  for(const osrf_gear::Shipment& shipment : current_order.shipments) {
    for(const osrf_gear::Product& product : shipment.products) {
      get_bin.request.material_type = product.type.c_str();
      find_bin.call(get_bin);
      for(const osrf_gear::StorageUnit& storage_unit : get_bin.response.storage_units) {
         ROS_WARN("The '%s' is found in: %s", get_bin.request.material_type.c_str(), storage_unit.unit_id.c_str());
      }
      ROS_WARN_STREAM("The pose of the product is:\n " << product.pose);
    }
  }
  
}

//Logical camera of AGV1 that takes in camera image
void log_cam_agv1_callback(const osrf_gear::LogicalCameraImage::ConstPtr & cam_msg) {
  osrf_gear::LogicalCameraImage temp_msg = *cam_msg;
  ariac_entry::Bin agv1;
  agv1.image = temp_msg;
  if(!sort_agv1) {
    if(!temp_msg.models.empty()){
      agv1.bin_name = "AGV 1";
      agv1.material_type = "any";
      agv_vector[0] = agv1;
      sort_agv1 = true;
    } else {
      agv1.bin_name = "AGV 1";
      agv1.material_type = "any";
      agv_vector[0] = agv1;
      sort_agv1 = true;
      ROS_WARN_ONCE("Nothing in agv1");
    }
  }
}

//Logical camera of AGV2 that takes in camera image
void log_cam_agv2_callback(const osrf_gear::LogicalCameraImage::ConstPtr & cam_msg) {
  osrf_gear::LogicalCameraImage temp_msg = *cam_msg;
  ariac_entry::Bin agv2;
  agv2.image = temp_msg;
  if(!sort_agv2) {
    if(!temp_msg.models.empty()){
      agv2.bin_name = "AGV 2";
      agv2.material_type = "any";
      agv_vector[1] = agv2;
      sort_agv2 = true;
    } else {
      agv2.bin_name = "AGV 2";
      agv2.material_type = "any";
      agv_vector[1] = agv2;
      sort_agv2 = true;
      ROS_WARN_ONCE("Nothing in agv2");
    }
  }
}

void log_cam_bin1_callback(const osrf_gear::LogicalCameraImage::ConstPtr & cam_msg) {
  osrf_gear::LogicalCameraImage temp_msg = *cam_msg;
  ariac_entry::Bin bin1;
  bin1.image = temp_msg;
  if(!sort_bin1) {
    if(!temp_msg.models.empty()) {
        bin1.bin_name = "Bin 1";
        bin1.material_type = temp_msg.models.front().type.c_str();
        bin_vector[0] = bin1;
        sort_bin1 = true;
    } else {
        bin1.bin_name = "Bin 1";
        bin1.material_type = "Empty";
        bin_vector[0] = bin1;
        sort_bin1 = true;
        ROS_WARN_ONCE("Nothing in bin 1");
    }
  }
}

void log_cam_bin2_callback(const osrf_gear::LogicalCameraImage::ConstPtr & cam_msg) {
  osrf_gear::LogicalCameraImage temp_msg = *cam_msg;
  ariac_entry::Bin bin2;
  bin2.image = temp_msg;
  if(!sort_bin2) {
    if(!temp_msg.models.empty()) {
        bin2.bin_name = "Bin 2";
        bin2.material_type = temp_msg.models.front().type.c_str();
        bin_vector[1] = bin2;
        sort_bin2 = true;
    } else {
        bin2.bin_name = "Bin 2";
        bin2.material_type = "Empty";
        bin_vector[1] = bin2;
        sort_bin2 = true;
        ROS_WARN_ONCE("Nothing in bin 2");
    }
  }
}

void log_cam_bin3_callback(const osrf_gear::LogicalCameraImage::ConstPtr & cam_msg) {
  osrf_gear::LogicalCameraImage temp_msg = *cam_msg;
  ariac_entry::Bin bin3;
  bin3.image = temp_msg;
  if(!sort_bin3) {
    if(!temp_msg.models.empty()) {
        bin3.bin_name = "Bin 3";
        bin3.material_type = temp_msg.models.front().type.c_str();
        bin_vector[2] = bin3;
        sort_bin3 = true;
    } else {
        bin3.bin_name = "Bin 3";
        bin3.material_type = "Empty";
        bin_vector[2] = bin3;
        sort_bin3 = true;
        ROS_WARN_ONCE("Nothing in bin 3");
    }
  }
}

void log_cam_bin4_callback(const osrf_gear::LogicalCameraImage::ConstPtr & cam_msg) {
  osrf_gear::LogicalCameraImage temp_msg = *cam_msg;
  ariac_entry::Bin bin4;
  bin4.image = temp_msg;
  if(!sort_bin4) {
    if(!temp_msg.models.empty()) {
        bin4.bin_name = "Bin 4";
        bin4.material_type = temp_msg.models.front().type.c_str();
        bin_vector[3] = bin4;
        sort_bin4 = true;
    } else {
        bin4.bin_name = "Bin 4";
        bin4.material_type = "Empty";
        bin_vector[3] = bin4;
        sort_bin4 = true;
        ROS_WARN_ONCE("Nothing in bin 4");
    }
  }
}

void log_cam_bin5_callback(const osrf_gear::LogicalCameraImage::ConstPtr & cam_msg) {
  osrf_gear::LogicalCameraImage temp_msg = *cam_msg;
  ariac_entry::Bin bin5;
  bin5.image = temp_msg;
  if(!sort_bin5) {
    if(!temp_msg.models.empty()) {
        bin5.bin_name = "Bin 5";
        bin5.material_type = temp_msg.models.front().type.c_str();
        bin_vector[4] = bin5;
        sort_bin5 = true;
      
    } else {
        bin5.bin_name = "Bin 5";
        bin5.material_type = "Empty";
        bin_vector[4] = bin5;
        ROS_WARN_ONCE("Nothing in bin 5");
    }
  }
}

void log_cam_bin6_callback(const osrf_gear::LogicalCameraImage::ConstPtr & cam_msg) {
  osrf_gear::LogicalCameraImage temp_msg = *cam_msg;
  ariac_entry::Bin bin6;
  bin6.image = temp_msg;
  if(!sort_bin6) {
    if(!temp_msg.models.empty()) {
        bin6.bin_name = "Bin 6";
        bin6.material_type = temp_msg.models.front().type.c_str();
        bin_vector[5] = bin6;
        sort_bin6 = true;
    } else {
        bin6.bin_name = "Bin 6";
        bin6.material_type = "Empty";
        bin_vector[5] = bin6;
        ROS_WARN_ONCE("Nothing in bin 6");
        sort_bin6 = true;
    }
  }
}

void log_cam_faulty1_callback(const osrf_gear::LogicalCameraImage::ConstPtr & cam_msg) {
  osrf_gear::LogicalCameraImage temp_msg = *cam_msg;
  ariac_entry::Bin faulty1;
  faulty1.image = temp_msg;
  if(!sort_faulty1) {
    if(!temp_msg.models.empty()){
      faulty1.bin_name = "Qaulity Control 1";
      faulty1.material_type = "any";
      fault_vector[0] = faulty1;
      sort_faulty1 = true;
    } else {
      faulty1.bin_name = "Quality Control 1";
      faulty1.material_type = "any";
      fault_vector[0] = faulty1;
      sort_faulty1 = true;
      ROS_WARN_ONCE("Nothing in Quality Control 1");
    }
  }
}

void log_cam_faulty2_callback(const osrf_gear::LogicalCameraImage::ConstPtr & cam_msg) {
  osrf_gear::LogicalCameraImage temp_msg = *cam_msg;
  ariac_entry::Bin faulty2;
  faulty2.image = temp_msg;
  if(!sort_faulty2) {
    if(!temp_msg.models.empty()){
      faulty2.bin_name = "Qaulity Control 2";
      faulty2.material_type = "any";
      fault_vector[1] = faulty2;
      sort_faulty2 = true;
    } else {
      faulty2.bin_name = "Quality Control 2";
      faulty2.material_type = "any";
      fault_vector[1] = faulty2;
      sort_faulty2 = true;
      ROS_WARN_ONCE("Nothing in Quality Control 2");
    }
  }
}

void goalActiveCallback(){
  ROS_INFO("Goal active");
}

void feedbackCallback(const control_msgs::FollowJointTrajectoryFeedback::ConstPtr & fb) {
  ROS_INFO_STREAM(fb);
}

void resultCallback(const control_msgs::FollowJointTrajectoryResult::ConstPtr & res) {
  ROS_INFO_STREAM(res);
}

void joint_states_callback(const sensor_msgs::JointState::ConstPtr & joint_msg) {
  joint_states = *joint_msg;
}

void move_arm(trajectory_msgs::JointTrajectory & joint_trajectory) {
  control_msgs::FollowJointTrajectoryAction joint_trajectory_as;
  control_msgs::FollowJointTrajectoryGoal goal;
  joint_trajectory_as.action_goal.goal.trajectory = joint_trajectory;
  //ROS_WARN_STREAM_ONCE("MOVE_ARM" << joint_trajectory);
  
  //trajectory_ac.sendGoal(goal, &resultCallback, &goalActiveCallback, &feedbackCallback);
}

void get_trajectory(geometry_msgs::PoseStamped goal_pose) {

  
  trajectory_msgs::JointTrajectory joint_trajectory;
  int num_sols;
  int count;
  double q_desired[8][6];

  ik_service::PoseIK ik_srv;
  ik_srv.request.part_pose = goal_pose.pose;
  if(ik_client.call(ik_srv)) {
    num_sols = ik_srv.response.num_sols;
    joint_trajectory.header.seq = count++;
    joint_trajectory.header.stamp = ros::Time::now();
    joint_trajectory.header.frame_id = "/world";

    joint_trajectory.joint_names.clear();
    joint_trajectory.joint_names.push_back("linear_arm_actuator_joint");
    joint_trajectory.joint_names.push_back("shoulder_pan_joint");
    joint_trajectory.joint_names.push_back("shoulder_lift_joint");
    joint_trajectory.joint_names.push_back("elbow_joint");
    joint_trajectory.joint_names.push_back("wrist_1_joint");
    joint_trajectory.joint_names.push_back("wrist_2_joint");
    joint_trajectory.joint_names.push_back("wrist_3_joint");

    joint_trajectory.points.resize(2);
    joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());
    
    for (int indy = 0; indy < joint_trajectory.joint_names.size(); indy++) {
      for (int indz = 0; indz < joint_states.name.size(); indz++) {
        if(joint_trajectory.joint_names[indy] == joint_states.name[indz]) {
          joint_trajectory.points[0].positions[indy] = joint_states.position[indz];
          break;
        }
      }
    }
    
    joint_trajectory.points[0].time_from_start = ros::Duration(0.0);
    int q_des_index = 0;
    joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());
    joint_trajectory.points[1].positions[0] = joint_states.position[1];

    for(int i = 0; i < 8; i++) {
      for(int j = 0; j < 6; j++) {
        q_desired[i][j] = ik_srv.response.joint_solutions[i].joint_angles[j];
      }
    }
    
    for (int indy = 0; indy < 6; indy++) {
      joint_trajectory.points[1].positions[indy + 1] = q_desired[q_des_index][indy];
    }
    
    joint_trajectory.points[1].positions[0] = goal_pose.pose.position.y - 0.5;
    
   
    

    joint_trajectory.points[1].time_from_start = ros::Duration(3.0);
    ROS_WARN("PUBLISHING...");
    ROS_WARN_STREAM_ONCE(joint_trajectory);
    joint_trajectory_pub.publish(joint_trajectory);
    
    ROS_WARN("FINISH PUBLISHING");
    ros::Duration(3.0).sleep();
    
  
  } else {
    ROS_WARN_ONCE("Client failed to start");
  }
  
}



void get_transform(std::vector<ariac_entry::Bin> & bin_vector_input) {

 geometry_msgs::TransformStamped tfStamped;
 geometry_msgs::PoseStamped part_pose, goal_pose;

  try {
        tfStamped = tfBuffer.lookupTransform("arm1_base_link", "logical_camera_bin4_frame", ros::Time(0.0), ros::Duration(1.0));
        ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(), tfStamped.child_frame_id.c_str());
  } catch (tf2::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        
  }
  
  part_pose.pose = bin_vector_input.at(3).image.models.at(1).pose;   
  
  tf2::doTransform(part_pose, goal_pose, tfStamped);

  goal_pose.pose.position.z += 0.10;
  goal_pose.pose.orientation.w = 0.707;
  goal_pose.pose.orientation.x = 0.0;
  goal_pose.pose.orientation.y = 0.707;
  goal_pose.pose.orientation.z = 0.0;
  
  get_trajectory(goal_pose);

  
}



int main(int argc, char **argv)
{
  bool found_bin = false;
  ros::init(argc, argv, "ariac_entry_node");
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::NodeHandle node;
  ros::Subscriber orders_sub = node.subscribe("/ariac/orders", 10, order_callback);
  ros::Publisher bin_contents_pub = node.advertise<ariac_entry::Bins>("bin_contents", 10);
  joint_trajectory_pub = node.advertise<trajectory_msgs::JointTrajectory>("/ariac/arm1/arm/command",10);
  

  find_bin = node.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");

  ros::Subscriber log_cam_agv1_sub = node.subscribe("/ariac/logical_camera_agv1", 10, log_cam_agv1_callback);
  ros::Subscriber log_cam_agv2_sub = node.subscribe("/ariac/logical_camera_agv2", 10, log_cam_agv2_callback);

  ros::Subscriber log_cam_bin1_sub = node.subscribe("/ariac/logical_camera_bin1", 10, log_cam_bin1_callback);
  ros::Subscriber log_cam_bin2_sub = node.subscribe("/ariac/logical_camera_bin2", 10, log_cam_bin2_callback);
  ros::Subscriber log_cam_bin3_sub = node.subscribe("/ariac/logical_camera_bin3", 10, log_cam_bin3_callback);
  ros::Subscriber log_cam_bin4_sub = node.subscribe("/ariac/logical_camera_bin4", 10, log_cam_bin4_callback);
  ros::Subscriber log_cam_bin5_sub = node.subscribe("/ariac/logical_camera_bin5", 10, log_cam_bin5_callback);
  ros::Subscriber log_cam_bin6_sub = node.subscribe("/ariac/logical_camera_bin6", 10, log_cam_bin6_callback);
  
  
  ros::Subscriber log_cam_faulty1_sub = node.subscribe("/ariac/quality_control_sensor_1", 10, log_cam_faulty1_callback);
  ros::Subscriber log_cam_faulty2_sub = node.subscribe("/ariac/quality_control_sensor_2", 10, log_cam_faulty2_callback);
  
  ros::Subscriber joint_state_sub = node.subscribe("/ariac/arm1/joint_states", 10, joint_states_callback);

  //actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> trajectory_ac("ariac/arm/follow_joint_trajectory", true);
  ik_client = node.serviceClient<ik_service::PoseIK>("pose_ik");
  

  start_competition(node);
  ros::service::waitForService("/ariac/material_locations", 10);
  ros::service::waitForService("pose_ik", 10);

  ros::Rate loop_rate(10);
  int count = 0;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while(ros::ok()) {
    
    ROS_INFO_STREAM_THROTTLE(10, joint_states);
    if (order_vector.size() == 1 && !found_bin){
      get_bin_of_first(node);
      found_bin = true;
      if((!bin_vector.empty())) {
        ROS_WARN_ONCE("GETTING TRANSFORM");
        get_transform(bin_vector);
        
      } else {
        ROS_WARN("No data");
      }
    } else {
      ROS_WARN_ONCE("No orders received yet...");
    }
    
    bin_contents.bins = bin_vector;
    bin_contents_pub.publish(bin_contents);
   
    loop_rate.sleep();
    ++count;
  }
  

  ROS_INFO("Setup complete");

  return 0;
  

}

