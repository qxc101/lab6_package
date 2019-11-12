#include "ros/ros.h"
#include "std_msgs/String.h"
// Include the header file for Trigger.
#include "std_srvs/Trigger.h"
#include <sstream>
#include "osrf_gear/Order.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"
// Include a file from the ur_kinematics package
#include "ur_kinematics/ur_kin.h"
// Transformation header files
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"

std::vector<osrf_gear::Order> order_vector;
osrf_gear::LogicalCameraImage image_from_cam;
osrf_gear::JointStates joint_receieved;

void order_callback(const osrf_gear::Order::ConstPtr & order_msg) {
    ROS_INFO_STREAM("Received order:\n" << *order_msg);
   order_vector.push_back(*order_msg);
  }

void joint_callback(const osrf_gear::JointState::ConstPtr & joint_msg) {
   joint_receieved = *joint_msg;
  }

void cam_callback(const osrf_gear::LogicalCameraImage::ConstPtr& cam_msg) {
  image_from_cam = *cam_msg;
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Subscriber orders_subscriber = n.subscribe("/ariac/orders", 10,order_callback);
  ros::Subscriber cam_subscriber = n.subscribe("/ariac/logical_camera", 10, cam_callback);
  ros::Subscriber joint_subscriber = n.subscribe("/ariac/joint_states", 10, joint_callback);
  ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  ros::ServiceClient material_client = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");  
  ros::Rate loop_rate(10);
  order_vector.clear();
  
  // This function made our program more stable
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std_srvs::Trigger start_srv; 
  begin_client.call(start_srv);
  if (!start_srv.response.success) {
    ROS_ERROR_STREAM("Failed to start the competition: " << start_srv.response.message);
  } else {
    ROS_INFO("Competition started!");
  }
  // Declare the transformation buffer to maintain a list of transformations
  tf2_ros::Buffer tfBuffer;
  // Instantiate a listener that listens to the tf and tf_static topics and to update
  // the buffer.
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  // main loop
  while (ros::ok()){
  if (order_vector.size() > 0) {
      ROS_INFO("New Order Received");
      osrf_gear::GetMaterialLocations locations;
      locations.request.material_type = order_vector[0].kits[0].objects[0].type;
      int i = 0;
      if(material_client.call(locations)) {
        ROS_INFO("Part is located at %s", locations.response.storage_units[0].unit_id.c_str());

        for(i = 0; i < image_from_cam.models.size(); i++) {
          if (image_from_cam.models[i].type == order_vector[0].kits[0].objects[0].type)
            break;
        }
	ROS_INFO("location: x [%2.2f] y [%2.2f] z [%2.2f]", image_from_cam.models[i].pose.position.x, image_from_cam.models[i].pose.position.y, image_from_cam.models[i].pose.position.z);
        
        // Retrieve the transformation
	geometry_msgs::TransformStamped tfStamped;
        try {
          // move_group.getPlanningFrame().c_str() don't work
          tfStamped = tfBuffer.lookupTransform("world", "logical_camera_frame", ros::Time(0.0), ros::Duration(1.0));
          ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(), tfStamped.child_frame_id.c_str());
        } catch (tf2::TransformException &ex) {
          ROS_ERROR("%s", ex.what());
        }
	// tf2_ross::Buffer.lookupTransform(“to_frame”, “from_frame”, “how_recent”, “how_long_to_wait”);
	// Create variables
	geometry_msgs::PoseStamped part_pose, goal_pose;
	// Copy pose from the logical camera. part_pose.pose = ...;
	part_pose.pose = image_from_cam.models[i].pose;
	tf2::doTransform(part_pose, goal_pose, tfStamped);
	// Add height to the goal pose.
	goal_pose.pose.position.z += 0.10; // 10 cm above the part
	// Tell the end effector to rotate 90 degrees around the y-axis (in quaternions...
	// more on quaternions later in the semester).
	goal_pose.pose.orientation.w = 0.707;
	goal_pose.pose.orientation.x = 0.0;
	goal_pose.pose.orientation.y = 0.707;
	goal_pose.pose.orientation.z = 0.0;
	// Set the desired pose for the arm in the arm controller.
	

	}
    }
    loop_rate.sleep();
}

  return 0;
}
