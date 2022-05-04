#include "node_graph/node_graph.hpp"

#include "gazebo_msgs/msg/model_states.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "ros2_knowledge_graph/GraphNode.hpp"
#include "ros2_knowledge_graph/graph_utils.hpp"
#include "ros2_knowledge_graph_msgs/msg/edge.hpp"
#include "ros2_knowledge_graph_msgs/msg/graph.hpp"
#include "ros2_knowledge_graph_msgs/msg/graph_update.hpp"
#include "ros2_knowledge_graph_msgs/msg/node.hpp"
#include <math.h>

using std::placeholders::_1;
using namespace std::chrono_literals;
using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

NodeGraph::NodeGraph() : rclcpp_lifecycle::LifecycleNode("node_graph") {
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

CallbackReturnT NodeGraph::on_configure(const rclcpp_lifecycle::State& state) {
  joint_cmd_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/head_controller/joint_trajectory", 100);

  graph_ = std::make_shared<ros2_knowledge_graph::GraphNode>(shared_from_this());

  RCLCPP_INFO(get_logger(), "[%s] Configuring from [%s] state...", get_name(),
              state.label().c_str());


  return CallbackReturnT::SUCCESS;
}
CallbackReturnT NodeGraph::on_activate(const rclcpp_lifecycle::State& state) {
  joint_cmd_pub_->on_activate();
  RCLCPP_INFO(get_logger(), "[%s] Activating from [%s] state...", get_name(),
              state.label().c_str());
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT NodeGraph::on_deactivate(const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), "[%s] Deactivating from [%s] state...", get_name(),
              state.label().c_str());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT NodeGraph::on_cleanup(const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), "[%s] Cleanning Up from [%s] state...", get_name(),
              state.label().c_str());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT NodeGraph::on_shutdown(const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), "[%s] Shutting Down from [%s] state...", get_name(),
              state.label().c_str());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT NodeGraph::on_error(const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), "[%s] Shutting Down from [%s] state...", get_name(),
              state.label().c_str());
  return CallbackReturnT::SUCCESS;
}

void NodeGraph::do_work() {
  std::string fromFrameRel = "cabinet";
  std::string toFrameRel = "head_1_link";
  geometry_msgs::msg::TransformStamped transformStamped;

  try {
    transformStamped = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);
    // transformStamped = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, this->get_clock()->now(), rclcpp::Duration(1,0));
  } catch (tf2::TransformException& ex) {
    RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", toFrameRel.c_str(),
                fromFrameRel.c_str(), ex.what());
    return;
  }
  std::cout << "X: " << transformStamped.transform.translation.x << " Y: " << transformStamped.transform.translation.y <<  " Z: " << transformStamped.transform.translation.z<< std::endl;
  
  float pan = atan2(transformStamped.transform.translation.y, transformStamped.transform.translation.x);
  float tilt = atan2(transformStamped.transform.translation.z, transformStamped.transform.translation.x);

  std::cout << "PAN: " << pan * 180 /M_PI << " TILT: " << tilt * 180 /M_PI << std::endl;
  
  trajectory_msgs::msg::JointTrajectory command_msg;
  trajectory_msgs::msg::JointTrajectoryPoint point;

  command_msg.header.stamp = this->get_clock()->now();
  command_msg.joint_names.push_back("head_1_joint");
  command_msg.joint_names.push_back("head_2_joint");

  
  point.positions.resize(2);
  point.velocities.resize(2);
  point.accelerations.resize(2);
  point.effort.resize(2);
  point.positions[0] = tilt;
  point.positions[1] = 0.0;
  point.velocities[0] = 10.0;
  point.velocities[1] = 10.0;
  point.accelerations[0] = 10.0;
  point.accelerations[1] = 10.0;
  point.effort[0] = 10.0;
  point.effort[1] = 10.0;

  point.time_from_start = rclcpp::Duration(1s);
  command_msg.points.resize(1);
  command_msg.points.at(0) = point;
  
  joint_cmd_pub_->publish(command_msg);
}
