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
#include "ros2_knowledge_graph_msgs/msg/node.hpp"
using std::placeholders::_1;

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

NodeGraph::NodeGraph() : rclcpp_lifecycle::LifecycleNode("node_graph") {}

CallbackReturnT NodeGraph::on_configure(const rclcpp_lifecycle::State& state) {
  graph_ = std::make_shared<ros2_knowledge_graph::GraphNode>(shared_from_this());
  RCLCPP_INFO(get_logger(), "[%s] Configuring from [%s] state...", get_name(),
              state.label().c_str());

  ros2_knowledge_graph_msgs::msg::Node robot = ros2_knowledge_graph::new_node("kbot", "robot");
  ros2_knowledge_graph_msgs::msg::Node table = ros2_knowledge_graph::new_node("table", "object");

  graph_->update_node(robot);
  graph_->update_node(table);

  ros2_knowledge_graph_msgs::msg::Edge edge = ros2_knowledge_graph::new_edge<std::string>("kbot", "table", "sees");

  graph_->update_edge(edge);

  return CallbackReturnT::SUCCESS;
}
CallbackReturnT NodeGraph::on_activate(const rclcpp_lifecycle::State& state) {
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