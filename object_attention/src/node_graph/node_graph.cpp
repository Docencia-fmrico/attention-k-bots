#include "node_graph/node_graph.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "ros2_knowledge_graph/graph_utils.hpp"
#include "ros2_knowledge_graph/GraphNode.hpp"
#include "ros2_knowledge_graph_msgs/msg/edge.hpp"
#include "ros2_knowledge_graph_msgs/msg/graph.hpp"
#include "ros2_knowledge_graph_msgs/msg/node.hpp"
#include "ros2_knowledge_graph_msgs/msg/graph_update.hpp"

using std::placeholders::_1;

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

NodeGraph::NodeGraph() : rclcpp_lifecycle::LifecycleNode("node_graph") {}

CallbackReturnT NodeGraph::on_configure(const rclcpp_lifecycle::State& state) {
  graph_ = std::make_shared<ros2_knowledge_graph::GraphNode>(shared_from_this());
  RCLCPP_INFO(get_logger(), "[%s] Configuring from [%s] state...", get_name(),
              state.label().c_str());

  

  auto edge_tf = graph_->get_edges<geometry_msgs::msg::TransformStamped>("map", "bookshelf_4");

  
  auto content_tf_opt = ros2_knowledge_graph::get_content<geometry_msgs::msg::TransformStamped>(edge_tf[0].content);

  //std::cout << content_tf_opt.has_value()<< std::endl;

 

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