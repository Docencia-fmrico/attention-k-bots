#ifndef NODE_GRAPH_HPP_

#define NODE_GRAPH_HPP_

#include "ros2_knowledge_graph/GraphNode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"


class NodeGraph : public rclcpp_lifecycle::LifecycleNode {
 public:
  NodeGraph();

  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT on_configure(const rclcpp_lifecycle::State& state);
  CallbackReturnT on_activate(const rclcpp_lifecycle::State& state);
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State& state);
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State& state);
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State& state);
  CallbackReturnT on_error(const rclcpp_lifecycle::State& state);

 private:
  std::shared_ptr<ros2_knowledge_graph::GraphNode> graph_;
  
};

#endif  // NODE_GRAPH_HPP_