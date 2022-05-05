#ifndef NODE_GRAPH_HPP_

#define NODE_GRAPH_HPP_

#include "ros2_knowledge_graph/GraphNode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

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
  void do_work();
  std::vector<std::string> find_objects(std::string object_to_find);
  std::map<std::string, std::vector<float>> select_object();
  void look_for_object();
  void watch_object();

 private:
  std::shared_ptr<ros2_knowledge_graph::GraphNode> graph_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp_lifecycle::LifecyclePublisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_cmd_pub_;
  float MAX_DISTANCE = 5;
};

#endif  // NODE_GRAPH_HPP_