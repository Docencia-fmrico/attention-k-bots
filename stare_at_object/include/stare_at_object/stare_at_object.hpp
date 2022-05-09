#ifndef STARE_AT_OBJECT_HPP_

#define STARE_AT_OBJECT_HPP_

#include "ros2_knowledge_graph/GraphNode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

class StareNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  StareNode();

  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT on_configure(const rclcpp_lifecycle::State& state);
  CallbackReturnT on_activate(const rclcpp_lifecycle::State& state);
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State& state);
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State& state);
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State& state);
  CallbackReturnT on_error(const rclcpp_lifecycle::State& state);
  void do_work();
  std::vector<std::string> find_objects();
  void select_object();
  void look_for_object();

 private:
  std::shared_ptr<ros2_knowledge_graph::GraphNode> graph_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp_lifecycle::LifecyclePublisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_cmd_pub_;
  
  std::vector<std::string> objects_names_;
  std::vector<std::vector<float>> objects_angle_;
  double fovea_time_ = 0.0;
  double distance_ = 5.0;
  double prev_exploration_ = 0.0;
  double prev_look_to_ = 0.0;
  int size_points_ = 0;
  int object_to_see = 0;
};

#endif  // STARE_AT_OBJECT_HPP_