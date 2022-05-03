#include "geometry_msgs/msg/twist.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "attention/attention.hpp"

#include "gazebo_msgs/msg/model_states.hpp"
#include "ros2_knowledge_graph/GraphNode.hpp"
using std::placeholders::_1;

AttentionNode::AttentionNode() : rclcpp_lifecycle::LifecycleNode("node_Attention") {
  
  graph_ = std::make_shared<ros2_knowledge_graph::GraphNode>(shared_from_this());

  gazeboSub_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
      "/gazebo/model_states", 10, std::bind(&AttentionNode::GazeboCallback, this, _1));
}

void AttentionNode::GazeboCallback(const gazebo_msgs::msg::ModelStates::SharedPtr msg) {
  std::cout << msg->name.at(0) << std::endl;
}
