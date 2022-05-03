#ifndef ATTENTION__ATTENTION_HPP_

#define ATTENTION__ATTENTION_HPP_



#include "ros2_knowledge_graph/GraphNode.hpp"
#include "gazebo_msgs/msg/model_states.hpp"

using std::placeholders::_1;

class AttentionNode : public rclcpp::Node
{
public:
  AttentionNode();

private:
  void GazeboCallback(const gazebo_msgs::msg::ModelStates::SharedPtr msg); 
  std::shared_ptr<ros2_knowledge_graph::GraphNode> graph_;
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr gazeboSub_;

};


#endif  // ATTENTION__ATTENTION_HPP_