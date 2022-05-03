#ifndef ATTENTION__ATTENTION_HPP_

#define ATTENTION__ATTENTION_HPP_

#include "gazebo_msgs/msg/model_states.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "ros2_knowledge_graph/GraphNode.hpp"
using std::placeholders::_1;

class AttentionNode : public rclcpp_lifecycle::LifecycleNode {
   public:
    AttentionNode();

    using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CallbackReturnT on_configure(const rclcpp_lifecycle::State& state);
    CallbackReturnT on_activate(const rclcpp_lifecycle::State& state);
    CallbackReturnT on_deactivate(const rclcpp_lifecycle::State& state);
    CallbackReturnT on_cleanup(const rclcpp_lifecycle::State& state);
    CallbackReturnT on_shutdown(const rclcpp_lifecycle::State& state);
    CallbackReturnT on_error(const rclcpp_lifecycle::State& state);

   private:
    void GazeboCallback(const gazebo_msgs::msg::ModelStates::SharedPtr msg);
    std::shared_ptr<ros2_knowledge_graph::GraphNode> graph_;
    bool graph_initialized_ = false;
    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr gazeboSub_;
};

#endif  // ATTENTION__ATTENTION_HPP_