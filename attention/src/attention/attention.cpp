#include "attention/attention.hpp"

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

AttentionNode::AttentionNode() : rclcpp_lifecycle::LifecycleNode("graph_filliing_node") {
    gazeboSub_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
        "/gazebo/model_states", 10, std::bind(&AttentionNode::GazeboCallback, this, _1));
}

void AttentionNode::GazeboCallback(const gazebo_msgs::msg::ModelStates::SharedPtr msg) {
    if (!graph_initialized_) return;
    
    for (int i = 0; i < msg->name.size(); i++){
      
        ros2_knowledge_graph_msgs::msg::Node object = ros2_knowledge_graph::new_node(msg->name[i], "object");
        graph_->update_node(object);

        geometry_msgs::msg::TransformStamped tf;
        tf.header.frame_id = "odom";
        tf.header.stamp = now();
        tf.child_frame_id = msg->name[i];
        tf.transform.translation.x = msg->pose[i].position.x;
        tf.transform.translation.y = msg->pose[i].position.y;
        tf.transform.translation.z = msg->pose[i].position.z + 0.7;
        
        auto edge_tf = ros2_knowledge_graph::new_edge("odom", msg->name[i], tf, true);
        graph_->update_edge(edge_tf);
      
    }
}

CallbackReturnT AttentionNode::on_configure(const rclcpp_lifecycle::State& state) {
    RCLCPP_INFO(
        get_logger(), "[%s] Configuring from [%s] state...", get_name(),
        state.label().c_str());

    graph_ = std::make_shared<ros2_knowledge_graph::GraphNode>(shared_from_this());
    

    ros2_knowledge_graph_msgs::msg::Node robot = ros2_knowledge_graph::new_node("kbot", "robot");
    graph_->update_node(robot);

    ros2_knowledge_graph_msgs::msg::Node map = ros2_knowledge_graph::new_node("odom", "odom");
    graph_->update_node(map);
    graph_initialized_ = true;
    return CallbackReturnT::SUCCESS;
}
CallbackReturnT AttentionNode::on_activate(const rclcpp_lifecycle::State& state) {
    RCLCPP_INFO(
        get_logger(), "[%s] Activating from [%s] state...", get_name(),
        state.label().c_str());

    return CallbackReturnT::SUCCESS;
}

CallbackReturnT AttentionNode::on_deactivate(const rclcpp_lifecycle::State& state) {
    RCLCPP_INFO(
        get_logger(), "[%s] Deactivating from [%s] state...", get_name(),
        state.label().c_str());

    return CallbackReturnT::SUCCESS;
}

CallbackReturnT AttentionNode::on_cleanup(const rclcpp_lifecycle::State& state) {
    RCLCPP_INFO(
        get_logger(), "[%s] Cleanning Up from [%s] state...", get_name(),
        state.label().c_str());

    return CallbackReturnT::SUCCESS;
}

CallbackReturnT AttentionNode::on_shutdown(const rclcpp_lifecycle::State& state) {
    RCLCPP_INFO(
        get_logger(), "[%s] Shutting Down from [%s] state...", get_name(),
        state.label().c_str());

    return CallbackReturnT::SUCCESS;
}

CallbackReturnT AttentionNode::on_error(const rclcpp_lifecycle::State& state) {
    RCLCPP_INFO(
        get_logger(), "[%s] Shutting Down from [%s] state...", get_name(),
        state.label().c_str());
    return CallbackReturnT::SUCCESS;
}