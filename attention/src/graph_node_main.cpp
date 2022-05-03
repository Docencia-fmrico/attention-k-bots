#include "attention/node_graph.hpp"

int main(int argc, char** argv) {
   rclcpp::init(argc, argv);
   auto node = std::make_shared<NodeGraph>();

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin_some(node->get_node_base_interface());
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  rclcpp::spin(node->get_node_base_interface());
}