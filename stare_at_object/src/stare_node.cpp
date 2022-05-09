#include "stare_at_object/stare_at_object.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StareNode>();

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin_some(node->get_node_base_interface());
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  rclcpp::Rate rate(10);
  while (rclcpp::ok()) {
    node->do_work();
    //node->look_for_object();

    rclcpp::spin_some(node->get_node_base_interface());
    rate.sleep();
  }

}