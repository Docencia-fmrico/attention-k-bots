#include "node_graph/node_graph.hpp"

#include <math.h>
#include <time.h>

#include <chrono>
#include <thread>

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
#include "ros2_knowledge_graph_msgs/msg/graph_update.hpp"
#include "ros2_knowledge_graph_msgs/msg/node.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

NodeGraph::NodeGraph() : rclcpp_lifecycle::LifecycleNode("node_graph") {
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

CallbackReturnT NodeGraph::on_configure(const rclcpp_lifecycle::State& state) {
  joint_cmd_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/head_controller/joint_trajectory", 100);

  graph_ = std::make_shared<ros2_knowledge_graph::GraphNode>(shared_from_this());

  RCLCPP_INFO(get_logger(), "[%s] Configuring from [%s] state...", get_name(),
              state.label().c_str());

  return CallbackReturnT::SUCCESS;
}
CallbackReturnT NodeGraph::on_activate(const rclcpp_lifecycle::State& state) {
  joint_cmd_pub_->on_activate();
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

std::vector<std::string> NodeGraph::find_objects(std::string object_to_find) {
  std::vector<std::string> all_names = graph_->get_node_names();
  std::vector<std::string> names;
  for (std::string name : all_names) {
    if (name.find(object_to_find) != std::string::npos) {
      names.push_back(name);
    }
  }

  return names;
}

void NodeGraph::select_object() {
  objects_names_.clear();
  objects_angle_.clear();
  std::vector<std::string> objects_found = find_objects("cabinet");
  std::sort(objects_found.begin(), objects_found.end());

  for (std::string fromFrameRel : objects_found) {


    // std::string fromFrameRel = "cabinet";
    std::string toFrameRel = "head_1_link";
    geometry_msgs::msg::TransformStamped transformStamped;

    try {
      transformStamped = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);
      // transformStamped = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel,
      // this->get_clock()->now(), rclcpp::Duration(1,0));
    } catch (tf2::TransformException& ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", toFrameRel.c_str(),
                  fromFrameRel.c_str(), ex.what());
      continue;
    }

    float distance =
        sqrt(transformStamped.transform.translation.y * transformStamped.transform.translation.y +
             transformStamped.transform.translation.x * transformStamped.transform.translation.x);

    if (distance <= MAX_DISTANCE) {
      float pan =
          atan2(transformStamped.transform.translation.y, transformStamped.transform.translation.x);
      float tilt =
          atan2(transformStamped.transform.translation.z, transformStamped.transform.translation.x);

      if (-M_PI_2 < pan && pan < M_PI_2) {
        objects_names_.push_back(fromFrameRel);
        std::vector<float> translation;
        translation.push_back(pan);
        translation.push_back(tilt);
        objects_angle_.push_back(translation);
      }
    }
  }
}

void NodeGraph::look_for_object() {

  if (now().seconds() - prev_exploration_ > 8){
    prev_exploration_ = now().seconds();
    trajectory_msgs::msg::JointTrajectory command_msg;
    trajectory_msgs::msg::JointTrajectoryPoint point;

    command_msg.header.stamp = now();
    command_msg.joint_names.push_back("head_1_joint");
    command_msg.joint_names.push_back("head_2_joint");

    point.positions.resize(2);
    point.velocities.resize(2);
    point.accelerations.resize(2);
    point.effort.resize(2);
    point.velocities[0] = 0.15;
    point.velocities[1] = 0.15;
    point.accelerations[0] = 0.1;
    point.accelerations[1] = 0.1;
    point.effort[0] = 0.1;
    point.effort[1] = 0.1;

    point.time_from_start = rclcpp::Duration(2s);

    // Create trajectory
    point.positions[0] = -M_PI_2;
    point.positions[1] = -M_PI_2 / 3;
    command_msg.points.push_back(point);
    // joint_cmd_pub_->publish(command_msg);

    point.time_from_start = rclcpp::Duration(4s);
    point.positions[0] = -M_PI_2;
    point.positions[1] = 0.0;
    command_msg.points.push_back(point);

    point.time_from_start = rclcpp::Duration(6s);
    point.positions[0] = M_PI_2;
    point.positions[1] = 0.0;
    command_msg.points.push_back(point);

    point.time_from_start = rclcpp::Duration(8s);
    point.positions[0] = M_PI_2;
    point.positions[1] = -M_PI_2 / 3;
    command_msg.points.push_back(point);

    joint_cmd_pub_->publish(command_msg);
  }
  //std::this_thread::sleep_for(std::chrono::milliseconds(8000));
}

void NodeGraph::watch_object() {}

void NodeGraph::do_work() {
  select_object();

  std::cout << "Size: " << objects_names_.size() << std::endl;
  if (objects_names_.empty()) {
    look_for_object();
  } else {
    if (objects_names_.size() != size_points_){
      size_points_ = objects_names_.size();
      object_to_see = 0;
      prev_look_to_ = 0;
    } else {
      
      trajectory_msgs::msg::JointTrajectory command_msg;
      trajectory_msgs::msg::JointTrajectoryPoint point;

      command_msg.header.stamp = now();
      command_msg.joint_names.push_back("head_1_joint");
      command_msg.joint_names.push_back("head_2_joint");

      point.positions.resize(2);
      point.velocities.resize(2);
      point.accelerations.resize(2);
      point.effort.resize(2);
      point.positions[0] = objects_angle_[object_to_see].at(0);
      point.positions[1] = objects_angle_[object_to_see].at(1);
      point.velocities[0] = 0.1;
      point.velocities[1] = 0.1;
      point.accelerations[0] = 0.1;
      point.accelerations[1] = 0.1;
      point.effort[0] = 0.1;
      point.effort[1] = 0.1;

      point.time_from_start = rclcpp::Duration(1s);
      command_msg.points.push_back(point);

      joint_cmd_pub_->publish(command_msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(3000));
      if (now().seconds() - prev_look_to_ > 2){
        prev_look_to_ = now().seconds();
        object_to_see++;
        if (object_to_see >= size_points_) object_to_see = 0;
      }
    }
  }

  //rclcpp::Time start_watching_time = this->get_clock()->now();
  // while((this->get_clock()->now() - start_watching_time) < 1.0) {  }

  // std::cout << "X: " << transformStamped.transform.translation.x
  // << " Y: " << transformStamped.transform.translation.y
  // << " Z: " << transformStamped.transform.translation.z << std::endl;

  // std::cout << "PAN: " << pan * 180 / M_PI << " TILT: " << tilt * 180 / M_PI << std::endl;
}
