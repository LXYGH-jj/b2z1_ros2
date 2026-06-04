#pragma once

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "b2z1_msgs/msg/b2_command.hpp"
#include "b2z1_msgs/msg/combined_command.hpp"

namespace b2z1_coordinator {

class CoordinatorNode : public rclcpp::Node {
public:
  CoordinatorNode();

private:
  void on_command(const b2z1_msgs::msg::CombinedCommand::SharedPtr msg);

  rclcpp::Subscription<b2z1_msgs::msg::CombinedCommand>::SharedPtr combined_sub_;
  rclcpp::Publisher<b2z1_msgs::msg::B2Command>::SharedPtr b2_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr z1_pub_;
};

}  // namespace b2z1_coordinator
