#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "b2z1_b2_bridge/ros2_b2_sport_client.hpp"
#include "b2z1_msgs/msg/b2_command.hpp"

namespace b2z1_b2_bridge {

class B2BridgeNode : public rclcpp::Node {
public:
  B2BridgeNode();

private:
  void on_command(const b2z1_msgs::msg::B2Command::SharedPtr msg);

  B2SportClient sport_client_;
  rclcpp::Subscription<b2z1_msgs::msg::B2Command>::SharedPtr command_sub_;
};

}  // namespace b2z1_b2_bridge
