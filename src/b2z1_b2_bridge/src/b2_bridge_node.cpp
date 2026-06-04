#include "b2z1_b2_bridge/b2_bridge_node.hpp"

#include "unitree_api/msg/request.hpp"

namespace b2z1_b2_bridge {

B2BridgeNode::B2BridgeNode()
: Node("b2_bridge_node"),
  sport_client_(this)
{
  command_sub_ = create_subscription<b2z1_msgs::msg::B2Command>(
    "b2_command",
    10,
    std::bind(&B2BridgeNode::on_command, this, std::placeholders::_1));
}

void B2BridgeNode::on_command(const b2z1_msgs::msg::B2Command::SharedPtr msg)
{
  unitree_api::msg::Request req;
  const auto& mode = msg->mode;

  if (mode == "damp") {
    sport_client_.Damp(req);
  } else if (mode == "balance_stand") {
    sport_client_.BalanceStand(req);
  } else if (mode == "stop_move") {
    sport_client_.StopMove(req);
  } else if (mode == "stand_down") {
    sport_client_.StandDown(req);
  } else if (mode == "recovery_stand") {
    sport_client_.RecoveryStand(req);
  } else if (mode == "move") {
    sport_client_.Move(req, msg->vx, msg->vy, msg->vyaw);
  } else {
    RCLCPP_WARN(get_logger(), "Unsupported B2 mode: '%s'", mode.c_str());
  }
}

}  // namespace b2z1_b2_bridge

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<b2z1_b2_bridge::B2BridgeNode>());
  rclcpp::shutdown();
  return 0;
}
