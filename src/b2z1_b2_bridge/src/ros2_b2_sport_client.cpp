#include "b2z1_b2_bridge/ros2_b2_sport_client.hpp"

namespace b2z1_b2_bridge {

B2SportClient::B2SportClient(rclcpp::Node* node) : node_(node) {
  req_puber_ = node_->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
}

void B2SportClient::Damp(unitree_api::msg::Request& req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_DAMP;
  req_puber_->publish(req);
}

void B2SportClient::BalanceStand(unitree_api::msg::Request& req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_BALANCESTAND;
  req_puber_->publish(req);
}

void B2SportClient::StopMove(unitree_api::msg::Request& req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_STOPMOVE;
  req_puber_->publish(req);
}

void B2SportClient::StandDown(unitree_api::msg::Request& req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_STANDDOWN;
  req_puber_->publish(req);
}

void B2SportClient::RecoveryStand(unitree_api::msg::Request& req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_RECOVERYSTAND;
  req_puber_->publish(req);
}

void B2SportClient::Move(unitree_api::msg::Request& req, float vx, float vy, float vyaw) {
  nlohmann::json js;
  js["x"] = vx;
  js["y"] = vy;
  js["z"] = vyaw;
  req.parameter = js.dump();
  req.header.identity.api_id = ROBOT_SPORT_API_ID_MOVE;
  req_puber_->publish(req);
}

}  // namespace b2z1_b2_bridge
