#pragma once

#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>

#include "unitree_api/msg/request.hpp"

namespace b2z1_b2_bridge {

constexpr int32_t ROBOT_SPORT_API_ID_DAMP = 1001;
constexpr int32_t ROBOT_SPORT_API_ID_BALANCESTAND = 1002;
constexpr int32_t ROBOT_SPORT_API_ID_STOPMOVE = 1003;
constexpr int32_t ROBOT_SPORT_API_ID_STANDDOWN = 1005;
constexpr int32_t ROBOT_SPORT_API_ID_RECOVERYSTAND = 1006;
constexpr int32_t ROBOT_SPORT_API_ID_MOVE = 1008;

class B2SportClient {
public:
  explicit B2SportClient(rclcpp::Node* node);

  void Damp(unitree_api::msg::Request& req);
  void BalanceStand(unitree_api::msg::Request& req);
  void StopMove(unitree_api::msg::Request& req);
  void StandDown(unitree_api::msg::Request& req);
  void RecoveryStand(unitree_api::msg::Request& req);
  void Move(unitree_api::msg::Request& req, float vx, float vy, float vyaw);

private:
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber_;
  rclcpp::Node* node_;
};

}  // namespace b2z1_b2_bridge
