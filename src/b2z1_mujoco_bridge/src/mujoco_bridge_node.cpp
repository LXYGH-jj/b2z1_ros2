#include "b2z1_mujoco_bridge/mujoco_bridge_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iterator>
#include <stdexcept>
#include <string>

#include <unitree/robot/channel/channel_factory.hpp>

namespace b2z1_mujoco_bridge {
namespace {
constexpr double kPosStopF = 2.146e9;
constexpr double kVelStopF = 16000.0;
constexpr int kTotalMotors = 20;
constexpr int kB2Motors = 12;
constexpr int kZ1Motors = 6;
constexpr int kZ1StartIndex = 12;

template <std::size_t N>
std::array<double, N> load_array_param(
  rclcpp::Node& node,
  const std::string& name,
  const std::array<double, N>& default_value)
{
  const auto vec = node.declare_parameter<std::vector<double>>(
    name, std::vector<double>(default_value.begin(), default_value.end()));
  if (vec.size() != N) {
    throw std::runtime_error(
      "Parameter '" + name + "' must have length " + std::to_string(N) +
      ", got " + std::to_string(vec.size()));
  }

  std::array<double, N> out{};
  std::copy(vec.begin(), vec.end(), out.begin());
  return out;
}

double sum_array(const std::array<double, 5>& values)
{
  double total = 0.0;
  for (double v : values) {
    total += v;
  }
  return total;
}
}  // namespace

MujocoBridgeNode::MujocoBridgeNode()
: Node("b2z1_mujoco_bridge_node"),
  have_lowstate_(false),
  b2_stand_target_1_q_(load_array_param<12>(
    *this, "b2_stand_target_1_q",
    {0.0, 1.36, -2.65, 0.0, 1.36, -2.65, 0.2, 1.36, -2.65, 0.2, 1.36, -2.65})),
  b2_stand_target_2_q_(load_array_param<12>(
    *this, "b2_stand_target_2_q",
    {0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3})),
  b2_stand_target_3_q_(load_array_param<12>(
    *this, "b2_stand_target_3_q",
    {0.0, 1.36, -2.65, 0.0, 1.36, -2.65, 0.0, 1.36, -2.65, 0.0, 1.36, -2.65})),
  b2_stand_target_4_q_(load_array_param<12>(
    *this, "b2_stand_target_4_q",
    {-0.5, 1.36, -2.65, 0.5, 1.36, -2.65, -0.5, 1.36, -2.65, 0.5, 1.36, -2.65})),
  b2_stand_phase_durations_(load_array_param<5>(
    *this, "b2_stand_phase_durations",
    {1.0, 1.8, 2.0, 2.2, 1.0})),
  b2_down_q_(load_array_param<12>(
    *this, "b2_down_q",
    {0.0, 1.28, -2.80, 0.0, 1.28, -2.80, 0.0, 1.28, -2.80, 0.0, 1.28, -2.80})),
  b2_kp_by_joint_(load_array_param<12>(
    *this, "b2_kp_by_joint",
    {260.0, 260.0, 260.0, 260.0, 260.0, 260.0, 430.0, 430.0, 430.0, 430.0, 430.0, 430.0})),
  b2_kd_by_joint_(load_array_param<12>(
    *this, "b2_kd_by_joint",
    {7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 8.0, 8.0, 8.0, 8.0, 8.0, 8.0})),
  b2_from_q_{},
  b2_to_q_(b2_down_q_),
  z1_target_q_(load_array_param<6>(
    *this, "z1_home_q",
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0})),
  z1_from_q_{},
  z1_to_q_(z1_target_q_),
  z1_kp_by_joint_(load_array_param<6>(
    *this, "z1_kp_by_joint",
    {100.0, 100.0, 100.0, 100.0, 10.0, 10.0})),
  z1_kd_by_joint_(load_array_param<6>(
    *this, "z1_kd_by_joint",
    {10.0, 10.0, 10.0, 5.0, 1.0, 1.0})),
  current_b2_mode_("damp"),
  network_interface_(declare_parameter<std::string>("network_interface", "lo")),
  ramp_duration_sec_(declare_parameter<double>("ramp_duration_sec", 3.5)),
  ground_support_kp_scale_(declare_parameter<double>("ground_support_kp_scale", 0.28)),
  ground_support_kd_scale_(declare_parameter<double>("ground_support_kd_scale", 0.45)),
  debug_z1_state_(declare_parameter<bool>("debug_z1_state", true)),
  transition_start_time_(this->now()),
  gain_ramp_start_time_(this->now()),
  last_debug_log_time_(this->now()),
  transition_duration_sec_(sum_array(b2_stand_phase_durations_)),
  have_active_command_(false),
  warned_move_mode_(false),
  use_official_stand_sequence_(false)
{
  unitree::robot::ChannelFactory::Instance()->Init(0, network_interface_.c_str());

  lowcmd_publisher_.reset(
    new unitree::robot::ChannelPublisher<unitree_go::msg::dds_::LowCmd_>("rt/lowcmd"));
  lowcmd_publisher_->InitChannel();
  lowstate_sub_ = std::make_shared<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>>("rt/lowstate");
  lowstate_sub_->InitChannel([this](const void* msg) {
    std::lock_guard<std::mutex> lock(lowstate_mutex_);
    latest_lowstate_ = *static_cast<const unitree_go::msg::dds_::LowState_*>(msg);
    have_lowstate_ = true;
  });

  init_lowcmd_message();

  combined_sub_ = create_subscription<b2z1_msgs::msg::CombinedCommand>(
    "combined_command",
    10,
    std::bind(&MujocoBridgeNode::on_command, this, std::placeholders::_1));

  publish_timer_ = create_wall_timer(
    std::chrono::milliseconds(2),
    std::bind(&MujocoBridgeNode::publish_lowcmd, this));

  RCLCPP_INFO(
    get_logger(),
    "MuJoCo bridge publishing lowcmd on '%s' through interface '%s'.",
    "rt/lowcmd",
    network_interface_.c_str());
  RCLCPP_INFO(
    get_logger(),
    "Using fixed per-joint PD gains. front kp=260/kd=7, rear kp=430/kd=8, ramp=%.1fs.",
    ramp_duration_sec_);
  if (debug_z1_state_) {
    RCLCPP_INFO(get_logger(), "Z1 debug logging is enabled (q/dq/tau_est around target updates).");
  }
}

void MujocoBridgeNode::init_lowcmd_message()
{
  low_cmd_.head()[0] = 0xFE;
  low_cmd_.head()[1] = 0xEF;
  low_cmd_.level_flag() = 0xFF;
  low_cmd_.gpio() = 0;

  for (int i = 0; i < kTotalMotors; ++i) {
    set_motor_stop(i, i < kB2Motors ? 0x0A : 0x01);
  }
}

void MujocoBridgeNode::set_motor_hold(int index, double q, double kp, double kd, uint8_t mode)
{
  auto& m = low_cmd_.motor_cmd()[index];
  m.mode() = mode;
  m.q() = static_cast<float>(q);
  m.kp() = static_cast<float>(kp);
  m.dq() = 0.0f;
  m.kd() = static_cast<float>(kd);
  m.tau() = 0.0f;
}

void MujocoBridgeNode::set_motor_stop(int index, uint8_t mode)
{
  auto& m = low_cmd_.motor_cmd()[index];
  m.mode() = mode;
  m.q() = static_cast<float>(kPosStopF);
  m.kp() = 0.0f;
  m.dq() = static_cast<float>(kVelStopF);
  m.kd() = 0.0f;
  m.tau() = 0.0f;
}

void MujocoBridgeNode::on_command(const b2z1_msgs::msg::CombinedCommand::SharedPtr msg)
{
  std::array<double, 12> current_b2_q{};
  std::array<double, 6> current_z1_q{};
  double current_gain_scale = 0.0;
  sample_current_targets(current_b2_q, current_z1_q, current_gain_scale);

  const bool prev_b2_active = have_active_command_ && current_b2_mode_ != "damp";
  current_b2_mode_ = msg->b2_mode.empty() ? std::string("balance_stand") : msg->b2_mode;
  const bool next_b2_active = current_b2_mode_ != "damp";
  use_official_stand_sequence_ =
    !prev_b2_active &&
    next_b2_active &&
    (current_b2_mode_ == "balance_stand" || current_b2_mode_ == "move");

  b2_from_q_ = current_b2_q;
  if (current_b2_mode_ == "stand_down") {
    b2_to_q_ = b2_down_q_;
  } else {
    b2_to_q_ = b2_stand_target_4_q_;
  }
  if (use_official_stand_sequence_) {
    b2_from_q_ = b2_down_q_;
  }

  z1_from_q_ = current_z1_q;
  std::copy(msg->z1_q.begin(), msg->z1_q.end(), z1_to_q_.begin());
  std::copy(z1_to_q_.begin(), z1_to_q_.end(), z1_target_q_.begin());

  transition_start_time_ = this->now();
  if (use_official_stand_sequence_) {
    transition_duration_sec_ = sum_array(b2_stand_phase_durations_);
  } else {
    transition_duration_sec_ = std::max(0.25, static_cast<double>(msg->duration));
  }
  if (!prev_b2_active && next_b2_active) {
    gain_ramp_start_time_ = this->now();
  } else if (prev_b2_active && next_b2_active) {
    gain_ramp_start_time_ = this->now() - rclcpp::Duration::from_seconds(ramp_duration_sec_);
  } else if (!prev_b2_active) {
    gain_ramp_start_time_ = this->now();
  }
  have_active_command_ = true;

  if (current_b2_mode_ == "move" && !warned_move_mode_) {
    RCLCPP_WARN(
      get_logger(),
      "MuJoCo bridge is in simple PD mode. 'move' is treated as a standing hold; vx/vy/vyaw are ignored.");
    warned_move_mode_ = true;
  }

  RCLCPP_INFO(
    get_logger(),
    "Updated combined target: b2_mode=%s, z1_q=[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
    current_b2_mode_.c_str(),
    z1_target_q_[0], z1_target_q_[1], z1_target_q_[2],
    z1_target_q_[3], z1_target_q_[4], z1_target_q_[5]);
  log_z1_debug_snapshot("command_update");
}

void MujocoBridgeNode::sample_current_targets(
  std::array<double, 12>& b2_q,
  std::array<double, 6>& z1_q,
  double& gain_scale) const
{
  const double transition_elapsed = std::max(0.0, (this->now() - transition_start_time_).seconds());
  const double transition_duration = std::max(1.0e-3, transition_duration_sec_);
  const double alpha = std::clamp(transition_elapsed / transition_duration, 0.0, 1.0);

  if (use_official_stand_sequence_) {
    const double t1 = b2_stand_phase_durations_[0];
    const double t2 = t1 + b2_stand_phase_durations_[1];
    const double t3 = t2 + b2_stand_phase_durations_[2];
    const double t4 = t3 + b2_stand_phase_durations_[3];
    for (int i = 0; i < kB2Motors; ++i) {
      if (transition_elapsed < t1) {
        const double local_alpha = transition_elapsed / std::max(1.0e-3, b2_stand_phase_durations_[0]);
        b2_q[i] = b2_from_q_[i] + (b2_stand_target_1_q_[i] - b2_from_q_[i]) * local_alpha;
      } else if (transition_elapsed < t2) {
        const double local_alpha =
          (transition_elapsed - t1) / std::max(1.0e-3, b2_stand_phase_durations_[1]);
        b2_q[i] =
          b2_stand_target_1_q_[i] +
          (b2_stand_target_2_q_[i] - b2_stand_target_1_q_[i]) * local_alpha;
      } else if (transition_elapsed < t3) {
        b2_q[i] = b2_stand_target_2_q_[i];
      } else if (transition_elapsed < t4) {
        const double local_alpha =
          (transition_elapsed - t3) / std::max(1.0e-3, b2_stand_phase_durations_[3]);
        b2_q[i] =
          b2_stand_target_2_q_[i] +
          (b2_stand_target_3_q_[i] - b2_stand_target_2_q_[i]) * local_alpha;
      } else {
        const double local_alpha =
          (transition_elapsed - t4) / std::max(1.0e-3, b2_stand_phase_durations_[4]);
        b2_q[i] =
          b2_stand_target_3_q_[i] +
          (b2_stand_target_4_q_[i] - b2_stand_target_3_q_[i]) * std::clamp(local_alpha, 0.0, 1.0);
      }
    }
  } else {
    for (int i = 0; i < kB2Motors; ++i) {
      b2_q[i] = b2_from_q_[i] + (b2_to_q_[i] - b2_from_q_[i]) * alpha;
    }
  }
  for (int i = 0; i < kZ1Motors; ++i) {
    z1_q[i] = z1_from_q_[i] + (z1_to_q_[i] - z1_from_q_[i]) * alpha;
  }

  const double gain_elapsed = std::max(0.0, (this->now() - gain_ramp_start_time_).seconds());
  const double gain_duration = std::max(1.0e-3, ramp_duration_sec_);
  gain_scale = std::clamp(gain_elapsed / gain_duration, 0.0, 1.0);
}

void MujocoBridgeNode::publish_lowcmd()
{
  const bool b2_damp_mode = current_b2_mode_ == "damp";
  const bool b2_ground_support_mode = current_b2_mode_ == "ground_support";
  const bool have_target = have_active_command_;
  constexpr uint8_t kB2Mode = 0x0A;
  constexpr uint8_t kZ1Mode = 0x01;

  std::array<double, 12> current_b2_q{};
  std::array<double, 6> current_z1_q{};
  double gain_scale = 0.0;
  sample_current_targets(current_b2_q, current_z1_q, gain_scale);
  double b2_gain_scale = gain_scale;
  if (use_official_stand_sequence_ && current_b2_mode_ == "balance_stand") {
    const double stand_elapsed = std::max(0.0, (this->now() - transition_start_time_).seconds());
    const double first_phase = std::max(1.0e-3, b2_stand_phase_durations_[0]);
    b2_gain_scale = std::clamp(stand_elapsed / first_phase, 0.0, 1.0);
  }

  for (int i = 0; i < kB2Motors; ++i) {
    if ((b2_damp_mode && !b2_ground_support_mode) || !have_target) {
      set_motor_stop(i, kB2Mode);
    } else {
      const double kp_scale = b2_ground_support_mode ? ground_support_kp_scale_ : 1.0;
      const double kd_scale = b2_ground_support_mode ? ground_support_kd_scale_ : 1.0;
      set_motor_hold(
        i,
        b2_ground_support_mode ? b2_down_q_[i] : current_b2_q[i],
        b2_kp_by_joint_[i] * kp_scale * b2_gain_scale,
        b2_kd_by_joint_[i] * kd_scale * b2_gain_scale,
        kB2Mode);
    }
  }

  for (int i = 0; i < kZ1Motors; ++i) {
    if (!have_target) {
      set_motor_stop(kZ1StartIndex + i, kZ1Mode);
    } else {
      set_motor_hold(
        kZ1StartIndex + i,
        current_z1_q[i],
        z1_kp_by_joint_[i] * gain_scale,
        z1_kd_by_joint_[i] * gain_scale,
        kZ1Mode);
    }
  }

  for (int i = kZ1StartIndex + kZ1Motors; i < kTotalMotors; ++i) {
    set_motor_stop(i, kZ1Mode);
  }

  low_cmd_.crc() = crc32_core(
    reinterpret_cast<uint32_t*>(&low_cmd_),
    (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
  lowcmd_publisher_->Write(low_cmd_);

  if (debug_z1_state_) {
    const auto now = this->now();
    if ((now - last_debug_log_time_).seconds() >= 0.2) {
      last_debug_log_time_ = now;
      log_z1_debug_snapshot("stream");
    }
  }
}

void MujocoBridgeNode::log_z1_debug_snapshot(const char* reason)
{
  if (!debug_z1_state_ || !lowstate_sub_) {
    return;
  }

  if (!have_lowstate_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Z1 debug snapshot skipped: no rt/lowstate received yet.");
    return;
  }

  std::lock_guard<std::mutex> lock(lowstate_mutex_);
  const auto& motors = latest_lowstate_.motor_state();
  if (motors.size() < static_cast<std::size_t>(kZ1StartIndex + kZ1Motors)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Z1 debug snapshot skipped: lowstate motor_state too short (%zu).",
      motors.size());
    return;
  }

  RCLCPP_INFO(
    get_logger(),
    "[%s] Z1 target q=[%.2f %.2f %.2f %.2f %.2f %.2f] | actual q=[%.2f %.2f %.2f %.2f %.2f %.2f] | dq=[%.2f %.2f %.2f %.2f %.2f %.2f] | tau=[%.2f %.2f %.2f %.2f %.2f %.2f]",
    reason,
    z1_target_q_[0], z1_target_q_[1], z1_target_q_[2], z1_target_q_[3], z1_target_q_[4], z1_target_q_[5],
    motors[kZ1StartIndex + 0].q(), motors[kZ1StartIndex + 1].q(), motors[kZ1StartIndex + 2].q(),
    motors[kZ1StartIndex + 3].q(), motors[kZ1StartIndex + 4].q(), motors[kZ1StartIndex + 5].q(),
    motors[kZ1StartIndex + 0].dq(), motors[kZ1StartIndex + 1].dq(), motors[kZ1StartIndex + 2].dq(),
    motors[kZ1StartIndex + 3].dq(), motors[kZ1StartIndex + 4].dq(), motors[kZ1StartIndex + 5].dq(),
    motors[kZ1StartIndex + 0].tau_est(), motors[kZ1StartIndex + 1].tau_est(), motors[kZ1StartIndex + 2].tau_est(),
    motors[kZ1StartIndex + 3].tau_est(), motors[kZ1StartIndex + 4].tau_est(), motors[kZ1StartIndex + 5].tau_est());
}

uint32_t MujocoBridgeNode::crc32_core(uint32_t* ptr, uint32_t len)
{
  unsigned int xbit = 0;
  unsigned int data = 0;
  unsigned int crc32 = 0xFFFFFFFF;
  const unsigned int polynomial = 0x04c11db7;

  for (unsigned int i = 0; i < len; ++i) {
    xbit = 1U << 31;
    data = ptr[i];
    for (unsigned int bits = 0; bits < 32; ++bits) {
      if (crc32 & 0x80000000U) {
        crc32 <<= 1;
        crc32 ^= polynomial;
      } else {
        crc32 <<= 1;
      }
      if (data & xbit) {
        crc32 ^= polynomial;
      }
      xbit >>= 1;
    }
  }

  return crc32;
}

}  // namespace b2z1_mujoco_bridge

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<b2z1_mujoco_bridge::MujocoBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
