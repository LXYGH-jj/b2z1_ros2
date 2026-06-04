#include "b2z1_coordinator/coordinator_node.hpp"

#include <string>
#include <vector>

#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace b2z1_coordinator {

namespace {
std::vector<std::string> z1_joint_names()
{
  return {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
}
}  // namespace

CoordinatorNode::CoordinatorNode()
: Node("b2z1_coordinator")
{
  b2_pub_ = create_publisher<b2z1_msgs::msg::B2Command>("b2_command", 10);
  z1_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/joint_trajectory_controller/joint_trajectory", 10);

  combined_sub_ = create_subscription<b2z1_msgs::msg::CombinedCommand>(
    "combined_command",
    10,
    std::bind(&CoordinatorNode::on_command, this, std::placeholders::_1));
}

void CoordinatorNode::on_command(const b2z1_msgs::msg::CombinedCommand::SharedPtr msg)
{
  b2z1_msgs::msg::B2Command b2_msg;
  b2_msg.mode = msg->b2_mode;
  b2_msg.vx = msg->vx;
  b2_msg.vy = msg->vy;
  b2_msg.vyaw = msg->vyaw;
  b2_pub_->publish(b2_msg);

  trajectory_msgs::msg::JointTrajectory traj;
  traj.joint_names = z1_joint_names();

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions.assign(msg->z1_q.begin(), msg->z1_q.end());

  const double duration = msg->duration > 1.0e-3f ? static_cast<double>(msg->duration) : 2.0;
  point.time_from_start = rclcpp::Duration::from_seconds(duration);
  traj.points.push_back(point);

  z1_pub_->publish(traj);
}

}  // namespace b2z1_coordinator

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<b2z1_coordinator::CoordinatorNode>());
  rclcpp::shutdown();
  return 0;
}
