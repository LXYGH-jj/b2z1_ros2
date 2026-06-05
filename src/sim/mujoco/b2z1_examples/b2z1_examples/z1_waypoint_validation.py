import rclpy
from rclpy.node import Node

from b2z1_msgs.msg import MujocoDemoCommand
from b2z1_z1_trajectories import mujoco_safe_sequence


class Z1WaypointValidationNode(Node):
    def __init__(self) -> None:
        super().__init__("z1_waypoint_validation")
        self.publisher_ = self.create_publisher(MujocoDemoCommand, "mujoco_demo_command", 10)
        self.timer_ = self.create_timer(0.1, self.step_sequence)
        self.start_time_ = self.get_clock().now()
        self.sequence_ = mujoco_safe_sequence()
        self.stage_ = 0

    def publish_command(self, z1_q: list[float], duration: float, log_text: str) -> None:
        msg = MujocoDemoCommand()
        msg.b2_mode = "ground_support"
        msg.vx = 0.0
        msg.vy = 0.0
        msg.vyaw = 0.0
        msg.z1_q = z1_q
        msg.duration = duration
        self.publisher_.publish(msg)
        self.get_logger().info(log_text)

    def step_sequence(self) -> None:
        elapsed = (self.get_clock().now() - self.start_time_).nanoseconds / 1e9

        if self.stage_ < len(self.sequence_) and elapsed >= self.sequence_[self.stage_].trigger_sec:
            stage = self.sequence_[self.stage_]
            self.publish_command(
                list(stage.q),
                stage.duration_sec,
                stage.description,
            )
            self.stage_ += 1
        elif self.stage_ == len(self.sequence_) and elapsed >= 14.4:
            self.get_logger().info("Z1 waypoint validation sequence complete.")
            self.timer_.cancel()


def main() -> None:
    rclpy.init()
    node = Z1WaypointValidationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
