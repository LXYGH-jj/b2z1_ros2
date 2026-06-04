import rclpy
from rclpy.node import Node

from b2z1_msgs.msg import CombinedCommand


class B2OnlyStandThenDownNode(Node):
    def __init__(self) -> None:
        super().__init__("b2_only_stand_then_down")
        self.publisher_ = self.create_publisher(CombinedCommand, "combined_command", 10)
        self.timer_ = self.create_timer(0.1, self.step_sequence)
        self.start_time_ = self.get_clock().now()
        self.stage_ = 0

    def publish_command(self, b2_mode: str, duration: float, log_text: str) -> None:
        msg = CombinedCommand()
        msg.b2_mode = b2_mode
        msg.vx = 0.0
        msg.vy = 0.0
        msg.vyaw = 0.0
        msg.z1_q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg.duration = duration
        self.publisher_.publish(msg)
        self.get_logger().info(log_text)

    def step_sequence(self) -> None:
        elapsed = (self.get_clock().now() - self.start_time_).nanoseconds / 1e9

        if self.stage_ == 0 and elapsed >= 1.0:
            self.publish_command(
                "balance_stand",
                8.0,
                "Stage 1: command B2 to stand up.",
            )
            self.stage_ = 1
        elif self.stage_ == 1 and elapsed >= 10.5:
            self.publish_command(
                "stand_down",
                3.0,
                "Stage 2: command B2 to stand down smoothly.",
            )
            self.stage_ = 2
        elif self.stage_ == 2 and elapsed >= 14.5:
            self.get_logger().info("Sequence complete.")
            self.timer_.cancel()


def main() -> None:
    rclpy.init()
    node = B2OnlyStandThenDownNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
