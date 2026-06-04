import math

import rclpy
from rclpy.node import Node

from b2z1_msgs.msg import CombinedCommand


class Z1OnlyReachNode(Node):
    def __init__(self) -> None:
        super().__init__("z1_only_reach")
        self.publisher_ = self.create_publisher(CombinedCommand, "combined_command", 10)
        self.timer_ = self.create_timer(0.1, self.step_sequence)
        self.start_time_ = self.get_clock().now()
        self.stage_ = 0

    def publish_command(self, z1_q: list[float], duration: float, log_text: str) -> None:
        msg = CombinedCommand()
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

        if self.stage_ == 0 and elapsed >= 1.0:
            self.publish_command(
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                2.0,
                "Stage 1: command Z1 to home pose.",
            )
            self.stage_ = 1
        elif self.stage_ == 1 and elapsed >= 3.2:
            self.publish_command(
                [0.0, 1.0, -1.0, 0.0, 0.0, 0.0],
                2.0,
                "Stage 2: command Z1 to reach pose.",
            )
            self.stage_ = 2
        elif self.stage_ == 2 and elapsed >= 5.4:
            self.publish_command(
                [math.pi / 4.0, 1.0, -1.0, 0.0, 0.0, 0.0],
                2.0,
                "Stage 3: command Z1 to waypoint_test wrist pose.",
            )
            self.stage_ = 3
        elif self.stage_ == 3 and elapsed >= 7.6:
            self.publish_command(
                [0.0, 0.45, -0.45, 0.0, 0.0, 0.0],
                2.5,
                "Stage 4: command Z1 to a gentle retract pose.",
            )
            self.stage_ = 4
        elif self.stage_ == 4 and elapsed >= 10.4:
            self.publish_command(
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                3.5,
                "Stage 5: command Z1 back to home.",
            )
            self.stage_ = 5
        elif self.stage_ == 5 and elapsed >= 14.4:
            self.get_logger().info("Sequence complete.")
            self.timer_.cancel()


def main() -> None:
    rclpy.init()
    node = Z1OnlyReachNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
