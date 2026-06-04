import rclpy
from rclpy.node import Node

from b2z1_msgs.msg import CombinedCommand


class B2OnlyStandNode(Node):
    def __init__(self) -> None:
        super().__init__("b2_only_stand")
        self.publisher_ = self.create_publisher(CombinedCommand, "combined_command", 10)
        self.timer_ = self.create_timer(1.0, self.publish_once)
        self.sent_ = False

    def publish_once(self) -> None:
        if self.sent_:
            return

        msg = CombinedCommand()
        msg.b2_mode = "balance_stand"
        msg.vx = 0.0
        msg.vy = 0.0
        msg.vyaw = 0.0
        msg.z1_q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg.duration = 8.0
        self.publisher_.publish(msg)
        self.get_logger().info("Published b2_only_stand command.")
        self.sent_ = True


def main() -> None:
    rclpy.init()
    node = B2OnlyStandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
