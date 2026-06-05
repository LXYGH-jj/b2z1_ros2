import rclpy
from rclpy.node import Node

from b2z1_msgs.msg import MujocoDemoCommand


class B2StandValidationNode(Node):
    def __init__(self) -> None:
        super().__init__("b2_stand_validation")
        self.publisher_ = self.create_publisher(MujocoDemoCommand, "mujoco_demo_command", 10)
        self.timer_ = self.create_timer(1.0, self.publish_validation_command)
        self.published_ = False

    def publish_validation_command(self) -> None:
        if self.published_:
            return

        msg = MujocoDemoCommand()
        msg.b2_mode = "balance_stand"
        msg.vx = 0.0
        msg.vy = 0.0
        msg.vyaw = 0.0
        msg.z1_q = [0.0] * 6
        msg.duration = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info("Published B2 stand validation command.")
        self.published_ = True
        self.timer_.cancel()


def main() -> None:
    rclpy.init()
    node = B2StandValidationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
