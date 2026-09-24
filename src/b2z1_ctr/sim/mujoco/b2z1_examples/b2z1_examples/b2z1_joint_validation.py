import math
from dataclasses import dataclass

import rclpy
from rclpy.node import Node

from b2z1_msgs.msg import MujocoDemoCommand


@dataclass(frozen=True)
class JointStage:
    trigger_sec: float
    b2_mode: str
    z1_q: tuple[float, float, float, float, float, float]
    duration_sec: float
    description: str


def joint_validation_sequence() -> tuple[JointStage, ...]:
    """Small, deterministic sequence that makes both robots move together."""
    return (
        JointStage(
            trigger_sec=1.0,
            b2_mode="balance_stand",
            z1_q=(0.0, 0.60, -0.60, 0.0, 0.0, 0.0),
            duration_sec=8.0,
            description="Start B2 stand while Z1 moves to a conservative reach pose.",
        ),
        JointStage(
            trigger_sec=10.0,
            b2_mode="balance_stand",
            z1_q=(math.pi / 4.0, 0.80, -0.80, 0.0, 0.0, 0.0),
            duration_sec=2.5,
            description="Keep B2 standing while Z1 moves to the second waypoint.",
        ),
        JointStage(
            trigger_sec=13.0,
            b2_mode="stand_down",
            z1_q=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            duration_sec=4.0,
            description="Lower B2 while Z1 returns home.",
        ),
    )


class B2Z1JointValidationNode(Node):
    def __init__(self) -> None:
        super().__init__("b2z1_joint_validation")
        self.publisher_ = self.create_publisher(
            MujocoDemoCommand, "mujoco_demo_command", 10
        )
        self.sequence_ = joint_validation_sequence()
        self.stage_ = 0
        self.start_time_ = self.get_clock().now()
        self.timer_ = self.create_timer(0.05, self.step_sequence)

    def step_sequence(self) -> None:
        elapsed = (self.get_clock().now() - self.start_time_).nanoseconds / 1e9

        if self.stage_ < len(self.sequence_):
            stage = self.sequence_[self.stage_]
            if elapsed < stage.trigger_sec:
                return

            msg = MujocoDemoCommand()
            msg.b2_mode = stage.b2_mode
            msg.vx = 0.0
            msg.vy = 0.0
            msg.vyaw = 0.0
            msg.z1_q = list(stage.z1_q)
            msg.duration = stage.duration_sec
            self.publisher_.publish(msg)
            self.get_logger().info(
                f"Joint stage {self.stage_ + 1}/{len(self.sequence_)}: "
                f"{stage.description}"
            )
            self.stage_ += 1
            return

        if elapsed >= 17.5:
            self.get_logger().info("B2+Z1 joint validation sequence complete.")
            self.timer_.cancel()


def main() -> None:
    rclpy.init()
    node = B2Z1JointValidationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
