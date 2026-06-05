from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Z1Stage:
    trigger_sec: float
    duration_sec: float
    q: tuple[float, float, float, float, float, float]
    description: str


def official_waypoint_sequence() -> list[Z1Stage]:
    return [
        Z1Stage(
            trigger_sec=1.0,
            duration_sec=2.0,
            q=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            description="Stage 1: command Z1 to home pose.",
        ),
        Z1Stage(
            trigger_sec=3.2,
            duration_sec=2.0,
            q=(0.0, 1.0, -1.0, 0.0, 0.0, 0.0),
            description="Stage 2: command Z1 to reach pose.",
        ),
        Z1Stage(
            trigger_sec=5.4,
            duration_sec=2.0,
            q=(math.pi / 4.0, 1.0, -1.0, 0.0, 0.0, 0.0),
            description="Stage 3: command Z1 to waypoint_test wrist pose.",
        ),
        Z1Stage(
            trigger_sec=7.6,
            duration_sec=3.5,
            q=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            description="Stage 4: command Z1 back to home.",
        ),
    ]


def mujoco_safe_sequence() -> list[Z1Stage]:
    return [
        Z1Stage(
            trigger_sec=1.0,
            duration_sec=2.0,
            q=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            description="Stage 1: command Z1 to home pose.",
        ),
        Z1Stage(
            trigger_sec=3.2,
            duration_sec=2.0,
            q=(0.0, 1.0, -1.0, 0.0, 0.0, 0.0),
            description="Stage 2: command Z1 to reach pose.",
        ),
        Z1Stage(
            trigger_sec=5.4,
            duration_sec=2.0,
            q=(math.pi / 4.0, 1.0, -1.0, 0.0, 0.0, 0.0),
            description="Stage 3: command Z1 to waypoint_test wrist pose.",
        ),
        Z1Stage(
            trigger_sec=7.6,
            duration_sec=2.5,
            q=(0.0, 0.45, -0.45, 0.0, 0.0, 0.0),
            description="Stage 4: command Z1 to a gentle retract pose.",
        ),
        Z1Stage(
            trigger_sec=10.4,
            duration_sec=3.5,
            q=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            description="Stage 5: command Z1 back to home.",
        ),
    ]
