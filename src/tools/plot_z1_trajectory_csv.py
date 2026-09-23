#!/usr/bin/env python3

"""
Plot Unitree Z1 trajectory logs recorded by record_trajectory_states.py.

Outputs PNG figures into the requested directory, defaulting to
/home/liu/b2z1_ros2_ws/src/z1_ros2/docs.
"""

from __future__ import annotations

import argparse
import csv
from collections import OrderedDict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


DEFAULT_JOINT_CSV = Path("/home/liu/b2z1_ros2_ws/z1_logs/z1_trajectory_joint_states.csv")
DEFAULT_CTRL_CSV = Path("/home/liu/b2z1_ros2_ws/z1_logs/z1_trajectory_controller_state.csv")
DEFAULT_OUTPUT_DIR = Path("/home/liu/b2z1_ros2_ws/src/z1_ros2/docs")


def _to_float(value: str):
    if value is None:
        return None
    value = value.strip()
    if not value:
        return None
    try:
        return float(value)
    except ValueError:
        return None


def _read_csv(path: Path):
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
        return reader.fieldnames or [], rows


def _group_joint_state_columns(fieldnames):
    groups = OrderedDict()
    for name in fieldnames:
        if name == "t_sec":
            continue
        if name.endswith("_pos"):
            joint = name[:-4]
            groups.setdefault(joint, {})["pos"] = name
        elif name.endswith("_vel"):
            joint = name[:-4]
            groups.setdefault(joint, {})["vel"] = name
        elif name.endswith("_eff"):
            joint = name[:-4]
            groups.setdefault(joint, {})["eff"] = name
    return groups


def _group_controller_columns(fieldnames):
    groups = OrderedDict()
    for name in fieldnames:
        if name == "t_sec":
            continue
        for suffix in ("_ref_pos", "_ref_vel", "_ref_eff", "_fb_pos", "_fb_vel", "_fb_eff", "_err_pos", "_err_vel", "_err_eff", "_out_pos", "_out_vel", "_out_eff"):
            if name.endswith(suffix):
                joint = name[: -len(suffix)]
                groups.setdefault(joint, {})[suffix[1:]] = name
                break
    return groups


def _extract_time_and_series(rows, time_key, column_map):
    time_values = []
    series = {key: [] for key in column_map}
    for row in rows:
        time_values.append(_to_float(row.get(time_key)))
        for key, column in column_map.items():
            series[key].append(_to_float(row.get(column)))
    return time_values, series


def _plot_metric_grid(time_values, data_map, joints, metric_key, ylabel, title, output_path, legend_labels=None):
    num_joints = len(joints)
    cols = 2 if num_joints > 1 else 1
    rows = (num_joints + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(14, 3.2 * rows), sharex=True)
    axes = list(axes.flat) if hasattr(axes, "flat") else [axes]

    for idx, joint in enumerate(joints):
        ax = axes[idx]
        for series_key, series_label in data_map[joint]:
            ax.plot(time_values, series_key, label=series_label, linewidth=1.5)
        ax.set_title(joint)
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)
        if idx == 0:
            ax.legend(loc="best", fontsize=9)

    for idx in range(num_joints, len(axes)):
        axes[idx].axis("off")

    for ax in axes[max(0, num_joints - cols):]:
        ax.set_xlabel("time [s]")

    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(output_path, dpi=200, bbox_inches="tight")
    plt.close(fig)


def _plot_joint3_aligned(time_values, rows, output_dir: Path):
    joint = "joint3"

    actual_pos = [_to_float(row.get("joint3_pos")) for row in rows]
    actual_vel = [_to_float(row.get("joint3_vel")) for row in rows]
    actual_eff = [_to_float(row.get("joint3_eff")) for row in rows]

    fig, axes = plt.subplots(3, 1, figsize=(14, 9), sharex=True)
    series = [
        (actual_pos, "position [rad]", "joint3 actual position"),
        (actual_vel, "velocity [rad/s]", "joint3 actual velocity"),
        (actual_eff, "effort [Nm]", "joint3 actual effort"),
    ]
    for ax, (values, ylabel, title) in zip(axes, series):
        ax.plot(time_values, values, linewidth=1.6)
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel("time [s]")
    fig.suptitle("Z1 joint3 actual state aligned by time")
    fig.tight_layout()
    fig.savefig(output_dir / "z1_joint3_actual_aligned.png", dpi=200, bbox_inches="tight")
    plt.close(fig)


def plot_joint_states(joint_csv: Path, output_dir: Path):
    fieldnames, rows = _read_csv(joint_csv)
    groups = _group_joint_state_columns(fieldnames)
    if not groups:
        raise RuntimeError(f"No joint-state columns found in {joint_csv}")

    time_values, series = _extract_time_and_series(rows, "t_sec", {
        f"{joint}_{metric}": column
        for joint, mapping in groups.items()
        for metric, column in mapping.items()
    })

    joints = list(groups.keys())
    for metric, ylabel, suffix in (
        ("pos", "position [rad]", "position"),
        ("vel", "velocity [rad/s]", "velocity"),
        ("eff", "effort [Nm]", "effort"),
    ):
        data_map = OrderedDict()
        for joint in joints:
            data_map[joint] = [(
                series[f"{joint}_{metric}"],
                metric,
            )]
        _plot_metric_grid(
            time_values,
            data_map,
            joints,
            metric,
            ylabel,
            f"Z1 actual joint {suffix} over time",
            output_dir / f"z1_actual_joint_{suffix}.png",
        )

    _plot_joint3_aligned(time_values, rows, output_dir)


def plot_controller_states(ctrl_csv: Path, output_dir: Path):
    fieldnames, rows = _read_csv(ctrl_csv)
    groups = _group_controller_columns(fieldnames)
    if not groups:
        raise RuntimeError(f"No controller-state columns found in {ctrl_csv}")

    time_values = [_to_float(row.get("t_sec")) for row in rows]
    joints = list(groups.keys())

    def build_series(metric):
        data_map = OrderedDict()
        for joint in joints:
            columns = groups[joint]
            series_list = []
            for prefix, label in (("ref", "reference"), ("fb", "feedback"), ("err", "error"), ("out", "output")):
                key = f"{prefix}_{metric}"
                if key in columns:
                    series_list.append((_extract_column(rows, columns[key]), label))
            data_map[joint] = series_list
        return data_map

    for metric, ylabel, suffix in (
        ("pos", "position [rad]", "position_compare"),
        ("vel", "velocity [rad/s]", "velocity_compare"),
        ("eff", "effort [Nm]", "effort_compare"),
    ):
        _plot_metric_grid(
            time_values,
            build_series(metric),
            joints,
            metric,
            ylabel,
            f"Z1 controller {suffix.replace('_', ' ')} over time",
            output_dir / f"z1_controller_{suffix}.png",
        )

    joint = "joint3"
    if joint in groups:
        fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
        aligned_series = [
            ("pos", "position [rad]", "reference vs feedback vs error"),
            ("vel", "velocity [rad/s]", "reference vs feedback vs error"),
            ("eff", "effort [Nm]", "reference vs feedback vs error"),
        ]
        for ax, (metric, ylabel, title) in zip(axes, aligned_series):
            columns = groups[joint]
            for prefix, label in (("ref", "reference"), ("fb", "feedback"), ("err", "error"), ("out", "output")):
                key = f"{prefix}_{metric}"
                if key in columns:
                    values = [_to_float(row.get(columns[key])) for row in rows]
                    ax.plot(time_values, values, linewidth=1.5, label=label)
            ax.set_ylabel(ylabel)
            ax.set_title(f"joint3 controller {title}")
            ax.grid(True, alpha=0.3)
            ax.legend(loc="best", fontsize=9)
        axes[-1].set_xlabel("time [s]")
        fig.suptitle("Z1 joint3 controller state aligned by time")
        fig.tight_layout()
        fig.savefig(output_dir / "z1_joint3_controller_aligned.png", dpi=200, bbox_inches="tight")
        plt.close(fig)


def _extract_column(rows, column_name):
    return [_to_float(row.get(column_name)) for row in rows]


def main():
    parser = argparse.ArgumentParser(description="Plot Z1 trajectory CSV logs.")
    parser.add_argument("--joint-csv", type=Path, default=DEFAULT_JOINT_CSV)
    parser.add_argument("--controller-csv", type=Path, default=DEFAULT_CTRL_CSV)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)

    plot_joint_states(args.joint_csv, args.output_dir)
    plot_controller_states(args.controller_csv, args.output_dir)

    print(f"Saved figures to {args.output_dir}")


if __name__ == "__main__":
    main()
