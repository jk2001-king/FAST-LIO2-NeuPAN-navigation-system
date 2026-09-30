#!/usr/bin/env python3
"""Plot baseline vs combined XY trajectories from ROS 2 bag directories."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import List, Sequence, Tuple

import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message


def read_topic_messages(bag_dir: Path, topic_name: str) -> List[Tuple[int, object]]:
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
        ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )

    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    if topic_name not in topic_types:
        raise ValueError(f"Topic {topic_name!r} not found in bag {bag_dir}")

    msg_type = get_message(topic_types[topic_name])
    messages: List[Tuple[int, object]] = []

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic != topic_name:
            continue
        messages.append((timestamp, deserialize_message(data, msg_type)))

    if not messages:
        raise ValueError(f"No messages read from topic {topic_name!r} in bag {bag_dir}")

    return messages


def pose_xy(messages: Sequence[Tuple[int, object]]) -> Tuple[List[float], List[float]]:
    xs = [msg.pose.pose.position.x for _, msg in messages]
    ys = [msg.pose.pose.position.y for _, msg in messages]
    return xs, ys


def normalize_xy(xs: Sequence[float], ys: Sequence[float]) -> Tuple[List[float], List[float]]:
    if not xs or not ys:
        return [], []
    x0 = xs[0]
    y0 = ys[0]
    return [x - x0 for x in xs], [y - y0 for y in ys]


def goal_xy_list(messages: Sequence[Tuple[int, object]]) -> List[Tuple[float, float]]:
    return [(msg.pose.position.x, msg.pose.position.y) for _, msg in messages]


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Create a baseline vs combined trajectory comparison figure from ROS 2 bags."
    )
    parser.add_argument("--base-bag", required=True, help="Path to the baseline ROS 2 bag directory")
    parser.add_argument("--combined-bag", required=True, help="Path to the combined ROS 2 bag directory")
    parser.add_argument(
        "--pose-topic",
        default="/localization",
        help="Pose/odometry topic to use for XY trajectory extraction (default: /localization)",
    )
    parser.add_argument(
        "--goal-topic",
        default="/goal_pose",
        help="Goal topic to use for goal marker extraction (default: /goal_pose)",
    )
    parser.add_argument(
        "--output",
        default="trajectory_comparison.png",
        help="Output PNG path (default: trajectory_comparison.png)",
    )
    parser.add_argument(
        "--title",
        default="Trajectory Comparison",
        help="Figure title (default: Trajectory Comparison)",
    )
    parser.add_argument(
        "--normalize-start",
        action="store_true",
        help="Normalize each run to start at (0,0) before plotting",
    )
    args = parser.parse_args()

    base_bag = Path(args.base_bag).expanduser().resolve()
    combined_bag = Path(args.combined_bag).expanduser().resolve()
    output = Path(args.output).expanduser().resolve()

    base_pose_msgs = read_topic_messages(base_bag, args.pose_topic)
    combined_pose_msgs = read_topic_messages(combined_bag, args.pose_topic)

    base_x, base_y = pose_xy(base_pose_msgs)
    combined_x, combined_y = pose_xy(combined_pose_msgs)

    base_goals = goal_xy_list(read_topic_messages(base_bag, args.goal_topic))
    combined_goals = goal_xy_list(read_topic_messages(combined_bag, args.goal_topic))

    if args.normalize_start:
        base_x, base_y = normalize_xy(base_x, base_y)
        combined_x, combined_y = normalize_xy(combined_x, combined_y)
        base_x0 = base_pose_msgs[0][1].pose.pose.position.x
        base_y0 = base_pose_msgs[0][1].pose.pose.position.y
        combined_x0 = combined_pose_msgs[0][1].pose.pose.position.x
        combined_y0 = combined_pose_msgs[0][1].pose.pose.position.y
        base_goals = [(gx - base_x0, gy - base_y0) for gx, gy in base_goals]
        combined_goals = [(gx - combined_x0, gy - combined_y0) for gx, gy in combined_goals]

    plt.figure(figsize=(8, 5))
    plt.plot(base_x, base_y, color="red", linewidth=2.0, label="FAST-LIO2 + Nav2")
    plt.plot(combined_x, combined_y, color="green", linewidth=2.0, label="Combined Tech")

    plt.scatter(base_x[0], base_y[0], color="black", marker="o", s=50, label="Start")
    for idx, (gx, gy) in enumerate(base_goals):
        plt.scatter(
            gx,
            gy,
            color="red",
            marker="x",
            s=80,
            label="Baseline Goal" if idx == 0 else None,
        )
    for idx, (gx, gy) in enumerate(combined_goals):
        plt.scatter(
            gx,
            gy,
            color="green",
            marker="x",
            s=80,
            label="Combined Goal" if idx == 0 else None,
        )
    plt.scatter(base_x[-1], base_y[-1], color="red", marker="s", s=40, label="Baseline End")
    plt.scatter(combined_x[-1], combined_y[-1], color="green", marker="s", s=40, label="Combined End")

    plt.axis("equal")
    axis_suffix = "relative" if args.normalize_start else "map frame"
    plt.xlabel(f"x (m, {axis_suffix})")
    plt.ylabel(f"y (m, {axis_suffix})")
    plt.title(args.title)
    plt.grid(True, alpha=0.3)
    plt.legend(loc="best", fontsize=9)
    plt.tight_layout()

    output.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(output, dpi=200)
    print(f"Saved trajectory figure to: {output}")


if __name__ == "__main__":
    main()
