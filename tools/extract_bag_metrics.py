#!/usr/bin/env python3
"""Extract paper metrics from ROS 2 bag directories.

Metrics:
- final_position_error_m: distance between last /localization pose and last /goal_pose
- max_position_jump_m: maximum XY jump between consecutive /localization samples
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Iterable, List, Sequence, Tuple

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message


def expand_bag_dirs(inputs: Sequence[str]) -> List[Path]:
    bag_dirs: List[Path] = []
    for item in inputs:
        path = Path(item).expanduser()
        matches = sorted(path.parent.glob(path.name)) if any(ch in item for ch in "*?[]") else [path]
        for match in matches:
            if match.is_dir() and (match / "metadata.yaml").exists():
                bag_dirs.append(match.resolve())
    # de-duplicate while preserving order
    unique: List[Path] = []
    seen = set()
    for bag_dir in bag_dirs:
        if bag_dir not in seen:
            unique.append(bag_dir)
            seen.add(bag_dir)
    return unique


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
        return []

    msg_type = get_message(topic_types[topic_name])
    messages: List[Tuple[int, object]] = []

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic != topic_name:
            continue
        messages.append((timestamp, deserialize_message(data, msg_type)))

    return messages


def final_position_error(localization_msgs: Sequence[Tuple[int, object]], goal_msgs: Sequence[Tuple[int, object]]) -> float | None:
    if not localization_msgs or not goal_msgs:
        return None
    loc = localization_msgs[-1][1].pose.pose.position
    goal = goal_msgs[-1][1].pose.position
    return math.hypot(loc.x - goal.x, loc.y - goal.y)


def max_position_jump(localization_msgs: Sequence[Tuple[int, object]]) -> float | None:
    if len(localization_msgs) < 2:
        return None
    max_jump = 0.0
    prev = localization_msgs[0][1].pose.pose.position
    for _, msg in localization_msgs[1:]:
        curr = msg.pose.pose.position
        jump = math.hypot(curr.x - prev.x, curr.y - prev.y)
        if jump > max_jump:
            max_jump = jump
        prev = curr
    return max_jump


def summarize_bag(bag_dir: Path, localization_topic: str, goal_topic: str) -> dict:
    localization_msgs = read_topic_messages(bag_dir, localization_topic)
    goal_msgs = read_topic_messages(bag_dir, goal_topic)
    return {
        "bag": bag_dir.name,
        "final_position_error_m": final_position_error(localization_msgs, goal_msgs),
        "max_position_jump_m": max_position_jump(localization_msgs),
        "localization_samples": len(localization_msgs),
        "goal_messages": len(goal_msgs),
    }


def write_csv(rows: Iterable[dict], output_path: Path) -> None:
    rows = list(rows)
    fieldnames = [
        "bag",
        "final_position_error_m",
        "max_position_jump_m",
        "localization_samples",
        "goal_messages",
    ]
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def fmt(value: float | None) -> str:
    if value is None:
        return "N/A"
    return f"{value:.3f}"


def main() -> None:
    parser = argparse.ArgumentParser(description="Extract final error and max localization jump from ROS 2 bags.")
    parser.add_argument(
        "--bags",
        nargs="+",
        required=True,
        help="Bag directories or glob patterns, e.g. ~/IICC_ws/rosbag/base_*",
    )
    parser.add_argument(
        "--localization-topic",
        default="/localization",
        help="Localization topic in map frame (default: /localization)",
    )
    parser.add_argument(
        "--goal-topic",
        default="/goal_pose",
        help="Goal topic (default: /goal_pose)",
    )
    parser.add_argument(
        "--output-csv",
        default="bag_metrics.csv",
        help="Output CSV path (default: bag_metrics.csv)",
    )
    args = parser.parse_args()

    bag_dirs = expand_bag_dirs(args.bags)
    if not bag_dirs:
        raise SystemExit("No bag directories found.")

    rows = [summarize_bag(bag_dir, args.localization_topic, args.goal_topic) for bag_dir in bag_dirs]
    write_csv(rows, Path(args.output_csv).expanduser().resolve())

    print("bag,final_position_error_m,max_position_jump_m,localization_samples,goal_messages")
    for row in rows:
        print(
            f"{row['bag']},{fmt(row['final_position_error_m'])},{fmt(row['max_position_jump_m'])},"
            f"{row['localization_samples']},{row['goal_messages']}"
        )


if __name__ == "__main__":
    main()
