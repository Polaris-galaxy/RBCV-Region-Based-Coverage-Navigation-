"""从 rosbag 读取机器人平面位姿。

- **ROS 2 bag**：本文件 ``iter_poses_from_bag``（``AnyReader`` + 常见 Pose/Odom）。
- **ROS 1 .bag**：使用 ``exploration.rosbag_ros1`` 或脚本 ``scripts/inspect_rosbag1.py``、
  ``scripts/dump_ros1_poses.py``。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterator


@dataclass
class Pose2D:
    t_s: float
    x: float
    y: float
    yaw: float


def iter_poses_from_bag(
    bag_path: str | Path,
    *,
    topic: str,
    xy_from_fields: tuple[str, str] = ("x", "y"),
    yaw_field: str | None = "theta",
    t_field: str = "timestamp",
) -> Iterator[Pose2D]:
    """使用 ``rosbags`` 读取常见 ``nav_msgs/msg/Odometry`` 简化版不可靠，故按 topic 指定的原始类型解码。

    支持：
    - ``geometry_msgs/msg/PoseStamped``
    - ``nav_msgs/msg/Odometry``
    """
    try:
        from rosbags.highlevel import AnyReader  # type: ignore
    except ImportError as e:
        raise ImportError(
            "需要安装 rosbags: pip install rosbags"
        ) from e

    from rosbags.typesys import Stores, get_typestore

    ts = get_typestore(Stores.ROS2_HUMBLE)  # 多数 Humble-format bag 可读

    bag_path = Path(bag_path)
    with AnyReader([bag_path]) as reader:
        connections = [c for c in reader.connections if c.topic == topic]
        if not connections:
            names = sorted({c.topic for c in reader.connections})
            raise ValueError(
                f"未找到 topic {topic!r}。已知 topics（部分）: {names[:40]}..."
            )
        con = connections[0]
        for _, t_raw, raw in reader.messages(connections=[con]):
            msg = ts.deserialize_cdr(raw, con.msgtype)

            stamp = getattr(msg.header, "stamp", None) if hasattr(msg, "header") else None
            if stamp is not None:
                t_s = float(stamp.sec) + float(stamp.nanosec) * 1e-9
            else:
                t_s = float(getattr(msg, t_field, 0.0))

            if con.msgtype.endswith("Odometry"):
                pose = msg.pose.pose
            else:
                pose = msg.pose

            x = float(pose.position.x)
            y = float(pose.position.y)
            qz = float(pose.orientation.z)
            qw = float(pose.orientation.w)
            yaw = 2.0 * __import__("math").atan2(qz, qw)

            yield Pose2D(t_s=t_s, x=x, y=y, yaw=yaw)
