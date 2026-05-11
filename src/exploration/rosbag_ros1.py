"""ROS 1 (.bag v2.0) 浏览话题与解码平面位姿.

依赖 ``pip install rosbags``（纯 Python，无需在本机安装 ROS）。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterator


@dataclass
class TopicInfo:
    topic: str
    msgtype: str
    msgcount: int


def try_get_ros1_typestores() -> list[tuple[str, object]]:
    from rosbags.typesys import Stores, get_typestore

    out: list[tuple[str, object]] = []
    for name in ("ROS1_NOETIC", "ROS1_MELODIC", "ROS1_KINETIC"):
        if hasattr(Stores, name):
            try:
                out.append((name, get_typestore(getattr(Stores, name))))
            except Exception:
                continue
    return out


def inspect_rosbag1(path: str | Path) -> list[TopicInfo]:
    """列出 topic / ROS1 类型名 / 近似条数（若连接元数据提供）。"""
    path = Path(path)
    try:
        from rosbags.rosbag1 import Reader
    except ImportError as e:
        raise ImportError("请 pip install rosbags") from e

    rows: list[TopicInfo] = []
    with Reader(path) as reader:
        for c in reader.connections:
            mc = getattr(c, "msgcount", None)
            if mc is None:
                mc = getattr(c, "count", None)
            try:
                mc_i = int(mc) if mc is not None else -1
            except Exception:
                mc_i = -1
            rows.append(TopicInfo(topic=str(c.topic), msgtype=str(c.msgtype), msgcount=mc_i))
        rows.sort(key=lambda x: x.topic)
        return rows


def _stores_chain() -> list[tuple[str, object]]:
    stores = try_get_ros1_typestores()
    if not stores:
        raise RuntimeError(
            "rosbags 未提供可用的 ROS1 类型仓，请升级: pip install -U rosbags"
        )
    return stores


def _deserialize_ros1(raw: bytes, mtype: str) -> object:
    last_exc: Exception | None = None
    for _name, ts in _stores_chain():
        try:
            return ts.deserialize_ros1(raw, mtype)
        except Exception as exc:
            last_exc = exc
            continue
    if last_exc is not None:
        raise last_exc
    raise RuntimeError("deserialize_ros1 全部失败")


def _xyz_yaw_from_ros_pose(pose: object) -> tuple[float, float, float]:
    import math

    px = float(pose.position.x)
    py = float(pose.position.y)
    qx = float(pose.orientation.x)
    qy = float(pose.orientation.y)
    qz = float(pose.orientation.z)
    qw = float(pose.orientation.w)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return px, py, yaw


def extract_pose_xy_yaw(msg: object, msgtype: str) -> tuple[float, float, float] | None:
    """从常见 ROS1 里程计 / 位姿消息取 ``x,y,yaw``。未知类型返回 ``None``。"""
    mt = msgtype.replace("nav_msgs/msg/", "nav_msgs/").lower()
    if "odometry" in mt:
        pose = getattr(getattr(msg, "pose", None), "pose", None)
        if pose is None:
            return None
        return _xyz_yaw_from_ros_pose(pose)
    if "posestamped" in mt:
        return _xyz_yaw_from_ros_pose(msg.pose)
    if "posewithcovariancestamped" in mt:
        return _xyz_yaw_from_ros_pose(msg.pose.pose)
    return None


def _stamp_to_seconds(stamp: object) -> float:
    if stamp is None:
        return 0.0
    if isinstance(stamp, (int, float)):
        x = float(stamp)
        return x * 1e-9 if x > 1e13 else x
    secs = getattr(stamp, "sec", getattr(stamp, "secs", None))
    nsec = getattr(stamp, "nanosec", getattr(stamp, "nsecs", getattr(stamp, "nsec", 0))) or 0
    try:
        s = float(secs if secs is not None else 0.0)
        ns = float(nsec if nsec is not None else 0.0)
        return s + ns * 1e-9
    except Exception:
        return 0.0


def _message_stamp_seconds(msg: object) -> float:
    if hasattr(msg, "header"):
        return _stamp_to_seconds(getattr(msg.header, "stamp"))
    # Odometry-like 可能没有 header.sec 而是自身 stamp
    st = getattr(msg, "stamp", None)
    return _stamp_to_seconds(st)


def _raw_timestamp_seconds(ts: object) -> float | None:
    """rosbags ``reader.messages`` 第三项常为 ``int`` 纳秒计数。"""
    if ts is None:
        return None
    if isinstance(ts, (int, float)):
        v = float(ts)
        return v * 1e-9 if v > 5e17 else v
    secs = getattr(ts, "secs", getattr(ts, "sec", None))
    nsecs = getattr(ts, "nsecs", getattr(ts, "nanosec", getattr(ts, "nsec", 0)))
    if secs is None:
        return None
    return float(secs) + float(nsecs or 0) * 1e-9


def iter_poses_ros1_topic(
    path: str | Path,
    *,
    topic: str,
    on_unsupported_msgtype: str = "skip",
    msg_predicate: Callable[[object, str], bool] | None = None,
) -> Iterator[tuple[float, float, float, float]]:
    """顺序输出 ``(t_s, x, y, yaw)``。

    ``on_unsupported_msgtype``: ``skip`` 跳过不可测平面的类型；``raise`` 报错。

    Args:
        msg_predicate: 可选过滤 ``(msg, msgtype)->bool``。
    """
    path = Path(path)
    try:
        from rosbags.rosbag1 import Reader
    except ImportError as e:
        raise ImportError("请 pip install rosbags") from e

    with Reader(path) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        if not conns:
            known = sorted({c.topic for c in reader.connections})
            preview = ", ".join(known[:24])
            raise ValueError(
                f"未找到话题 {topic!r}。\n请先运行 inspect："
                f" py scripts/inspect_rosbag1.py {path}\n"
                f"已有话题（前缀）: {preview}"
            )

        conn = conns[0]
        mt = str(conn.msgtype)

        for _c, bag_ts, rawdata in reader.messages(connections=[conn]):
            try:
                msg = _deserialize_ros1(rawdata, mt)
            except Exception:
                if on_unsupported_msgtype == "raise":
                    raise
                continue
            if msg_predicate is not None and not msg_predicate(msg, mt):
                continue
            pose = extract_pose_xy_yaw(msg, mt)
            if pose is None:
                if on_unsupported_msgtype == "raise":
                    raise TypeError(
                        f"消息类型 {mt!r} 暂不支持自动取 x,y,yaw；"
                        "请换用 /odom、/amcl_pose 等，或扩展 extract_pose_xy_yaw。"
                    )
                continue
            x, y, yaw = pose
            t_bag = _raw_timestamp_seconds(bag_ts)
            t_msg = _message_stamp_seconds(msg)
            t_s = t_bag if t_bag is not None else t_msg
            yield (float(t_s), float(x), float(y), float(yaw))
