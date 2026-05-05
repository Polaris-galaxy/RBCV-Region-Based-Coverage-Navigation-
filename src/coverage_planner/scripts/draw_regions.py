"""交互式划分长方形区域：在地图上拖框 → 保存为分区 JSON.

用法::

    py scripts/draw_regions.py ../map_tools/maps/map7.yaml \
        ../map_tools/maps/my_regions.json

操作:
    - 鼠标左键拖动 = 画一个矩形 (松开后入列, 显示 R<id>)
    - n              = 给下一个矩形指定字符串 id (默认自动 R0/R1...)
    - u              = 撤销最后一个矩形
    - r              = 清空全部
    - s              = 保存到目标 JSON 路径
    - q              = 退出 (未保存内容会丢失)

坐标系:
    - 显示与保存均为 **地图坐标系 (米)**, 与 ROS map_server / RViz 一致;
    - 内部依赖 ``RosMapInfo.origin`` + ``resolution`` 完成像素 ↔ 米转换.
"""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
MAP_TOOLS_MAPS = os.path.abspath(os.path.join(ROOT, "..", "map_tools", "maps"))
sys.path.insert(0, ROOT)

import numpy as np  # noqa: E402

from coverage_planner.map_io import load_ros_map  # noqa: E402


def _build_extent(free: np.ndarray, info) -> tuple[float, float, float, float]:
    h, w = free.shape
    ox, oy, _ = info.origin
    res = float(info.resolution)
    return (ox, ox + w * res, oy, oy + h * res)


def main():
    if len(sys.argv) < 2:
        yaml_path = os.path.join(MAP_TOOLS_MAPS, "map7.yaml")
    else:
        yaml_path = sys.argv[1]

    if len(sys.argv) >= 3:
        save_path = sys.argv[2]
    else:
        save_path = os.path.join(MAP_TOOLS_MAPS, "my_regions.json")

    yaml_path = str(Path(yaml_path).resolve())
    save_path = str(Path(save_path).resolve())

    free, info = load_ros_map(yaml_path)
    h, w = free.shape
    extent = _build_extent(free, info)
    print(f"[draw] map: {h}x{w} px @ {info.resolution} m/px")
    print(f"[draw] map bounds (m): x=[{extent[0]:.2f}, {extent[1]:.2f}] "
          f"y=[{extent[2]:.2f}, {extent[3]:.2f}]")
    print(f"[draw] save target: {save_path}")

    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    from matplotlib.widgets import RectangleSelector

    fig, ax = plt.subplots(figsize=(11, 11))
    ax.imshow(
        free.astype(np.uint8),
        cmap="gray", origin="lower", extent=extent, interpolation="nearest",
    )
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_aspect("equal")
    ax.set_title("draw regions: drag=add, u=undo, r=clear, s=save, q=quit")

    state = {
        "regions": [],          # list[dict(id,xmin,ymin,xmax,ymax)]
        "next_id": "",          # 下一矩形 id (空则自动)
        "patches": [],          # 与 regions 等长
        "labels": [],
    }

    def _redraw_legend() -> None:
        for p in state["patches"]:
            try:
                p.remove()
            except Exception:
                pass
        for t in state["labels"]:
            try:
                t.remove()
            except Exception:
                pass
        state["patches"].clear()
        state["labels"].clear()
        for i, reg in enumerate(state["regions"]):
            rect = Rectangle(
                (reg["xmin"], reg["ymin"]),
                reg["xmax"] - reg["xmin"],
                reg["ymax"] - reg["ymin"],
                fill=False, edgecolor="orange", linewidth=1.5,
            )
            ax.add_patch(rect)
            txt = ax.text(
                (reg["xmin"] + reg["xmax"]) / 2,
                (reg["ymin"] + reg["ymax"]) / 2,
                reg["id"], color="orange", ha="center", va="center", fontsize=9,
            )
            state["patches"].append(rect)
            state["labels"].append(txt)
        fig.canvas.draw_idle()

    def on_select(eclick, erelease):
        x0, x1 = sorted((float(eclick.xdata), float(erelease.xdata)))
        y0, y1 = sorted((float(eclick.ydata), float(erelease.ydata)))
        if x1 - x0 < 1e-3 or y1 - y0 < 1e-3:
            return
        rid = state["next_id"] or f"R{len(state['regions'])}"
        state["regions"].append(
            {"id": rid, "xmin": x0, "ymin": y0, "xmax": x1, "ymax": y1}
        )
        state["next_id"] = ""
        print(f"[draw] + {rid}: x=[{x0:.2f},{x1:.2f}] y=[{y0:.2f},{y1:.2f}]")
        _redraw_legend()

    def on_key(event):
        if event.key == "u":
            if state["regions"]:
                rem = state["regions"].pop()
                print(f"[draw] - {rem['id']}")
                _redraw_legend()
        elif event.key == "r":
            state["regions"].clear()
            print("[draw] cleared")
            _redraw_legend()
        elif event.key == "n":
            try:
                rid = input("next region id (回车=自动): ").strip()
            except EOFError:
                rid = ""
            state["next_id"] = rid
            print(f"[draw] next id = {rid or '(auto)'}")
        elif event.key == "s":
            payload = {
                "frame_id": "map",
                "regions": state["regions"],
            }
            Path(save_path).write_text(
                json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
                encoding="utf-8",
            )
            print(f"[draw] saved {len(state['regions'])} regions -> {save_path}")
        elif event.key == "q":
            plt.close(fig)

    selector = RectangleSelector(
        ax, on_select,
        useblit=True,
        button=[1],
        minspanx=0.05, minspany=0.05,
        spancoords="data",
        interactive=False,
    )
    fig.canvas.mpl_connect("key_press_event", on_key)

    print("[draw] keys: drag=add, u=undo, r=clear, n=set next id, s=save, q=quit")
    plt.show()
    _ = selector


if __name__ == "__main__":
    main()
