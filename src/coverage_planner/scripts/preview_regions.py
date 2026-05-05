"""把现有分区 JSON 叠加到地图上预览, 方便确认对齐.

用法::

    py scripts/preview_regions.py ../map_tools/maps/map7.yaml \
        ../map_tools/maps/regions_map7.example.json
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
MAP_TOOLS_MAPS = os.path.abspath(os.path.join(ROOT, "..", "map_tools", "maps"))
sys.path.insert(0, ROOT)

import numpy as np  # noqa: E402

from coverage_planner.map_io import load_ros_map  # noqa: E402
from coverage_planner.region_io import (  # noqa: E402
    load_regions_json,
    rasterize_regions,
)


def main():
    yaml_path = sys.argv[1] if len(sys.argv) >= 2 else os.path.join(
        MAP_TOOLS_MAPS, "map7.yaml"
    )
    regions_path = sys.argv[2] if len(sys.argv) >= 3 else os.path.join(
        MAP_TOOLS_MAPS, "regions_map7.example.json"
    )

    free, info = load_ros_map(yaml_path)
    spec = load_regions_json(regions_path)
    labels = rasterize_regions(spec, info, free.shape)
    h, w = free.shape
    ox, oy, _ = info.origin
    res = float(info.resolution)
    extent = (ox, ox + w * res, oy, oy + h * res)

    print(f"map = {h}x{w} px, regions = {len(spec.regions)}, "
          f"frame_id = {spec.frame_id!r}")
    for i, reg in enumerate(spec.regions, start=1):
        in_rect = int((labels == i).sum())
        in_rect_free = int(((labels == i) & free).sum())
        print(
            f"  R{i} id={reg.id!r}  "
            f"x=[{reg.xmin:.2f},{reg.xmax:.2f}] y=[{reg.ymin:.2f},{reg.ymax:.2f}] "
            f"px_in_rect={in_rect}  px_in_rect∩free={in_rect_free}"
        )

    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    fig, axes = plt.subplots(1, 2, figsize=(16, 8))
    axes[0].imshow(free.astype(np.uint8), cmap="gray", origin="lower",
                   extent=extent, interpolation="nearest")
    axes[0].set_title("regions overlay (orange) on map")
    axes[0].set_xlabel("x (m)"); axes[0].set_ylabel("y (m)")
    cmap = plt.get_cmap("tab10")
    for i, reg in enumerate(spec.regions):
        c = cmap(i % 10)
        axes[0].add_patch(Rectangle(
            (reg.xmin, reg.ymin), reg.xmax - reg.xmin, reg.ymax - reg.ymin,
            fill=False, edgecolor=c, linewidth=2,
        ))
        axes[0].text((reg.xmin + reg.xmax) / 2, (reg.ymin + reg.ymax) / 2,
                     reg.id, color=c, ha="center", va="center", fontsize=10)
    axes[0].set_aspect("equal")

    show = labels.astype(np.float32)
    show[~free] = -1
    axes[1].imshow(show, cmap="tab20", origin="lower", extent=extent,
                   interpolation="nearest")
    axes[1].set_title("labels & free (-1=outside any region or obstacle)")
    axes[1].set_xlabel("x (m)"); axes[1].set_ylabel("y (m)")
    axes[1].set_aspect("equal")

    out_path = os.path.join(HERE, "regions_preview.png")
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    print(f"saved -> {out_path}")
    plt.show()


if __name__ == "__main__":
    main()
