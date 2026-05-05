"""分区独立覆盖 demo: 每个长方形分区按形状自动选择候选策略.

用法 (默认 r=2.5m, 与项目约定一致)::

    py examples/demo_regions.py
    py examples/demo_regions.py ../map_tools/maps/map7.yaml 2.5 \
        ../map_tools/maps/regions_map7.example.json

输出 ``examples/regions_result.png`` 与控制台明细.
"""
from __future__ import annotations

import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
MAP_TOOLS_MAPS = os.path.abspath(os.path.join(ROOT, "..", "map_tools", "maps"))
sys.path.insert(0, ROOT)

import numpy as np  # noqa: E402

from coverage_planner import (  # noqa: E402
    PlannerConfig, plan_partitions,
)
from coverage_planner.map_io import load_ros_map, meters_to_pixels  # noqa: E402
from coverage_planner.region_io import (  # noqa: E402
    load_regions_json,
    rasterize_regions,
)


def _plot(free_full, partition, r_px, save_path):
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle

    fig, ax = plt.subplots(figsize=(12, 12))
    ax.imshow(free_full, cmap="gray", origin="upper", interpolation="nearest")

    cmap = plt.get_cmap("tab10")
    for i, rp in enumerate(partition.region_plans):
        color = cmap(i % 10)
        pts = rp.result.selected_points
        for y, x in pts:
            ax.add_patch(Circle((float(x), float(y)), radius=r_px,
                                fill=True, facecolor=color, edgecolor="none",
                                alpha=0.18, linewidth=0))
        if pts.shape[0] > 0:
            ax.scatter(pts[:, 1], pts[:, 0], s=20, c=[color], marker="x",
                       label=f"R{rp.region_id} {rp.shape.kind} ({pts.shape[0]})")

    ax.set_title(
        f"per-region coverage | total circles = {partition.total_circles}"
    )
    ax.legend(loc="upper right", fontsize=8)
    ax.set_aspect("equal")
    plt.tight_layout()
    plt.savefig(save_path, dpi=150)
    plt.close(fig)


def main():
    yaml_path = os.path.join(MAP_TOOLS_MAPS, "map7.yaml")
    r_meters = 2.5
    regions_path = os.path.join(MAP_TOOLS_MAPS, "regions_map7.example.json")

    if len(sys.argv) >= 2:
        yaml_path = sys.argv[1]
    if len(sys.argv) >= 3:
        r_meters = float(sys.argv[2])
    if len(sys.argv) >= 4:
        regions_path = sys.argv[3]

    print(f"[demo] yaml         = {yaml_path}")
    print(f"[demo] r            = {r_meters} m")
    print(f"[demo] regions json = {regions_path}")

    free, info = load_ros_map(yaml_path)
    spec = load_regions_json(regions_path)
    labels = rasterize_regions(spec, info, free.shape)
    r_px = meters_to_pixels(r_meters, info.resolution)
    print(
        f"[demo] map={free.shape}, r_px={r_px:.2f}, "
        f"regions={len(spec.regions)}"
    )

    base = PlannerConfig(
        r=r_px,
        coverage_ratio=1.0,
        target_subsample=4,
        min_corridor=2.0,
        augment_for_full_cover=True,
        random_extra=200,
        enable_ils=True,
        ils_max_iter=40,
        ils_patience=10,
        overlap_tiebreak=True,
        verbose=False,
        seed=0,
    )

    t0 = time.time()
    partition = plan_partitions(
        free_full=free,
        labels=labels,
        r_px=r_px,
        base_config=base,
        verbose=True,
    )
    elapsed = time.time() - t0

    print()
    print("=" * 72)
    print(f"[demo] PARTITION SUMMARY  total = {partition.total_circles}  "
          f"({elapsed:.1f}s)")
    print("=" * 72)
    for rp in partition.region_plans:
        s = rp.shape
        n = rp.result.selected_points.shape[0]
        print(
            f"  R{rp.region_id:>2} kind={s.kind:<6} "
            f"free_px={s.free_px:>7d} rect={s.rectangularity:.2f} "
            f"wratio={s.width_ratio:.2f} -> |S|={n} "
            f"cov={100*rp.result.coverage_ratio:.2f}%"
        )

    save_path = os.path.join(HERE, "regions_result.png")
    try:
        _plot(free, partition, r_px, save_path)
        print(f"[demo] saved {save_path}")
    except Exception as e:
        print(f"[demo] plot failed: {e}")


if __name__ == "__main__":
    main()
