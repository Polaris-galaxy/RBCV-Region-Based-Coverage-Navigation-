"""分区独立覆盖 demo: 每个长方形分区按形状自动选择候选策略.

用法 (默认 r=2.5m, 与项目约定一致)::

    py scripts/demo_regions.py
    py scripts/demo_regions.py ../map_tools/maps/map7.yaml 2.5 \
        ../map_tools/maps/regions_map7.example.json

输出 ``scripts/分区覆盖结果.png`` 与中文控制台明细.
"""
from __future__ import annotations

import math
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
from coverage_planner.visibility import visible_disk  # noqa: E402


def _plot(free_full, partition, r_px, save_path):
    import matplotlib.pyplot as plt

    h, w = free_full.shape
    fig, ax = plt.subplots(figsize=(12, 12))
    bg = np.where(free_full, 255, 0).astype(np.uint8)
    ax.imshow(bg, cmap="gray", vmin=0, vmax=255,
              origin="upper", interpolation="nearest")

    points = partition.all_points()
    region_ids = partition.all_region_ids()
    cmap = plt.get_cmap("tab10")

    uniq_rids = sorted(set(int(x) for x in region_ids.tolist())) if points.shape[0] else []
    rid_to_color = {rid: cmap(i % 10) for i, rid in enumerate(uniq_rids)}

    rgba = np.zeros((h, w, 4), dtype=np.float32)
    n_rays = max(64, int(math.ceil(2.0 * math.pi * r_px)))
    alpha = 0.22

    n_pts = points.shape[0]
    if n_pts > 0:
        print(f"[绘图] 按可见性裁剪叠加 {n_pts} 个观测圆…")
    progress_step = max(1, n_pts // 10)
    for k, ((y, x), rid) in enumerate(zip(points, region_ids), start=1):
        rid = int(rid)
        color = rid_to_color[rid]
        mask = visible_disk(free_full, int(y), int(x), r_px, n_rays)
        if not mask.any():
            continue
        prev_a = rgba[mask, 3]
        new_a = prev_a + (1.0 - prev_a) * alpha
        for ch in range(3):
            prev_c = rgba[mask, ch]
            rgba[mask, ch] = (
                prev_c * prev_a + color[ch] * alpha * (1.0 - prev_a)
            ) / np.maximum(new_a, 1e-6)
        rgba[mask, 3] = new_a
        if k % progress_step == 0 or k == n_pts:
            print(f"[绘图] 进度 {k}/{n_pts}")

    ax.imshow(rgba, origin="upper", interpolation="nearest", zorder=1)

    legend_seen: set[int] = set()
    for (y, x), rid in zip(points, region_ids):
        rid = int(rid)
        color = rid_to_color[rid]
        if rid not in legend_seen:
            ax.scatter(
                [x], [y], s=26, c=[color], marker="x",
                linewidths=1.4, label=f"Region {rid}", zorder=3,
            )
            legend_seen.add(rid)
        else:
            ax.scatter(
                [x], [y], s=26, c=[color], marker="x",
                linewidths=1.4, zorder=3,
            )

    raw_n = partition.raw_total_circles
    final_n = partition.total_circles
    if partition.trim_stats is not None and raw_n != final_n:
        title = (
            f"Per-region coverage (visibility clipped) | "
            f"global trim: {raw_n} -> {final_n}"
        )
    else:
        title = (
            f"Per-region coverage (visibility clipped) | "
            f"total disks = {final_n}"
        )
    ax.set_title(title)
    ax.set_xlabel("x (px)")
    ax.set_ylabel("y (px)")

    if legend_seen:
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

    print(f"[演示] 地图 YAML        = {yaml_path}")
    print(f"[演示] 覆盖半径 r       = {r_meters} 米")
    print(f"[演示] 分区 JSON        = {regions_path}")

    free, info = load_ros_map(
        yaml_path,
        keep_largest_component=False,
        clean_map_verbose=True,
    )
    spec = load_regions_json(regions_path)
    labels = rasterize_regions(spec, info, free.shape)
    in_region = int(((labels >= 1) & free).sum())
    if in_region < 100:
        print(
            f"[演示] 警告：分区矩形与可走区只交到 {in_region} 像素，"
            "已回退到「整图作为单一分区」演示模式。"
        )
        labels = np.where(free, 1, 0).astype(np.int32)
    r_px = meters_to_pixels(r_meters, info.resolution)
    print(
        f"[演示] 地图尺寸={free.shape}，r（像素）={r_px:.2f}，"
        f"有效分区像素={int(((labels >= 1) & free).sum())}"
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
        global_trim=True,
        verbose=True,
    )
    elapsed = time.time() - t0

    print()
    print("=" * 72)
    if partition.trim_stats is not None:
        print(
            f"[演示] 汇总：圆盘数 {partition.raw_total_circles} → "
            f"{partition.total_circles}（全局精修共减 "
            f"{partition.trim_stats['removed']} 个），用时 {elapsed:.1f} 秒"
        )
    else:
        print(
            f"[演示] 汇总：圆盘总数 = {partition.total_circles}  "
            f"（用时 {elapsed:.1f} 秒）"
        )
    print("=" * 72)
    for rp in partition.region_plans:
        s = rp.shape
        n = rp.result.selected_points.shape[0]
        print(
            f"  分区{rp.region_id:>2} 类型={s.kind:<6} "
            f"可走像素={s.free_px:>7d} 长方形程度={s.rectangularity:.2f} "
            f"宽度比={s.width_ratio:.2f} → 圆盘数={n} "
            f"覆盖率={100*rp.result.coverage_ratio:.2f}%"
        )

    save_path = os.path.join(HERE, "分区覆盖结果.png")
    try:
        _plot(free, partition, r_px, save_path)
        print(f"[演示] 已保存结果图：{save_path}")
    except Exception as e:
        print(f"[演示] 绘图失败：{e}")


if __name__ == "__main__":
    main()
