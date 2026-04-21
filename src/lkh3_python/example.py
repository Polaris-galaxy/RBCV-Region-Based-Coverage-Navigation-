# -*- coding: utf-8 -*-
"""
LKH-3 简化版 - 使用示例
========================

展示三种典型用法：
    1. 自定义坐标求解
    2. 随机生成城市对比初始解与优化后解
    3. 读取 TSPLIB 文件（需要先下载数据）
"""

import random

from lkh3 import (
    lkh_solve,
    nearest_neighbor_tour,
    build_distance_matrix,
    tour_length,
)


def example_1_custom_coords() -> None:
    """示例 1：自定义 10 个城市的坐标，求最短回路。"""
    print("\n" + "=" * 60)
    print("示例 1：自定义坐标")
    print("=" * 60)

    coords = [
        (0, 0), (2, 4), (5, 2), (7, 7), (3, 9),
        (8, 3), (6, 6), (1, 7), (4, 1), (9, 8),
    ]

    tour, length = lkh_solve(
        coords,
        runs=5,
        candidate_k=5,
        time_limit=5.0,
        seed=123,
        verbose=False,
    )

    print(f"城市数 : {len(coords)}")
    print(f"最优路线 : {tour}")
    print(f"最优长度 : {length:.4f}")


def example_2_compare_init_vs_optimized() -> None:
    """示例 2：对比"贪心初始解"和"LKH 优化后"的差距。"""
    print("\n" + "=" * 60)
    print("示例 2：贪心初始解 vs LKH 优化")
    print("=" * 60)

    rng = random.Random(2026)
    n = 100
    coords = [(rng.uniform(0, 100), rng.uniform(0, 100)) for _ in range(n)]

    dist = build_distance_matrix(coords)
    init_tour = nearest_neighbor_tour(dist, 0)
    init_len = tour_length(init_tour, dist)

    _, opt_len = lkh_solve(
        coords,
        runs=8,
        candidate_k=5,
        time_limit=10.0,
        seed=42,
        verbose=False,
    )

    improvement = (init_len - opt_len) / init_len * 100
    print(f"城市数        : {n}")
    print(f"贪心初始长度  : {init_len:.2f}")
    print(f"LKH 优化长度  : {opt_len:.2f}")
    print(f"改进幅度      : {improvement:.2f}%")


def example_3_tsplib_file() -> None:
    """
    示例 3：读取 TSPLIB 文件（需要先下载数据）。

    如果没有 berlin52.tsp，会跳过此示例。
    下载地址：http://comopt.ifi.uni-heidelberg.de/software/TSPLIB95/tsp/
    """
    print("\n" + "=" * 60)
    print("示例 3：TSPLIB 文件（berlin52.tsp）")
    print("=" * 60)

    try:
        from lkh3 import read_tsplib

        coords = read_tsplib("berlin52.tsp")
        print(f"读取到 {len(coords)} 个城市")

        tour, length = lkh_solve(
            coords,
            runs=10,
            candidate_k=5,
            time_limit=15.0,
            seed=42,
            verbose=True,
        )
        print(f"berlin52 已知最优解 ≈ 7542")
        print(f"本实现求得长度      = {length:.2f}")
    except FileNotFoundError:
        print("未找到 berlin52.tsp，请自行下载后放到当前目录。跳过此示例。")


if __name__ == "__main__":
    example_1_custom_coords()
    example_2_compare_init_vs_optimized()
    example_3_tsplib_file()
