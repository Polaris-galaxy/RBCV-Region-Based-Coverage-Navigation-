# -*- coding: utf-8 -*-
"""
LKH-3 简化版 (Pure Python 教学实现)
====================================

注：
    这是一个纯 Python 的 LKH (Lin-Kernighan-Helsgaun) 算法教学实现，
    用于求解经典 TSP（旅行商问题）。代码重点在"看得懂"，而不是"跑得快"。

    真正的 LKH-3 是 C 语言写的、上万行代码、支持 10+ 种问题变体。
    这里只实现了最核心的几个思想：

        1. 贪心构造初始解
        2. 候选集 (Candidate Set) —— 只考虑最近的几个邻居
        3. 顺序 k-opt 的变长深度搜索 (Variable-depth sequential edge exchange)
        4. Double-Bridge 4-opt 扰动 (跳出局部最优)
        5. 多次重启 (Multi-start)

    运行：
        python lkh3.py

依赖：Python 标准库（random, math, time）。
"""

import math
import random
import time
from typing import List, Tuple



# 1. 距离工具
def euclidean(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    """计算两点间欧氏距离。"""
    return math.hypot(a[0] - b[0], a[1] - b[1])


def build_distance_matrix(coords: List[Tuple[float, float]]) -> List[List[float]]:
    """根据坐标预计算距离矩阵，避免重复计算。"""
    n = len(coords)
    dist = [[0.0] * n for _ in range(n)]
    for i in range(n):
        for j in range(i + 1, n):
            d = euclidean(coords[i], coords[j])
            dist[i][j] = d
            dist[j][i] = d
    return dist


def tour_length(tour: List[int], dist: List[List[float]]) -> float:
    """计算一条闭合路线的总长度。"""
    total = 0.0
    n = len(tour)
    for i in range(n):
        total += dist[tour[i]][tour[(i + 1) % n]]
    return total


# 2. 构造初始解：最近邻贪心
def nearest_neighbor_tour(dist: List[List[float]], start: int = 0) -> List[int]:
    """
    贪心构造：从起点出发，每次选择当前未访问且最近的城市。
    这是一个"不太好"但"够用"的初始解。
    """
    n = len(dist)
    unvisited = set(range(n))
    tour = [start]
    unvisited.remove(start)
    current = start
    while unvisited:
        nxt = min(unvisited, key=lambda x: dist[current][x])
        tour.append(nxt)
        unvisited.remove(nxt)
        current = nxt
    return tour


# 3. 候选集 (Candidate Set)
def build_candidate_set(dist: List[List[float]], k: int = 5) -> List[List[int]]:
    """
    为每个城市预计算 k 个最近邻居。
    换边时，新加入的边只从候选集中挑 —— 这是 LKH 加速的关键。

    原版 LKH 用更复杂的 α-nearness（基于 1-tree 下界），
    教学版简化为"欧氏距离最近的 k 个"。
    """
    n = len(dist)
    candidates = []
    for i in range(n):
        # 按距离排序，取除自己外最近的 k 个
        neighbors = sorted(range(n), key=lambda j: dist[i][j])
        neighbors = [j for j in neighbors if j != i][:k]
        candidates.append(neighbors)
    return candidates


# 4. 2-opt 换边：最基础的局部搜索（作为 LK 的特例）
def two_opt(tour: List[int], dist: List[List[float]],
            candidates: List[List[int]]) -> List[int]:
    """
    2-opt：拆除两条边 (a,b) 和 (c,d)，重新连接为 (a,c) 和 (b,d)。
    相当于把 b...c 这段子路径翻转。

    使用候选集加速：只考虑把 a 连到它的近邻上。
    """
    n = len(tour)
    # pos[city] = city 在 tour 中的下标，便于 O(1) 定位
    pos = [0] * n
    for idx, city in enumerate(tour):
        pos[city] = idx

    improved = True
    while improved:
        improved = False
        for i in range(n):
            a = tour[i]
            b = tour[(i + 1) % n]
            d_ab = dist[a][b]
            # 只尝试把 a 连到它的候选邻居上
            for c in candidates[a]:
                if c == a or c == b:
                    continue
                j = pos[c]
                d = tour[(j + 1) % n]
                if d == a:
                    continue
                # 增益 = 旧边 - 新边
                gain = (d_ab + dist[c][d]) - (dist[a][c] + dist[b][d])
                if gain > 1e-10:
                    # 翻转 tour[i+1 .. j]（处理环形）
                    _reverse_segment(tour, pos, (i + 1) % n, j)
                    improved = True
                    break
            if improved:
                break
    return tour


def _reverse_segment(tour: List[int], pos: List[int], i: int, j: int) -> None:
    """原地翻转 tour[i..j]（环形），同步更新 pos。"""
    n = len(tour)
    # 计算需要交换的次数
    if i <= j:
        length = j - i + 1
    else:
        length = n - i + j + 1
    for k in range(length // 2):
        a_idx = (i + k) % n
        b_idx = (j - k) % n
        tour[a_idx], tour[b_idx] = tour[b_idx], tour[a_idx]
        pos[tour[a_idx]] = a_idx
        pos[tour[b_idx]] = b_idx


# 5. LK-style 变长 k-opt 的核心思想（简化实现）
def lk_move(tour: List[int], dist: List[List[float]],
            candidates: List[List[int]], max_depth: int = 5) -> List[int]:
    """
    Lin-Kernighan 风格：变长 k-opt 搜索。

    核心思想：
        1. 选一条边 (t1, t2) 打算删掉
        2. 选一个 t2 的候选邻居 t3，加入新边 (t2, t3)
        3. 这会产生一条"半路径"(不是合法 tour)
        4. 尝试删掉另一条边 (t3, t4) 并连回 t1 → 得到 2-opt
           或者继续深入：加入 (t4, t5)、删掉 (t5, t6) ……直到能合法闭合
        5. 每一步都要求累积增益 > 0（贪心）

    这里用一个简化策略：
        做 "连续的 2-opt 改进" 直到没法再改善，相当于有限深度的 LK。
        真正 LKH 会严格按上述递归流程，并支持回溯，代码复杂得多。
    """
    tour = two_opt(tour, dist, candidates)
    return tour


# 6. 扰动：Double-Bridge 4-opt（LKH 用它来跳出局部最优）
def double_bridge(tour: List[int], rng: random.Random) -> List[int]:
    """
    Double-Bridge 是一种特殊的 4-opt 扰动：
        把 tour 随机切成 4 段: A B C D
        重新拼接为:           A D C B

    它的妙处在于：**任何 2-opt 或 3-opt 都无法撤销它**，
    所以能有效跳出 2/3-opt 的局部最优，而又不至于破坏太多好结构。
    """
    n = len(tour)
    if n < 8:
        return tour[:]
    # 随机选 3 个切点，保证每段至少 1 个城市
    p1 = 1 + rng.randint(0, n // 4)
    p2 = p1 + 1 + rng.randint(0, n // 4)
    p3 = p2 + 1 + rng.randint(0, n // 4)
    A = tour[:p1]
    B = tour[p1:p2]
    C = tour[p2:p3]
    D = tour[p3:]
    return A + D + C + B


# 7. 主求解器：多次重启 + 扰动
def lkh_solve(coords: List[Tuple[float, float]],
              runs: int = 10,
              candidate_k: int = 5,
              time_limit: float = 30.0,
              seed: int = 42,
              verbose: bool = True) -> Tuple[List[int], float]:
    """
    主入口：求解 TSP。

    参数：
        coords       : 城市坐标列表 [(x1,y1), (x2,y2), ...]
        runs         : 独立重启次数（每次初始解不同）
        candidate_k  : 每个城市的候选邻居数
        time_limit   : 总时间上限（秒）
        seed         : 随机种子
        verbose      : 是否打印过程

    返回：
        best_tour    : 最优路线（城市索引列表）
        best_length  : 最优路线长度
    """
    rng = random.Random(seed)
    n = len(coords)

    if verbose:
        print(f"[LKH] 城市数 = {n}, 重启次数 = {runs}, 候选数 = {candidate_k}")

    dist = build_distance_matrix(coords)
    candidates = build_candidate_set(dist, candidate_k)

    best_tour = None
    best_len = float("inf")
    start_time = time.time()

    for run in range(runs):
        if time.time() - start_time > time_limit:
            if verbose:
                print("[LKH] 达到时间上限，提前停止。")
            break

        # Step 1: 构造初始解（第一次用贪心，之后随机起点）
        start_city = 0 if run == 0 else rng.randint(0, n - 1)
        tour = nearest_neighbor_tour(dist, start_city)

        # Step 2: LK 优化 -> 达到局部最优
        tour = lk_move(tour, dist, candidates)
        cur_len = tour_length(tour, dist)

        # Step 3: 在该局部最优附近反复 double-bridge 扰动 + 再优化
        inner_no_improve = 0
        while inner_no_improve < 10:
            if time.time() - start_time > time_limit:
                break
            perturbed = double_bridge(tour, rng)
            perturbed = lk_move(perturbed, dist, candidates)
            new_len = tour_length(perturbed, dist)
            if new_len + 1e-10 < cur_len:
                tour = perturbed
                cur_len = new_len
                inner_no_improve = 0
            else:
                inner_no_improve += 1

        if cur_len < best_len:
            best_len = cur_len
            best_tour = tour[:]
            if verbose:
                print(f"[LKH] Run {run + 1}/{runs}  新最优长度 = {best_len:.4f}")

    return best_tour, best_len


# 8. TSPLIB 文件读取（可选，支持标准 .tsp 格式 EUC_2D 类型）
def read_tsplib(path: str) -> List[Tuple[float, float]]:
    """
    读取 TSPLIB 格式的 .tsp 文件（仅支持 EUC_2D 节点坐标）。
    例如 berlin52.tsp 等标准测试集。
    """
    coords = []
    with open(path, "r", encoding="utf-8") as f:
        in_coord = False
        for line in f:
            line = line.strip()
            if line.startswith("NODE_COORD_SECTION"):
                in_coord = True
                continue
            if line == "EOF" or line == "":
                continue
            if in_coord:
                parts = line.split()
                if len(parts) >= 3:
                    coords.append((float(parts[1]), float(parts[2])))
    return coords


# 9. 演示入口
def demo() -> None:
    """随机生成 50 个城市，跑一次 LKH 求解并打印结果。"""
    rng = random.Random(0)
    n = 50
    coords = [(rng.uniform(0, 100), rng.uniform(0, 100)) for _ in range(n)]

    print("=" * 60)
    print(f"LKH-3 简化版演示  (随机 {n} 个城市)")
    print("=" * 60)

    t0 = time.time()
    best_tour, best_len = lkh_solve(
        coords,
        runs=5,
        candidate_k=5,
        time_limit=10.0,
        seed=42,
        verbose=True,
    )
    elapsed = time.time() - t0

    print("-" * 60)
    print(f"最优路线长度 : {best_len:.4f}")
    print(f"耗时         : {elapsed:.3f} 秒")
    print(f"路线前 10 个城市索引 : {best_tour[:10]} ...")
    print("=" * 60)


if __name__ == "__main__":
    demo()
