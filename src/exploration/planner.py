"""近似最优探索回路：起点=终点，优先高权重格，边权来自 costmap。"""

from __future__ import annotations

import os
import random
import subprocess
from pathlib import Path

from .zoning import GridRect


def _resolve_planner_exe() -> str | None:
    """若设置 ``RBCV_EXPLORATION_PLANNER_EXE`` 或已本地编译 ``exploration_core_cpp``，返回可执行路径。"""
    v = (os.environ.get("RBCV_EXPLORATION_PLANNER_EXE") or "").strip()
    if v and Path(v).is_file():
        return v
    root = Path(__file__).resolve().parents[1]
    for rel in (
        Path("exploration_core_cpp") / "build" / "Release" / "rbcv_plan_tour.exe",
        Path("exploration_core_cpp") / "build" / "Release" / "rbcv_plan_tour",
        Path("exploration_core_cpp") / "build" / "rbcv_plan_tour.exe",
        Path("exploration_core_cpp") / "build" / "rbcv_plan_tour",
    ):
        cand = root / rel
        if cand.is_file():
            return str(cand)
    return None


def _native_plan_indices(
    n: int,
    start_idx: int,
    edge_weighted: list[tuple[int, int, int, float]],
) -> list[int] | None:
    exe = _resolve_planner_exe()
    if not exe:
        return None
    lines = [f"{n} {start_idx} {len(edge_weighted)}"]
    lines += [f"{ia} {ib} {w}" for ia, ib, _eid, w in edge_weighted]
    payload = ("\n".join(lines) + "\n").encode("ascii", errors="replace")
    try:
        proc = subprocess.run(
            [exe],
            input=payload,
            capture_output=True,
            timeout=max(30.0, 0.01 * n * n),
            check=False,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    if proc.returncode != 0:
        return None
    out = proc.stdout.decode("utf-8", errors="replace").strip().split()
    if not out:
        return None
    try:
        idxs = [int(x) for x in out]
    except ValueError:
        return None
    if len(idxs) < 2:
        return None
    return idxs


def _cell_touches_workspace_edge(
    cell: GridRect,
    workspace_xyxy: tuple[float, float, float, float],
    tol_m: float,
) -> bool:
    xmin, ymin, xmax, ymax = workspace_xyxy
    return (
        abs(cell.xmin - xmin) <= tol_m
        or abs(cell.xmax - xmax) <= tol_m
        or abs(cell.ymin - ymin) <= tol_m
        or abs(cell.ymax - ymax) <= tol_m
    )


def sample_random_start_cell_key(
    cells: list[GridRect],
    rect_weights: dict[tuple[int, int], float],
    workspace_xyxy: tuple[float, float, float, float],
    *,
    rng: random.Random,
    weight_boost: float = 1.35,
    edge_boost: float = 0.75,
    uniform_floor: float = 0.07,
    edge_tol_m: float = 0.08,
) -> tuple[int, int]:
    """按 **高权重 + 地图边界** 偏置抽样一个起始格坐标 ``(row, col)``。"""
    if not cells:
        return (0, 0)
    masses: list[float] = []
    keys: list[tuple[int, int]] = []
    for c in cells:
        k = (c.row, c.col)
        keys.append(k)
        w = float(rect_weights.get(k, 0.0))
        edge = _cell_touches_workspace_edge(c, workspace_xyxy, edge_tol_m)
        m = uniform_floor + weight_boost * w + (edge_boost if edge else 0.0)
        masses.append(max(1e-9, m))
    s = sum(masses)
    probs = [m / s for m in masses]
    j = rng.choices(range(len(cells)), weights=probs, k=1)[0]
    return keys[j]


def _plan_exploration_tour_python(
    cells: list[GridRect],
    edge_weighted: list[tuple[int, int, int, float]],
    start_idx: int,
) -> list[tuple[int, int]]:
    n = len(cells)
    adj: dict[int, dict[int, float]] = {i: {} for i in range(n)}
    for ia, ib, _eid, w in edge_weighted:
        adj[ia][ib] = w

    unvisited = set(range(n))
    cur = start_idx
    order = [cur]
    unvisited.remove(cur)

    while unvisited:
        best = None
        best_w = float("inf")
        for j in unvisited:
            w = adj[cur].get(j, float("inf"))
            if w < best_w:
                best_w = w
                best = j
        if best is None:
            break
        order.append(best)
        unvisited.remove(best)
        cur = best

    def tour_len(seq: list[int]) -> float:
        s = 0.0
        for a, b in zip(seq, seq[1:]):
            s += adj[a].get(b, 1e6)
        if len(seq) > 1:
            s += adj[seq[-1]].get(seq[0], 1e6)
        return s

    def two_opt(seq: list[int]) -> list[int]:
        improved = True
        best_seq = seq[:]
        best = tour_len(seq)
        while improved:
            improved = False
            for i in range(1, len(seq) - 2):
                for k in range(i + 1, len(seq)):
                    if k - i == 1:
                        continue
                    new = seq[:i] + seq[i:k][::-1] + seq[k:]
                    ln = tour_len(new)
                    if ln + 1e-9 < best:
                        best = ln
                        best_seq = new
                        seq = best_seq
                        improved = True
                        break
                if improved:
                    break
        return best_seq

    cyc = two_opt(order)
    closed = cyc + [cyc[0]]
    return [(cells[i].row, cells[i].col) for i in closed]


def plan_exploration_tour(
    cells: list[GridRect],
    edge_weighted: list[tuple[int, int, int, float]],
    *,
    start_cell_key: tuple[int, int],
) -> list[tuple[int, int]]:
    """启发式：从 ``start_cell_key`` 出发的改良最近邻 + 2-opt（节点为格子中心）。

    ``edge_weighted`` 行格式同 :func:`build_grid_cost_fields` 输出。
    若存在可执行文件 ``rbcv_plan_tour``（见 ``exploration_core_cpp``），则自动调用以加速。
    """
    if not cells:
        return []

    key_to_idx = {(c.row, c.col): i for i, c in enumerate(cells)}
    if start_cell_key not in key_to_idx:
        start_cell_key = (cells[0].row, cells[0].col)

    n = len(cells)
    si = key_to_idx[start_cell_key]
    nat = _native_plan_indices(n, si, edge_weighted)
    if nat is not None:
        exp = n + 1
        if len(nat) == exp and nat[0] == nat[-1]:
            if all(0 <= nat[i] < n for i in range(len(nat) - 1)):
                core = nat[:-1]
                if sorted(core) == list(range(n)):
                    return [(cells[i].row, cells[i].col) for i in nat]
    return _plan_exploration_tour_python(cells, edge_weighted, si)
