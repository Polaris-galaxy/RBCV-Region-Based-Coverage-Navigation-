"""贪心 Set Cover 单元测试。"""
from __future__ import annotations

import os
import sys

import numpy as np
from scipy.sparse import csr_matrix

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

from coverage_planner.set_cover import greedy_set_cover  # noqa: E402


def make_matrix(rows):
    n = len(rows)
    m = max(max(r) for r in rows) + 1
    A = np.zeros((n, m), dtype=bool)
    for i, r in enumerate(rows):
        A[i, list(r)] = True
    return csr_matrix(A)


def test_simple_cover_full():
    A = make_matrix(
        [
            [0, 1, 2],
            [2, 3, 4],
            [4, 5],
            [0, 5],
        ]
    )
    sel, n_cov = greedy_set_cover(A, coverage_ratio=1.0)
    covered = np.zeros(A.shape[1], dtype=bool)
    for i in sel:
        covered[A.indices[A.indptr[i] : A.indptr[i + 1]]] = True
    assert covered.all()
    assert n_cov == A.shape[1]
    assert len(sel) <= 3


def test_partial_cover():
    A = make_matrix(
        [
            [0, 1, 2, 3],
            [4, 5],
            [6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19],
        ]
    )
    sel, n_cov = greedy_set_cover(A, coverage_ratio=0.6)
    assert n_cov >= int(0.6 * A.shape[1])


def test_empty_target():
    A = make_matrix([[0, 1, 2]])
    target = np.zeros(A.shape[1], dtype=bool)
    sel, n_cov = greedy_set_cover(A, target_mask=target)
    assert sel == []
    assert n_cov == 0


def test_edt_tiebreak_prefers_higher_priority():
    """同增益、同重叠时选 tiebreak_priority 更大的行."""
    A = make_matrix([[0], [0]])
    pri = np.array([1.0, 100.0], dtype=np.float64)
    sel, n_cov = greedy_set_cover(
        A,
        coverage_ratio=1.0,
        overlap_tiebreak=True,
        tiebreak_priority=pri,
    )
    assert n_cov == 1
    assert sel == [1]


def test_nnz_tiebreak_prefers_fewer_visible_cells():
    """增益/重叠/pri 相同时选 CSR 行更短（可见格更少）."""
    A = make_matrix([[0, 1], [0]])
    target = np.array([True, False], dtype=bool)
    pri = np.array([1.0, 1.0], dtype=np.float64)
    sel, n_cov = greedy_set_cover(
        A,
        target_mask=target,
        coverage_ratio=1.0,
        overlap_tiebreak=True,
        tiebreak_priority=pri,
        nnz_tiebreak=True,
    )
    assert n_cov == 1
    assert sel == [1]


if __name__ == "__main__":
    test_simple_cover_full()
    test_partial_cover()
    test_empty_target()
    test_edt_tiebreak_prefers_higher_priority()
    print("ok")
