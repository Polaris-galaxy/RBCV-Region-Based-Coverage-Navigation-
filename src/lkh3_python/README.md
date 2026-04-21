# LKH-3 简化版（纯 Python 教学实现）

本目录实现了一个**纯 Python 的 LKH (Lin-Kernighan-Helsgaun) TSP 求解器**，
用于学习算法原理，**无任何外部依赖**。

## 文件结构

```
lkh3_python/
├── lkh3.py       # 核心算法实现（带详细中文注释）
├── example.py    # 使用示例
└── README.md     # 本文件
```

## 算法流程（对应 lkh3.py 中的函数）

```
┌─ nearest_neighbor_tour()   贪心构造初始解
│
├─ build_candidate_set()     预计算每个城市的 k 个近邻
│
├─ two_opt() / lk_move()     基于候选集的 k-opt 局部搜索
│
├─ double_bridge()           4-opt 扰动，跳出局部最优
│
└─ lkh_solve()               主循环：多次重启 + 扰动 + 优化
```

## 快速开始

### 1. 跑内置演示（50 个随机城市）

```bash
cd src/lkh3_python
python lkh3.py
```

输出示例：

```
============================================================
LKH-3 简化版演示  (随机 50 个城市)
============================================================
[LKH] 城市数 = 50, 重启次数 = 5, 候选数 = 5
[LKH] Run 1/5  新最优长度 = 567.8234
[LKH] Run 3/5  新最优长度 = 562.1187
------------------------------------------------------------
最优路线长度 : 562.1187
耗时         : 0.845 秒
============================================================
```

### 2. 在你自己的代码里调用

```python
from lkh3 import lkh_solve

coords = [(0, 0), (1, 5), (3, 2), (7, 8), (6, 1)]
tour, length = lkh_solve(coords, runs=5, time_limit=5.0)
print("最优路线:", tour)
print("长度:", length)
```

### 3. 读取 TSPLIB 标准测试集

```python
from lkh3 import read_tsplib, lkh_solve

coords = read_tsplib("berlin52.tsp")
tour, length = lkh_solve(coords, runs=10, time_limit=30.0)
```

> `berlin52.tsp` 等数据集可以从 TSPLIB 官网下载：
> http://comopt.ifi.uni-heidelberg.de/software/TSPLIB95/

## 参数说明

| 参数 | 含义 | 推荐值 |
|------|------|--------|
| `runs` | 独立重启次数（越多越接近最优） | 5~20 |
| `candidate_k` | 每个城市的候选邻居数 | 5（LKH 默认）|
| `time_limit` | 总时间上限（秒） | 按需 |
| `seed` | 随机种子（保证可复现） | 任意整数 |

## 和真实 LKH-3 的差距

| 特性 | 本实现 | 真实 LKH-3 |
|------|--------|------------|
| 语言 | Python | C |
| 代码量 | ~300 行 | ~10000+ 行 |
| 候选集 | 简化的最近邻 | α-nearness (基于 1-tree 下界) |
| k-opt | 仅 2-opt + double-bridge | 完整的序列 k-opt + 回溯 |
| 支持问题 | 仅对称 TSP | TSP/CVRP/VRPTW/PDPTW 等 10+ 种 |
| 性能 | 50~500 城市可用 | 百万级城市 |

所以：
- **学习/小规模实验** → 用本实现，看得懂、改得动
- **生产/大规模** → 用官方 LKH-3 或 Python 封装（`elkai`、`lkh` 等）

## 进一步学习

1. 尝试把 `lk_move()` 真正实现为序列 k-opt（支持回溯）
2. 把 `build_candidate_set()` 改成 α-nearness
3. 扩展到带容量约束 (CVRP) —— 违约时加惩罚项
4. 与 ROS2 结合：把求解器封装为 ROS2 节点，订阅目标点话题
