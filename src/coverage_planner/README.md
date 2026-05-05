# coverage_planner —— 含障碍大地图的等半径圆盘覆盖规划器

本目录属于 **RBCV（Region-Based Coverage Navigation）** 仓库的 Python 规划包。

针对 **大地图 + 窄通道（通道宽度 ≈ 观测直径） + 高障碍遮挡** 场景，
求解最小观测点集合，使所有可见性受限的圆盘并集覆盖整个自由空间。

## 1. 问题

给定二值栅格地图 `M`（自由=1，障碍=0），观测半径 `r`（像素），
求观测点集 `P = {p1,...,pn} ⊂ Free`，最小化 `|P|`，s.t.

```
Free  ⊆  ⋃_i ( Disk(p_i, r) ∩ Visible(p_i) )
```

其中 `Visible(p)` 为障碍遮挡下从 `p` 看到的可见域。
属于 NP-hard（艺术馆问题的有限可视距离版本），本包实现可扩展近似算法。

## 2. 总体架构（Divide-Greedy-Refine, DGR）

```
栅格地图 ──► 预处理（距离场+中轴）
              │
              ├──► 房间/通道分类（按局部宽度）
              │
              ├──► 候选点生成
              │     ├─ 通道：中轴自适应步长
              │     ├─ 房间：六边形点阵
              │     └─ 角落：反射顶点附近
              │
              ├──► 可见性裁剪：S(c)=Disk(c,r)∩Visible(c)
              │
              ├──► 贪心 Set Cover（lazy max-heap）
              │
              └──► 局部精修（2-opt + 连续微调，可选）
```

## 3. 目录结构

```
coverage_planner/
├── README.md                  （本文件）
├── docs/                      见 docs/README.md
│   └── ALGORITHM.md           算法详细推导与窄通道几何分析
├── coverage_planner/          主 Python 包
│   ├── map_io.py              栅格地图加载与预处理（距离场、中轴）
│   ├── region_io.py           长方形分区 JSON → labels 掩码（与 ROS 地图对齐）
│   ├── region_planner.py      逐分区自适应规划（open / narrow / mixed / tiny）
│   ├── candidates.py          候选观测点生成（六边形 / 中轴 / 反射顶点 / 随机）
│   ├── visibility.py          栅格可见性 + 圆盘裁剪 + 覆盖矩阵
│   ├── set_cover.py           贪心 Set Cover（含 overlap_tiebreak）
│   ├── refine.py              ILS 局部精修
│   ├── planner.py             顶层 DGR 流水线
│   └── viz.py                 可视化辅助
├── scripts/                   见 scripts/README.md（命令行工具）
└── tests/                     见 tests/README.md

地图与分区 JSON 集中在 **`../map_tools/maps/`**（由 ``map_tools`` 包维护，
可同时被 Python 与 ROS1 节点共用）；依赖统一在 **`../requirements.txt`**。

## 4. 安装

```bash
cd src
pip install -r requirements.txt
```

依赖：`numpy`、`scipy`、`scikit-image`、`imageio`、`matplotlib`、`pytest`（测试）。

## 5. 快速开始

完整端到端流程（划分 → 覆盖）见 [`../USAGE.md`](../USAGE.md)。最小命令：

```bash
cd src/coverage_planner
py scripts/preview_real_map.py ../map_tools/maps/map7.yaml
py scripts/draw_regions.py    ../map_tools/maps/map7.yaml ../map_tools/maps/my_regions.json
py scripts/preview_regions.py ../map_tools/maps/map7.yaml ../map_tools/maps/my_regions.json
py scripts/demo_regions.py    ../map_tools/maps/map7.yaml 2.5 ../map_tools/maps/my_regions.json
```

## 6. 当前实现状态

| 模块 | 状态 |
|---|---|
| `map_io.py`         | ✅ 栅格加载 + EDT + 中轴 |
| `region_io.py`      | ✅ 长方形分区 JSON + 栅格化 |
| `region_planner.py` | ✅ 形状分类 + 逐区自适应规划 |
| `candidates.py`     | ✅ 中轴自适应 + 六边形 + 反射顶点 |
| `visibility.py`     | ✅ 栅格光线遮挡 + 圆盘裁剪 |
| `set_cover.py`      | ✅ lazy heap 贪心 + overlap tiebreak |
| `planner.py`        | ✅ 端到端 DGR 主流程 |
| `refine.py`         | ✅ ILS 局部精修 |
| `viz.py`            | ✅ 结果可视化 |
| `partition.py`      | ⏳ TODO（自动区域分解；手工分区见 `region_io` / `region_planner`） |

## 7. 复杂度

- 候选点数 `|C| ≈ |骨架长度|/r + |房间面积|/r² + |反射顶点|`
- 单候选可见性扫描 `O(r²)`（栅格内）
- 贪心 Set Cover `O((|C|+|U|) log |C|)`
- 整体在 `4096×4096` 栅格、`r=64` 下单机分钟级可解

详见 `docs/ALGORITHM.md`。
