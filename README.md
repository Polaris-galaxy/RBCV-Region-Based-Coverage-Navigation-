# RBCV — Region-Based Coverage Navigation

本仓库用于：在**含障碍的大栅格地图**上，把自由空间按**长方形分区**拆分后，求一组**等半径观测点（圆盘）**，使所有自由像素在**视线遮挡（visibility clipped）**约束下被覆盖，并尽量**减少观测点数量**；并可选用 **`exploration`** 模块，在检测语义与权重上做**全局探索回路**规划。

- **离线主流程**：纯 Python（不依赖 ROS 即可运行）。
- **可选在线发布**：ROS1（Noetic / catkin）可发布 `/map` 与“分区标签栅格”，便于下游节点对齐离线结果。

## 问题定义（Problem）

给定二值栅格自由空间 `Free` 与观测半径 `r`，对每个候选点 `p` 定义可见裁剪圆盘：

```
S(p) = Disk(p, r) ∩ Visible(p)
Visible(p) = { x ∈ Free : segment(p, x) lies entirely in Free }
```

目标是在可见性遮挡约束下，求最小集合 `P ⊂ Free` 使：

```
Free ⊆ ⋃_{p ∈ P} S(p)
```

该问题为 NP-hard（艺术馆问题的有限可视距离版本）。

## 方法概述（Method）

核心算法在 `src/rbcv_disk_coverage/`，采用**分区 + 近似覆盖**的工程化流水线：

- **Divide（分区）**：人工拖框或编辑 JSON 生成长方形分区（米坐标、与 ROS 对齐）。
- **Candidate generation（候选生成）**：按分区形状自适应：
  - open 房间：六边形点阵（减少冗余）
  - narrow/corridor：中轴（medial axis）策略 +（必要时）反射顶点附近候选
- **Visibility（可见性裁剪）**：极坐标 ray-marching 生成稀疏覆盖矩阵（候选×自由像素）。
- **Greedy set cover（初解）**：lazy heap 贪心覆盖（可选 overlap tiebreak 减少边缘叠加）。
- **Refine（精修）**：
  - 分区内 ILS（try_remove + swap_2_for_1 + swap_3_for_2 + 扰动修复）
  - 全局精修：把所有分区候选**并集**到整图可见性下再做 ILS / regreedy，削跨区冗余
  - **软目标**：可按 EDT/孤立小连通域软化“贴墙/小像素空隙”目标，避免为噪声像素专门留圆
- **并行**：分区规划支持 `thread/process` 两种后端（Windows 推荐 `process` 以明显利用多核）。

> 说明：当前实现为**纯 CPU** 计算，未使用 GPU。

## 快速开始（Quickstart）

端到端流程见 **[src/USAGE.md](src/USAGE.md)**。最小命令示例：

```powershell
cd <仓库根>\src
pip install -r requirements.txt

cd <仓库根>\src\rbcv_disk_coverage
py scripts\preview_real_map.py ..\map_tools\maps\map7.yaml
py scripts\draw_regions.py    ..\map_tools\maps\map7.yaml ..\map_tools\maps\my_regions.json
py scripts\preview_regions.py ..\map_tools\maps\map7.yaml ..\map_tools\maps\my_regions.json
py scripts\demo_regions.py    ..\map_tools\maps\map7.yaml 2.5 ..\map_tools\maps\my_regions.json
```

并行后端（多线程 / 多进程）与“一键最优”详见 `src/USAGE.md` 的相关章节。

## 仓库结构（Modules）

| 路径 | 说明 |
|------|------|
| `src/rbcv_disk_coverage/` | **离线覆盖规划器（Python）**：地图 IO、分区规划、可见性、Set Cover、ILS 精修 |
| `src/map_tools/` | 地图资源目录 + Python 桥接 + ROS1 静态 `/map` 发布 |
| `src/coverage_regions_ros/` | ROS1：订阅 `/map`，发布分区标签栅格（OccupancyGrid） |
| `src/exploration/` | **语义探索（Python）**：检测框/观测、矩形权重格、闭合 tour；可选 `rosbags` 读 ROS1 bag |
| `src/USAGE.md` | **端到端使用说明**（从划区到输出图） |
| `src/FILES.md` | 文件索引（每个文件用途一览） |
| `src/WORKSPACE_LAYOUT.md` | **`src/` 目录分工**（几何覆盖 / 语义探索 / ROS） |

## 文档索引

- **使用说明**：`src/USAGE.md`
- **算法推导**：`src/rbcv_disk_coverage/docs/ALGORITHM.md`
- **文件索引**：`src/FILES.md`
- **目录分工**：`src/WORKSPACE_LAYOUT.md`
