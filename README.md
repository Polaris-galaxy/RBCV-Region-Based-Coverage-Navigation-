# RBCV — Region-Based Coverage Navigation

本仓库：**基于区域的地图分区与圆盘覆盖规划**（离线 Python + 可选 ROS1 地图/分区发布）。  
仓库文件夹名通常为 **`RBCV-Region-Based-Coverage-Navigation-`**（克隆路径以你本机为准）。

**当前不强制在此执行 ROS 编译**；后续可在 Ubuntu + catkin（ROS1 Noetic）或仅 Python 环境中操作。

## 内容索引

| 路径 | 说明 |
|------|------|
| `src/coverage_planner/` | 圆盘覆盖规划（Python，`map_io` / `region_io`） |
| `src/map_tools/` | **地图资源 + Python 桥接 + ROS1 静态 `/map` 发布**（与上一项共用地图文件） |
| `src/coverage_regions_ros/` | ROS1：分区标签栅格广播（订阅 `/map`） |
| `src/Kimera部署指南.md` | Kimera 语义 SLAM 部署备忘（ROS1） |
| `src/ROS2终端指令.md` | ROS 2 CLI 备忘（与本仓库 ROS1 节点无关，仅供查阅） |

各子目录内另有 **README.md** 便于快速查看。

## 操作指南

完整的「划分地图区域 → 圆盘覆盖」工作流见 **[src/USAGE.md](src/USAGE.md)**。

每个文件的用途索引见 **[src/FILES.md](src/FILES.md)**。
