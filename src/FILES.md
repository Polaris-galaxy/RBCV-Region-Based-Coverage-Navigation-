# RBCV 仓库文件一览

说明：**RBCV** = Region-Based Coverage Navigation。下列路径均相对仓库根目录。

---

## 仓库根

| 文件 | 作用 |
|------|------|
| `README.md` | 仓库总览、模块索引、`USAGE.md` 入口 |
| `.gitignore` | Git 忽略规则（如 `__pycache__`、`.pytest_cache`） |

---

## `src/`（代码与文档根）

| 文件 | 作用 |
|------|------|
| `README.md` | `src/` 下各包与文档索引 |
| `USAGE.md` | **端到端操作指南**：分区 JSON → 预览 → 逐区覆盖 |
| `requirements.txt` | Python 依赖（规划器 + 示例可视化 + pytest） |
| `Kimera部署指南.md` | Kimera（ROS1 Noetic）部署备忘 |
| `ROS2终端指令.md` | ROS 2 CLI 命令参考（备忘；本仓库 ROS1 节点与其无关） |
| `FILES.md` | **本文件**：每个文件的用途索引 |

---

## `src/map_tools/`（地图资源与 ROS1 `/map` 发布）

| 文件 | 作用 |
|------|------|
| `README.md` | `map_tools` 包说明 |
| `package.xml` | catkin 包清单 |
| `CMakeLists.txt` | catkin 构建：Python 安装、`maps/` 与 `launch/` 安装 |
| `setup.py` | `python/map_tools` 作为 ROS Python 包安装 |
| `launch/publish_static_map.launch` | 从磁盘 YAML+PGM 启动静态 `/map` 发布 |
| `launch/README.md` | launch 用法备忘 |
| `scripts/publish_static_map.py` | rospy 节点：读 YAML/PGM → 发布 `OccupancyGrid` |
| `scripts/README.md` | 脚本参数说明 |
| `python/map_tools/__init__.py` | 包入口：`bootstrap`、`paths` |
| `python/map_tools/bootstrap.py` | 自动把 `coverage_planner` 加入 `sys.path` |
| `python/map_tools/paths.py` | 解析 `maps/` 目录（源码树 / `rospack` / `share`） |
| `python/map_tools/bridge.py` | `build_occupancy_grid_msg`、`load_free_mask_for_planning` |
| `maps/map7.yaml` / `map7.pgm` | 示例 ROS 地图（yaml + 栅格图像） |
| `maps/map8.yaml` / `map8.pgm` | 另一套示例地图 |
| `maps/regions_map7.example.json` | **长方形分区示例**（米，`map` 帧） |
| `maps/README.md` | 地图目录说明 |

---

## `src/coverage_planner/`（离线 Python 规划包）

| 文件 | 作用 |
|------|------|
| `README.md` | 算法概述、目录结构、安装与快速命令 |

### `coverage_planner/coverage_planner/`（Python 子包）

| 文件 | 作用 |
|------|------|
| `__init__.py` | 懒加载导出公共 API |
| `map_io.py` | 栅格加载、`load_ros_map`、`load_ros_map_occupancy`、EDT、中轴、`GridMap` |
| `region_io.py` | 分区 JSON → `labels` 掩码；与 ROS 地图原点/分辨率对齐 |
| `region_planner.py` | 分区形状分类、`plan_partitions`（并行规划、全局 ILS/软目标） |
| `candidates.py` | 候选点：六边形、中轴行走、反射顶点、随机 |
| `visibility.py` | 射线可见性、覆盖稀疏矩阵、`augment_for_feasibility` |
| `set_cover.py` | 贪心 Set Cover（含 `overlap_tiebreak`） |
| `refine.py` | ILS 局部搜索精修 |
| `planner.py` | `PlannerConfig`、`plan_coverage` 顶层流水线 |
| `viz.py` | Matplotlib 结果可视化 |
| `mpl_zh.py` | Matplotlib 注册/选择中文字体（划区等脚本用，避免 Windows 乱码） |
| `README.md` | 子模块文件索引 |

### `coverage_planner/scripts/`（命令行工具）

| 文件 | 作用 |
|------|------|
| `README.md` | 工具脚本顺序与命令 |
| `preview_real_map.py` | 打印地图统计并保存自由空间预览 PNG |
| `draw_regions.py` | **交互**：鼠标拖框 → 保存分区 JSON |
| `preview_regions.py` | **校验**：分区 JSON 叠加地图并保存预览 PNG |
| `demo_regions.py` | **端到端**：逐分区自适应覆盖，`分区覆盖结果.png` |

### `coverage_planner/docs/`（算法文档）

| 文件 | 作用 |
|------|------|
| `README.md` | 文档索引 |
| `ALGORITHM.md` | DGR、窄通道与可见性推导 |

### `coverage_planner/tests/`（单元测试）

| 文件 | 作用 |
|------|------|
| `README.md` | pytest 用法 |
| `test_set_cover.py` | Set Cover 逻辑测试 |
| `test_regions.py` | `region_io` 栅格化与掩码测试 |
| `test_region_planner.py` | 分区分类与 `plan_partitions` smoke |
| `test_map_io_occupancy.py` | `load_ros_map_occupancy` 与示例地图一致性 |

---

## `src/coverage_regions_ros/`（ROS1：分区标签广播）

| 文件 | 作用 |
|------|------|
| `README.md` | 包用途与改名备忘 |
| `package.xml` | catkin 包名与依赖 |
| `CMakeLists.txt` | 安装脚本与 launch |
| `launch/publish_partition_labels.launch` | 启动分区标签发布节点 |
| `launch/README.md` | launch 参数说明 |
| `scripts/partition_labels_publisher.py` | rospy：订阅 `/map` → 栅格化 JSON 矩形 → 发布分区 `OccupancyGrid` + RViz Marker |
| `scripts/README.md` | 节点私有参数说明 |

---

## 运行时生成的文件（通常不入库）

下列由脚本生成，**建议不入库**（已在根目录 `.gitignore` 中忽略）：

| 典型路径 | 来源 |
|----------|------|
| `src/coverage_planner/scripts/分区叠图预览.png` | `preview_regions.py` |
| `src/coverage_planner/scripts/分区覆盖结果.png` | `demo_regions.py` |
| `src/coverage_planner/scripts/地图栅格预览.png` | `preview_real_map.py` |

本地新建的 **`my_regions.json`** 建议放在 `src/map_tools/maps/` 并与示例分区路径一致。
