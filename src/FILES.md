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
| `WORKSPACE_LAYOUT.md` | **`src/` 目录分工**（`rbcv_disk_coverage` / `exploration` / ROS） |
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
| `python/map_tools/bootstrap.py` | 自动定位 ``rbcv_disk_coverage`` 并将包 ``coverage_planner`` 加入 `sys.path` |
| `python/map_tools/paths.py` | 解析 `maps/` 目录（源码树 / `rospack` / `share`） |
| `python/map_tools/bridge.py` | `build_occupancy_grid_msg`、`load_free_mask_for_planning` |
| `maps/map7.yaml` / `map7.pgm` | 示例 ROS 地图（yaml + 栅格图像） |
| `maps/map8.yaml` / `map8.pgm` | 另一套示例地图 |
| `maps/regions_map7.example.json` | **长方形分区示例**（米，`map` 帧） |
| `maps/README.md` | 地图目录说明 |

---

## `src/exploration/`（语义探索与加权回路）

| 文件 / 目录 | 作用 |
|-------------|------|
| `README.md` | 数据契约、与圆盘覆盖目录 `rbcv_disk_coverage` 的配合方式 |
| `regions_bridge.py` | `regions.json`（分区矩形）→ `DetectionZone` 列表 |
| `survey/zone_catalog.py` | **survey_zones.json**：检测框 + 覆盖圆心/半径 |
| `survey/coverage_geometry.py` | 机器人是否处于某框的「覆盖可观测」区域 |
| `survey/pose_timeline.py` | rosbag 时间序列插值 → 平面位姿 |
| `survey/rosbag_merge.py` | rosbag 位姿 + 视觉 JSONL → `ObjectObservation` |
| `survey/stack.py` | `run_semantic_stack_from_files` 一键：调查 → 权重 → 网格 → tour |
| `pipeline.py` | `run_post_survey_planning`：观测 → 权重 → 网格代价 → 闭合 tour |
| `config.py` | `DetectionZone`、`SpatialRectPrior`（矩形格先验加成）等 |
| `detection_zones.py` | 检测框 JSON（`zones` 列表）读写 |
| `zoning.py` / `costmap.py` / `planner.py` | 均匀矩形、代价场、最近邻 + 2-opt |
| `fusion.py` / `aggregation.py` | 区内物体计数（track / DBSCAN）、权重聚合 |
| `pose_stream.py` / `rosbag_ros1.py` | ROS2 bag / ROS1 bag 位姿（依赖 `rosbags`） |
| `scripts/run_semantic_stack.py` | **主入口**：survey JSON + bag + 检测 JSONL → 导航计划 JSON |
| `scripts/inspect_rosbag1.py`、`dump_ros1_poses.py` | ROS1 bag 检视与轨迹导出 |
| `data/survey_zones.example.json` | 含 **coverage.sites_m** 的检测框示例 |
| `data/detections.example.jsonl` | 视觉输出示例（时间 + 类别 + 世界坐标） |
| `data/zones.example.json` | 仅检测框（无覆盖点）示例 |
| `tests/test_smoke.py` | pytest 烟测 |
| `tests/test_survey_geometry.py` | 覆盖区判定单测 |

---

## `src/rbcv_disk_coverage/`（离线 Python 规划包）

| 文件 | 作用 |
|------|------|
| `README.md` | 算法概述、目录结构、安装与快速命令 |

### `rbcv_disk_coverage/coverage_planner/`（Python 子包，导入名 `coverage_planner`）

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

### `rbcv_disk_coverage/scripts/`（命令行工具）

| 文件 | 作用 |
|------|------|
| `README.md` | 工具脚本顺序与命令 |
| `preview_real_map.py` | 打印地图统计并保存自由空间预览 PNG |
| `draw_regions.py` | **交互**：鼠标拖框 → 保存分区 JSON |
| `preview_regions.py` | **校验**：分区 JSON 叠加地图并保存预览 PNG |
| `demo_regions.py` | **端到端**：逐分区自适应覆盖，`分区覆盖结果.png` |

### `rbcv_disk_coverage/docs/`（算法文档）

| 文件 | 作用 |
|------|------|
| `README.md` | 文档索引 |
| `ALGORITHM.md` | DGR、窄通道与可见性推导 |

### `rbcv_disk_coverage/tests/`（单元测试）

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
| `src/rbcv_disk_coverage/scripts/分区叠图预览.png` | `preview_regions.py` |
| `src/rbcv_disk_coverage/scripts/分区覆盖结果.png` | `demo_regions.py` |
| `src/rbcv_disk_coverage/scripts/地图栅格预览.png` | `preview_real_map.py` |

本地新建的 **`my_regions.json`** 建议放在 `src/map_tools/maps/` 并与示例分区路径一致。
