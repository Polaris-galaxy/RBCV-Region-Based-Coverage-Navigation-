# map_tools（RBCV 仓库）

**ROS1 catkin 包**：集中存放 **`maps/`**（YAML/PGM/分区 JSON），并提供 Python 桥接与可选 **`/map` 静态发布节点**，与同仓库内的 **`coverage_planner`**（源码目录名为 **`rbcv_disk_coverage/`**）共用地图读写逻辑。

## 目录

| 路径 | 说明 |
|------|------|
| `maps/` | `map7/map8` 及 `regions_map7.example.json` |
| `python/map_tools/` | `paths`、`bootstrap`、`bridge`（生成 `OccupancyGrid`） |
| `scripts/publish_static_map.py` | rospy：从磁盘发布 `nav_msgs/OccupancyGrid` |
| `launch/publish_static_map.launch` | 示例启动 |

## Python（无需 ROS）

在已安装依赖的环境下将 **`src/rbcv_disk_coverage`** 加入 `PYTHONPATH`（或由 `bootstrap` 自动查找），然后：

```python
from map_tools.paths import default_map_yaml
from map_tools.bridge import build_occupancy_grid_msg, load_free_mask_for_planning

yaml = default_map_yaml("map7")
free, info = load_free_mask_for_planning(yaml)  # 与 load_ros_map 相同
msg = build_occupancy_grid_msg(yaml, frame_id="map")  # 需有 nav_msgs（ROS Python）
```

若找不到 ``coverage_planner``，可设置 **`RBCV_DISK_COVERAGE_SRC`**（或旧名 **`COVERAGE_PLANNER_SRC`**）指向 **`.../src/rbcv_disk_coverage`**（其下应有子目录 `coverage_planner/`）。

## ROS1（后续操作）

```bash
# 在工作空间编译后
source devel/setup.bash
roslaunch map_tools publish_static_map.launch
```

分区发布包 **`coverage_regions_ros`** 的 launch 默认从 **`$(find map_tools)/maps/regions_map7.example.json`** 读取分区。
