# scripts

| 脚本 | 类型 | 说明 |
|------|------|------|
| `partition_labels_publisher.py` | rospy 节点 | 订阅 `~map_topic`，发布 `~partition_topic`（`OccupancyGrid`，`-1`=区外，`1..127`=分区序号）；可选 RViz `MarkerArray` |

安装后：`rosrun coverage_regions_ros partition_labels_publisher.py`

参数详见脚本顶部注释（`~regions_json` 必填）。
