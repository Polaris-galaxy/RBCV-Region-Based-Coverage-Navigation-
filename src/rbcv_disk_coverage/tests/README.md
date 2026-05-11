# tests

单元测试（`pytest`）。

```bash
cd src/rbcv_disk_coverage
py -m pytest tests/ -q
```

| 文件 | 覆盖对象 |
|------|----------|
| `test_set_cover.py` | Set Cover |
| `test_map_io_occupancy.py` | `load_ros_map_occupancy` + `map_tools/maps` 示例 |
| `test_region_planner.py` | 分区分类与逐区规划 smoke |
