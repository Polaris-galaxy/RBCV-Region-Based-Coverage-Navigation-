"""加载真实 ROS 地图并保存预览图.

用法:
    py examples/preview_real_map.py ../map_tools/maps/map7.yaml
"""
from __future__ import annotations

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
MAP_TOOLS_MAPS = os.path.abspath(os.path.join(ROOT, "..", "map_tools", "maps"))
sys.path.insert(0, ROOT)

import numpy as np  # noqa: E402

from coverage_planner.map_io import load_ros_map  # noqa: E402


def main(yaml_path: str) -> None:
    free, info = load_ros_map(yaml_path)
    h, w = free.shape
    free_count = int(free.sum())
    free_ratio = free_count / (h * w)
    print(
        f"image       = {info.image_path}\n"
        f"resolution  = {info.resolution} m/px\n"
        f"shape       = ({h}, {w}) -> {h*info.resolution:.1f} x {w*info.resolution:.1f} m\n"
        f"free pixels = {free_count} ({free_ratio*100:.1f}%)\n"
        f"occ_thresh  = {info.occupied_thresh}\n"
        f"free_thresh = {info.free_thresh}"
    )

    img = (free.astype(np.uint8) * 255)
    out_path = os.path.join(HERE, "real_map_preview.png")
    try:
        import imageio.v3 as iio

        iio.imwrite(out_path, img)
    except Exception:
        from PIL import Image

        Image.fromarray(img).save(out_path)
    print(f"preview saved -> {out_path}")


if __name__ == "__main__":
    yaml_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        MAP_TOOLS_MAPS, "map7.yaml"
    )
    main(yaml_path)
