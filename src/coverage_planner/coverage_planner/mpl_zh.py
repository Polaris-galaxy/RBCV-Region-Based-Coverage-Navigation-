"""Configure matplotlib fonts so Chinese/UI glyphs render correctly."""

from __future__ import annotations

import logging
import os
import platform
from pathlib import Path

LOGGER = logging.getLogger(__name__)

# Matplotlib 里 ``family='sans-serif'`` 会走 fontconfig 解析，``-`` 会触发 ParseException。
_GENERIC_FAMILIES = frozenset(
    {"sans-serif", "serif", "monospace", "cursive", "fantasy", "default"}
)


def configure_matplotlib_chinese_font() -> tuple[str | None, str | None]:
    """注册/选择可显示中文的字体。

    Returns:
        ``(display_name, font_path)``。若得到 ``font_path``，请用
        ``FontProperties(fname=font_path)``（最稳）；否则可用 ``family=display_name``，
        但 **不要** 把 ``sans-serif`` 等泛族名传给 ``FontProperties(family=...)``。
    """
    import matplotlib
    from matplotlib import font_manager as fm

    matplotlib.rcParams["axes.unicode_minus"] = False

    if platform.system() != "Windows":
        matplotlib.rcParams["font.sans-serif"] = [
            "PingFang SC",
            "Hiragino Sans GB",
            "Noto Sans CJK SC",
            "Microsoft YaHei",
            "WenQuanYi Zen Hei",
            "SimHei",
            "DejaVu Sans",
        ]
        matplotlib.rcParams["font.family"] = "sans-serif"
        return matplotlib.rcParams["font.sans-serif"][0], None

    fonts_dir = Path(os.environ.get("WINDIR", r"C:\Windows")) / "Fonts"
    ordered = ("msyh.ttc", "msyhbd.ttc", "simhei.ttf", "simsun.ttc")

    for fname in ordered:
        fpath = fonts_dir / fname
        if not fpath.is_file():
            continue
        try:
            fm.fontManager.addfont(str(fpath))
            prop = fm.FontProperties(fname=str(fpath))
            # get_family() 常常是泛族名 ``sans-serif``；应用具体名应用 get_name()
            name = str(prop.get_name() or "").strip()
            if not name or name.lower() in _GENERIC_FAMILIES:
                if "msyh" in fname.lower():
                    name = "Microsoft YaHei"
                elif "simhei" in fname.lower():
                    name = "SimHei"
                elif "simsun" in fname.lower():
                    name = "SimSun"
                else:
                    name = "DejaVu Sans"

            matplotlib.rcParams["font.sans-serif"] = [
                name,
                "Microsoft YaHei",
                "SimHei",
                "DejaVu Sans",
            ]
            matplotlib.rcParams["font.family"] = "sans-serif"
            return name, str(fpath.resolve())
        except Exception as exc:
            LOGGER.warning("matplotlib: failed addfont(%s): %s", fpath, exc)

    matplotlib.rcParams["font.sans-serif"] = [
        "Microsoft YaHei",
        "SimHei",
        "DejaVu Sans",
    ]
    matplotlib.rcParams["font.family"] = "sans-serif"
    return "Microsoft YaHei", None
