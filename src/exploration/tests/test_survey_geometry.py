from __future__ import annotations

import sys
from pathlib import Path

_SRC = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_SRC))

from exploration.config import DetectionZone  # noqa: E402
from exploration.survey.coverage_geometry import robot_in_coverage_area  # noqa: E402
from exploration.survey.zone_catalog import CoverageSpec, SurveyZone  # noqa: E402


def test_robot_must_be_near_site_when_sites_defined() -> None:
    z = DetectionZone(zone_id="A", bbox_xyxy=(0, 0, 2, 2))
    row = SurveyZone(zone_id="A", zone=z, coverage=CoverageSpec(radius_m=0.5, sites_m=[(1.0, 1.0)]))
    assert robot_in_coverage_area(1.0, 1.0, row) is True
    assert robot_in_coverage_area(0.1, 0.1, row) is False


def test_empty_sites_means_full_footprint() -> None:
    z = DetectionZone(zone_id="B", bbox_xyxy=(0, 0, 1, 1))
    row = SurveyZone(zone_id="B", zone=z, coverage=CoverageSpec(radius_m=1.0, sites_m=[]))
    assert robot_in_coverage_area(0.2, 0.3, row) is True
