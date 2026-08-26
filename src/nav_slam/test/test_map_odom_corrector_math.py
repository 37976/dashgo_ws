"""Tests for map-to-odom correction innovation gates."""

import math

import pytest

from nav_slam.map_odom_corrector import (
    _correction_innovation,
    _tracking_innovation_allowed,
)


def test_correction_innovation_wraps_yaw_and_uses_translation_norm():
    translation, yaw = _correction_innovation(
        (1.0, 2.0, math.radians(179.0)),
        (1.3, 2.4, math.radians(-179.0)),
    )

    assert translation == pytest.approx(0.5)
    assert math.degrees(yaw) == pytest.approx(2.0)


def test_tracking_innovation_gate_rejects_symmetric_map_alias():
    reference = (3.916126, -1.389290, math.radians(-159.486645))
    false_candidates = [
        (9.926, 7.536, math.radians(20.961)),
        (10.087068, 7.584735, math.radians(20.936848)),
    ]

    for false_candidate in false_candidates:
        assert not _tracking_innovation_allowed(
            reference, false_candidate, 0.50, math.radians(5.0)
        )


def test_tracking_innovation_gate_keeps_normal_correction():
    reference = (3.916126, -1.389290, math.radians(-159.486645))
    normal_candidate = (4.113649, -1.376752, math.radians(-159.132553))

    assert _tracking_innovation_allowed(
        reference, normal_candidate, 0.50, math.radians(5.0)
    )
