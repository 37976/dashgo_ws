import math

from dashgo_xfeat_bringup.odom_fusion_node import (
    _imu_delta_is_consistent,
    _wrap_angle,
)


def test_imu_delta_wraps_across_pi():
    previous = math.radians(179.0)
    current = math.radians(-179.0)

    assert math.isclose(_wrap_angle(current - previous), math.radians(2.0))


def test_imu_delta_consistency_accepts_small_encoder_difference():
    assert _imu_delta_is_consistent(
        math.radians(5.2), math.radians(5.0), math.radians(2.0))


def test_imu_delta_consistency_rejects_scaled_or_reversed_measurement():
    assert not _imu_delta_is_consistent(
        math.radians(25.0), math.radians(5.0), math.radians(5.0))
    assert not _imu_delta_is_consistent(
        math.radians(-5.0), math.radians(5.0), math.radians(5.0))
