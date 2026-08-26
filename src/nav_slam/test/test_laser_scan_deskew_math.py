import math

from nav_slam.laser_scan_to_points import _interpolate_quaternion


def test_quaternion_interpolation_uses_shortest_yaw_path():
    start = (0.0, 0.0, math.sin(math.radians(170.0) / 2.0),
             math.cos(math.radians(170.0) / 2.0))
    end = (0.0, 0.0, math.sin(math.radians(-170.0) / 2.0),
           math.cos(math.radians(-170.0) / 2.0))

    midpoint = _interpolate_quaternion(start, end, 0.5)
    midpoint_yaw = math.degrees(2.0 * math.atan2(midpoint[2], midpoint[3]))

    assert math.isclose(abs(midpoint_yaw), 180.0, abs_tol=1e-6)


def test_quaternion_interpolation_preserves_unit_length():
    start = (0.0, 0.0, 0.0, 1.0)
    end = (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5))

    result = _interpolate_quaternion(start, end, 0.35)

    assert math.isclose(sum(value * value for value in result), 1.0)
