"""Regression cases for odometry after signed 16-bit encoder rollover."""
import pytest

from dashgo_driver_ros2.dashgo_driver_node import encoder_delta


@pytest.mark.parametrize('samples, expected', [
    ([32760, -32766, -32756, -32756, -32746], [10, 10, 0, 10]),
    ([-32760, 32766, 32756, 32756, 32746], [-10, -10, 0, -10]),
    ([32760, -32766, 32760, 32750], [10, -10, -10]),
    ([10, 0, -10, 0], [-10, -10, 10]),
])
def test_rollover_only_affects_crossing_interval(samples, expected):
    deltas = [encoder_delta(b, a, -32768, 32768)
              for a, b in zip(samples, samples[1:])]
    assert deltas == expected


@pytest.mark.parametrize('step', [100, -100])
def test_multiple_rollovers_preserve_distance(step):
    unwrapped = list(range(0, step * 2000, step))
    samples = [(value + 32768) % 65536 - 32768 for value in unwrapped]
    deltas = [encoder_delta(b, a, -32768, 32768)
              for a, b in zip(samples, samples[1:])]
    assert all(delta == step for delta in deltas)
    assert sum(deltas) == unwrapped[-1] - unwrapped[0]
