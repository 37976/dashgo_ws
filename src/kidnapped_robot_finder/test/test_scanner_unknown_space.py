import numpy as np

from global_localizer.kidnap_solver import _occupancy_masks
from global_localizer.scanner_simulator import compute_dt_map, get_lidar_points


def test_simulated_scan_does_not_cross_unknown_space():
    image = np.full((21, 21), 254, dtype=np.uint8)
    image[:, 12] = 205
    image[:, 16] = 0
    known_free, occupied = _occupancy_masks(image)

    points = get_lidar_points(
        image, 5, 10, add_noise=False,
        dt_map=compute_dt_map(image, known_free_mask=known_free),
        known_free_mask=known_free,
        occupied_mask=occupied,
    )

    assert not points
