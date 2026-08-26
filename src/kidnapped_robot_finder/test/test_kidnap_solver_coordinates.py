import numpy as np

from global_localizer.kidnap_solver import (
    _get_candidates,
    _limit_candidates_to_radius,
    _occupancy_masks,
    _map_pixel_to_world,
    _map_world_to_pixel,
    _select_stratified_candidates,
)


def test_map_pixel_to_world_uses_pixel_centers():
    world = _map_pixel_to_world(
        pixel_uv=(0.0, 0.0),
        map_height_px=100,
        map_resolution=0.05,
        map_origin=(-2.0, -3.0),
    )

    np.testing.assert_allclose(world, (-1.975, 1.975))


def test_map_pixel_to_world_preserves_subpixel_refinement():
    world = _map_pixel_to_world(
        pixel_uv=(39.25, 69.75),
        map_height_px=100,
        map_resolution=0.05,
        map_origin=(-2.0, -3.0),
    )

    np.testing.assert_allclose(world, (-0.0125, -1.5125))


def test_map_pixel_to_world_applies_map_specific_calibration():
    world = _map_pixel_to_world(
        pixel_uv=(0.0, 0.0),
        map_height_px=100,
        map_resolution=0.05,
        map_origin=(-2.0, -3.0),
        map_pose_offset=(0.0486, -0.0639),
    )

    np.testing.assert_allclose(world, (-1.9264, 1.9111))


def test_map_world_to_pixel_round_trip():
    pixel = np.array([39.25, 69.75])
    world = _map_pixel_to_world(
        pixel, 100, 0.05, (-2.0, -3.0), (0.0486, -0.0639))

    recovered = _map_world_to_pixel(
        world, 100, 0.05, (-2.0, -3.0), (0.0486, -0.0639))

    np.testing.assert_allclose(recovered, pixel)


def test_stratified_candidates_are_deterministic_and_cover_map():
    rows, cols = np.indices((100, 120))
    red_pixels = rows.ravel(), cols.ravel()

    first = _select_stratified_candidates(red_pixels, 24)
    second = _select_stratified_candidates(red_pixels, 24)

    assert first == second
    assert len(first) == 24
    selected_rows = np.array([row for row, _ in first])
    selected_cols = np.array([col for _, col in first])
    assert selected_rows.min() < 20
    assert selected_rows.max() > 80
    assert selected_cols.min() < 20
    assert selected_cols.max() > 100


def test_local_radius_limits_candidate_positions():
    rows, cols = np.indices((21, 21))
    limited_rows, limited_cols = _limit_candidates_to_radius(
        (rows.ravel(), cols.ravel()), center_pixel_uv=(10.0, 10.0), radius_px=3.0)

    distances = np.hypot(limited_cols - 10.0, limited_rows - 10.0)
    assert limited_rows.size > 0
    assert np.all(distances <= 3.0)
    assert limited_rows.size < rows.size


def test_unknown_map_pixels_are_not_free_or_occupied():
    image = np.array([[0, 89, 205, 206, 254, 255]], dtype=np.uint8)

    known_free, occupied = _occupancy_masks(
        image, free_thresh=0.196, occupied_thresh=0.65, negate=0)

    np.testing.assert_array_equal(
        known_free, [[False, False, False, True, True, True]])
    np.testing.assert_array_equal(
        occupied, [[True, True, False, False, False, False]])


def test_candidate_fallback_excludes_unknown_pixels():
    image = np.full((5, 5), 205, dtype=np.uint8)
    image[2, 2] = 254
    known_free, _ = _occupancy_masks(image)

    _, candidates, count, threshold = _get_candidates(
        image, min_distance=0.25, map_resolution=0.05,
        min_required=100, known_free_mask=known_free)

    assert threshold == -1
    assert count == 1
    assert list(zip(candidates[0], candidates[1])) == [(2, 2)]
