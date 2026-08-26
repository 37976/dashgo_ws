import cv2
import numpy as np


def get_distance_transform(map_image, min_distance, map_resolution=0.05,
                           threshold_px=3, known_free_mask=None):
    distance_px = min_distance / map_resolution

    if known_free_mask is None:
        _, binary_image = cv2.threshold(map_image, 150, 255, cv2.THRESH_BINARY)
    else:
        binary_image = np.where(known_free_mask, 255, 0).astype(np.uint8)
    distance_transform = cv2.distanceTransform(binary_image, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)

    # Create a copy of distance transform
    distance_transform_copy = distance_transform.copy()

    # Convert distance transform copy to RGB
    distance_transform_rgb = cv2.cvtColor(distance_transform_copy, cv2.COLOR_GRAY2RGB)

    # Set pixels near distance_px to red
    red_color = (255, 0, 0)
    near_pixels = np.abs(distance_transform_copy - distance_px) < threshold_px
    distance_transform_rgb[near_pixels] = red_color

    return distance_transform_rgb


