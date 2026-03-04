#!/usr/bin/env python3
"""Color mask utility for detecting colored objects using HSV thresholding."""

import cv2
import numpy as np


def color_mask(color_img, color_name='r'):
    """Returns a binary mask for the specified color.

    Parameters
    ----------
    color_img : np.ndarray
        Input image (RGB or BGR format)
    color_name : string
        Color to mask: 'r'/'red', 'g'/'green', 'b'/'blue', 'y'/'yellow'

    Returns
    -------
    mask : np.ndarray
        Binary mask (255 where color is detected, 0 elsewhere)
    """
    # Normalize color name
    color = color_name.lower().strip()
    if color in ['r', 'red']:
        color = 'red'
    elif color in ['g', 'green']:
        color = 'green'
    elif color in ['b', 'blue']:
        color = 'blue'
    elif color in ['y', 'yellow']:
        color = 'yellow'

    # Convert to BGR if needed, then to HSV
    if color_img.shape[-1] == 3:
        bgr_img = cv2.cvtColor(color_img, cv2.COLOR_RGB2BGR)
    else:
        bgr_img = color_img

    hsv_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)
    mask = None

    if color == 'red':
        # Red wraps around 0 and 180
        lower1 = np.array([0, 70, 50])
        upper1 = np.array([10, 255, 255])
        lower2 = np.array([170, 70, 50])
        upper2 = np.array([180, 255, 255])
        mask = cv2.bitwise_or(
            cv2.inRange(hsv_img, lower1, upper1),
            cv2.inRange(hsv_img, lower2, upper2)
        )
    elif color == 'green':
        lower = np.array([40, 70, 50])
        upper = np.array([80, 255, 255])
        mask = cv2.inRange(hsv_img, lower, upper)
    elif color == 'blue':
        lower = np.array([90, 50, 40])
        upper = np.array([130, 255, 255])
        mask = cv2.inRange(hsv_img, lower, upper)
    elif color == 'yellow':
        lower = np.array([15, 40, 100])
        upper = np.array([35, 255, 255])
        mask = cv2.inRange(hsv_img, lower, upper)

    if mask is not None:
        # Morphological cleanup
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return mask

    return np.zeros(color_img.shape[:2], dtype=np.uint8)
