# --------------------------------------------------------------------------------------
# Project: OpenMicroManipulator
# License: MIT (see LICENSE file for full description)
#          All text in here must be included in any redistribution.
# Author:  M. S. (diffraction limited)
# --------------------------------------------------------------------------------------

import cv2
import numpy as np

class ImagePointTracker:
    def __init__(self, patch_size=50, search_radius=20):
        self.patch_size = patch_size
        self.search_radius = search_radius
        self.template = None
        self.prev_img = None
        self.prev_pos = (0,0)

    def reset(self):
        self.prev_pos = (0,0)
        self.template = None

    def set_track_point(self, img, x, y):
        """
        Set the initial point to track and extract the template patch.
        """
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        x, y = int(x), int(y)
        half = self.patch_size // 2

        # Store patch and position
        self.template = gray[y - half:y + half + 1, x - half:x + half + 1].copy()
        self.prev_img = gray
        self.prev_pos = (x, y)

    def update(self, img):
        """
        Track the point in the new image using local template matching.
        :return: (x, y) of tracked point or None if lost
        """
        if self.template is None or self.prev_pos is None:
            return 0, 0

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        x0, y0 = self.prev_pos
        r = self.search_radius
        h = self.patch_size // 2

        # Extract search window around previous point
        x1, y1 = x0 - r - h, y0 - r - h
        x2, y2 = x0 + r + h + 1, y0 + r + h + 1

        if x1 < 0 or y1 < 0 or x2 > gray.shape[1] or y2 > gray.shape[0]:
            return self.prev_pos  # Out of bounds

        search_area = gray[y1:y2, x1:x2]

        # Run template matching
        res = cv2.matchTemplate(search_area, self.template, cv2.TM_CCOEFF_NORMED)
        _, max_val, _, max_loc = cv2.minMaxLoc(res)

        if max_val < 0.2:
            return self.prev_pos  # Low confidence

        dx, dy = max_loc
        new_x = x1 + dx + h
        new_y = y1 + dy + h

        self.prev_pos = (new_x, new_y)
        return new_x, new_y
