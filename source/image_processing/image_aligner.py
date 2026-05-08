# --------------------------------------------------------------------------------------
# Project: OpenMicroManipulator
# License: MIT (see LICENSE file for full description)
#          All text in here must be included in any redistribution.
# Author:  M. S. (diffraction limited)
# --------------------------------------------------------------------------------------

import cv2
import numpy as np

class ECCImageAligner:
    def __init__(self, gamma = 1.0):
        self.ref_gray = None
        self.warp_mode = cv2.MOTION_EUCLIDEAN  # rotation + translation
        # self.warp_mode = cv2.MOTION_AFFINE  # rotation + translation
        self.gamma=gamma
        self.warp_matrix = None

    def set_reference(self, ref_img):
        """Set the reference image (color or grayscale)."""
        if len(ref_img.shape) == 3:
            self.ref_gray = cv2.cvtColor(ref_img, cv2.COLOR_BGR2GRAY).astype(np.float32)
        else:
            self.ref_gray = ref_img.astype(np.float32)

        self.ref_gray = (self.ref_gray/255)**self.gamma

    def compute_transform(self, input_img, number_of_iterations=300, termination_eps=1e-3, translation_scale_factor=1.0):
        """
        Compute the transform aligning input_img to reference.

        Returns a dict with:
            - rotation (degrees)
            - translation (tx, ty)
            - warp_matrix (2x3 np.array)
            - aligned_image (input_img warped to reference)
            - correlation (ECC value)
        """
        if self.ref_gray is None:
            raise ValueError("Reference image not set. Call set_reference() first.")

        if len(input_img.shape) == 3:
            input_gray = cv2.cvtColor(input_img, cv2.COLOR_BGR2GRAY).astype(np.float32)
        else:
            input_gray = input_img.astype(np.float32)
        input_gray = (input_gray/255)**self.gamma

        if self.warp_matrix is None:
            self.warp_matrix = np.eye(2, 3, dtype=np.float32)
        criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, number_of_iterations, termination_eps)

        try:
            cc, warp_matrix = cv2.findTransformECC(self.ref_gray, input_gray, self.warp_matrix, self.warp_mode, criteria)
            self.warp_matrix = warp_matrix
        except cv2.error as e:
            print(f"ECC alignment failed: {e}")
            #raise RuntimeError(f"ECC alignment failed: {e}")

        if False:
            aligned_img = cv2.warpAffine(input_gray, self.warp_matrix, (self.ref_gray.shape[1], self.ref_gray.shape[0]),
                                         flags=cv2.INTER_LINEAR + cv2.WARP_INVERSE_MAP)
            diff = cv2.resize(aligned_img-self.ref_gray, dsize=None, fx=4, fy=4)
            #diff = cv2.resize(self.ref_gray, dsize=None, fx=4, fy=4)
            cv2.imshow('difference', diff+0.5)

        a = self.warp_matrix[0, 0]
        b = self.warp_matrix[0, 1]
        rotation_rad = np.arctan2(b, a)
        rotation_deg = np.degrees(rotation_rad)
        tx = self.warp_matrix[0, 2]
        ty = self.warp_matrix[1, 2]

        return {
            'rotation': rotation_deg,
            'translation': (tx*translation_scale_factor, ty*translation_scale_factor),
            'warp_matrix': self.warp_matrix,
            # 'aligned_image': aligned_img,
            # 'correlation': cc
        }