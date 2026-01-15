# --------------------------------------------------------------------------------------
# Project: OpenMicroManipulator
# License: MIT (see LICENSE file for full description)
#          All text in here must be included in any redistribution.
# Author:  M. S. (diffraction limited)
# --------------------------------------------------------------------------------------

import cv2
from hardware.abstract_camera import AbstractCamera

class OpenCVCamera(AbstractCamera):
    def __init__(self, camera_index=0):
        self.cap = cv2.VideoCapture(camera_index)
        self.grabbing = False

        if not self.cap.isOpened():
            print(f"Failed to open OpenCV camera at index {camera_index}")
            self.cap = None
        else:
            print(f"Using OpenCV camera at index {camera_index}")

    def is_connected(self):
        return self.cap is not None

    def get_exposure_time_range(self):
        # OpenCV doesn't expose this reliably
        return 0, 1

    def set_exposure_time(self, exposure_time_us):
        if self.cap:
            self.cap.set(cv2.CAP_PROP_EXPOSURE, float(exposure_time_us) / 1e6)

    def start_grabbing(self, single_grab=True):
        self.grabbing = True  # OpenCV doesn't require special start

    def stop_grabbing(self):
        self.grabbing = False

    def grab_single_triggered(self, timeout_ms=1000):
        # No triggering in OpenCV, just grab a frame
        return self.grab_one(timeout_ms)

    def grab_one(self, timeout_ms=5000):
        if not self.cap:
            return None
        ret, frame = self.cap.read()
        return frame if ret else None

    def grab_loop(self, callback, timeout_ms=5000):
        if not self.cap:
            return

        self.start_grabbing(single_grab=False)
        try:
            while self.grabbing:
                ret, frame = self.cap.read()
                if not ret:
                    print("Frame grab failed")
                    continue

                try:
                    if callback(frame) is False:
                        break
                except Exception as e:
                    print(f"Error in callback: {e}")
        finally:
            self.stop_grabbing()

    def __del__(self):
        if self.cap:
            self.cap.release()
