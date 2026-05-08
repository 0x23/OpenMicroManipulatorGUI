# --------------------------------------------------------------------------------------
# Project: OpenMicroManipulator
# License: MIT (see LICENSE file for full description)
#          All text in here must be included in any redistribution.
# Author:  M. S. (diffraction limited)
# --------------------------------------------------------------------------------------

import sys

import cv2
import numpy as np

from hardware.abstract_camera import AbstractCamera


class OpenCVCamera(AbstractCamera):
    @staticmethod
    def _enumeration_backends():
        if sys.platform.startswith("win"):
            return [
                (cv2.CAP_DSHOW, "DirectShow"),
                (cv2.CAP_MSMF, "MSMF"),
                (cv2.CAP_ANY, "Auto"),
            ]

        if sys.platform == "darwin":
            return [
                (cv2.CAP_AVFOUNDATION, "AVFoundation"),
                (cv2.CAP_ANY, "Auto"),
            ]

        return [
            (cv2.CAP_V4L2, "V4L2"),
            (cv2.CAP_ANY, "Auto"),
        ]

    @staticmethod
    def enumerate_devices(max_indices=8):
        devices = []

        for index in range(max_indices):
            for backend, backend_label in OpenCVCamera._enumeration_backends():
                capture = None
                try:
                    capture = cv2.VideoCapture(index, backend)
                    if not capture.isOpened():
                        continue

                    ok, _ = capture.read()
                    if not ok:
                        continue

                    devices.append({
                        "kind": "opencv",
                        "id": f"opencv:{index}:{backend}",
                        "label": f"OpenCV Camera {index}",
                        "index": index,
                        "backend": backend,
                        "backend_label": backend_label,
                    })
                    break
                except Exception:
                    continue
                finally:
                    if capture is not None:
                        capture.release()

        return devices

    def __init__(self, camera_index=0, backend=cv2.CAP_ANY, resolution=None):
        self.camera_index = int(camera_index)
        self.backend = backend
        self.cap = cv2.VideoCapture(self.camera_index, self.backend)
        self.grabbing = False

        if not self.cap.isOpened():
            print(f"Failed to open OpenCV camera at index {self.camera_index}")
            self.cap.release()
            self.cap = None
            return

        print(f"Using OpenCV camera at index {self.camera_index}")

        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        if resolution is not None:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])

        self.cap.set(cv2.CAP_PROP_AUTO_WB, 0)
        self.cap.set(cv2.CAP_PROP_WB_TEMPERATURE, 4100)

    @staticmethod
    def _convert_frame(frame):
        if frame is None:
            return None

        if len(frame.shape) == 3 and frame.shape[2] == 3:
            return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        return frame

    def is_connected(self):
        return self.cap is not None

    def set_gain(self, gain: float):
        if self.cap:
            self.cap.set(cv2.CAP_PROP_GAIN, gain)

    def set_white_balance(self, wb: float):
        if self.cap:
            self.cap.set(cv2.CAP_PROP_WB_TEMPERATURE, int(wb))

    def get_exposure_time_range(self):
        return 0, 1

    def set_exposure_time(self, exposure_time_us):
        if self.cap:
            self.cap.set(cv2.CAP_PROP_EXPOSURE, float(exposure_time_us))

    def start_grabbing(self, single_grab=True):
        self.grabbing = True

    def stop_grabbing(self):
        self.grabbing = False

    def grab_single_triggered(self, timeout_ms=1000):
        return self.grab_one(timeout_ms)

    def grab_one(self, timeout_ms=5000):
        if not self.cap:
            return None

        ret, frame = self.cap.read()
        return self._convert_frame(frame) if ret else None

    def grab_loop(self, callback, timeout_ms=5000):
        if not self.cap:
            return

        frame = np.ones((100, 100, 3), dtype=np.uint8) * 60
        self.start_grabbing(single_grab=False)
        try:
            while self.grabbing:
                new_frame = self.grab_one(timeout_ms)
                if new_frame is not None:
                    frame = new_frame

                try:
                    if callback(frame) is False:
                        break
                except Exception as exc:
                    print(f"Error in callback: {exc}")
        finally:
            self.stop_grabbing()

    def close(self):
        self.stop_grabbing()
        if self.cap:
            self.cap.release()
            self.cap = None

    def __del__(self):
        self.close()
