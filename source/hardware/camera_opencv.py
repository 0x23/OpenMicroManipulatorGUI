# --------------------------------------------------------------------------------------
# Project: OpenMicroManipulator
# License: MIT (see LICENSE file for full description)
#          All text in here must be included in any redistribution.
# Author:  M. S. (diffraction limited)
# --------------------------------------------------------------------------------------

import cv2
import numpy as np

from hardware.abstract_camera import AbstractCamera

class OpenCVCamera(AbstractCamera):
    def __init__(self, camera_index=0, resolution=None):
        self.cap = cv2.VideoCapture(camera_index)
        self.grabbing = False

        if not self.cap.isOpened():
            print(f"Failed to open OpenCV camera at index {camera_index}")
            self.cap = None
            return
        else:
            print(f"Using OpenCV camera at index {camera_index}")

        # Force MJPEG
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        fourcc = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        codec = "".join([chr((fourcc >> 8 * i) & 0xFF) for i in range(4)])
        print("Codec:", codec)
        print("Auto WB:", self.cap.get(cv2.CAP_PROP_AUTO_WB))
        print("Blue U:", self.cap.get(cv2.CAP_PROP_WHITE_BALANCE_BLUE_U))
        print("Red V:", self.cap.get(cv2.CAP_PROP_WHITE_BALANCE_RED_V))
        print("Red V:", self.cap.get(cv2.CAP_PROP_WHITE_BALANCE_RED_V))

        # in linux you can do: v4l2-ctl -d /dev/video4 --list-ctrls

        # Set resolution
        if resolution is not None:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])

        # Disable auto white balance
        self.cap.set(cv2.CAP_PROP_AUTO_WB, 0)
        # Set manual white balance (in OpenCV, sometimes B channel)
        #self.cap.set(cv2.CAP_PROP_WHITE_BALANCE_BLUE_U, 2800)
        #self.cap.set(cv2.CAP_PROP_WHITE_BALANCE_RED_V, 2800)
        self.cap.set(cv2.CAP_PROP_WB_TEMPERATURE, 4100)

    def is_connected(self):
        return self.cap is not None

    def set_gain(self, gain: float):
        if self.cap:
            self.cap.set(cv2.CAP_PROP_GAIN, gain)

    def set_white_balance(self, wb: float):
        """Set the camera's white balance if supported"""
        self.cap.set(cv2.CAP_PROP_WB_TEMPERATURE, int(wb))

    def get_exposure_time_range(self):
        # OpenCV doesn't expose this reliably
        return 0, 1

    def set_exposure_time(self, exposure_time_us):
        if self.cap:
            self.cap.set(cv2.CAP_PROP_EXPOSURE, float(exposure_time_us))

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

        frame = np.ones((100, 100, 3), dtype=np.uint8)*60
        self.start_grabbing(single_grab=False)
        try:
            while self.grabbing:
                ret, new_frame = self.cap.read()
                if not ret:
                    pass
                    #print("Frame grab failed")
                elif new_frame.shape[2] == 3:
                    frame = cv2.cvtColor(new_frame, cv2.COLOR_BGR2RGB)
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
