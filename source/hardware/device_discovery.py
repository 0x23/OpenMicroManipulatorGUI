from hardware.camera_basler import BaslerCamera
from hardware.camera_opencv import OpenCVCamera
from hardware.open_micro_stage_api import OpenMicroStageInterface


def list_serial_devices():
    devices = OpenMicroStageInterface.enumerate_devices()
    for d in devices:
        d["label"] = d["label"].removesuffix(" - Pico Serial")
    return devices


def list_camera_devices(max_opencv_indices=8):
    devices = []
    devices.extend(BaslerCamera.enumerate_devices())
    devices.extend(OpenCVCamera.enumerate_devices(max_indices=max_opencv_indices))
    return devices
