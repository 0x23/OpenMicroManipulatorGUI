import sys

import cv2
from serial.tools import list_ports

from hardware.camera_basler import BaslerCamera


def list_serial_devices():
    devices = []

    for port in sorted(list_ports.comports(), key=lambda item: item.device):
        description = (port.description or "").strip()
        label = port.device
        if description and description.lower() != "n/a" and description != port.device:
            label = f"{port.device} - {description}"

        devices.append({
            "id": port.device,
            "label": label,
            "port": port.device,
        })

    return devices


def _opencv_backends():
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


def list_camera_devices(max_opencv_indices=8):
    devices = []
    devices.extend(BaslerCamera.enumerate_devices())

    for index in range(max_opencv_indices):
        for backend, backend_label in _opencv_backends():
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
