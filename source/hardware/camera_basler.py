# --------------------------------------------------------------------------------------
# Project: OpenMicroManipulator
# License: MIT (see LICENSE file for full description)
#          All text in here must be included in any redistribution.
# Author:  M. S. (diffraction limited)
# --------------------------------------------------------------------------------------

from hardware.abstract_camera import AbstractCamera
import numpy as np

try:
    from pypylon import pylon
except ImportError:
    pylon = None

class BaslerCamera(AbstractCamera):
    def __init__(self, device_serial=None):
        self.camera = None
        self.connected = False
        self.grabbing = False
        self.device_serial = device_serial
        self.converter = None

        if pylon is None:
            print("Basler camera support unavailable: pypylon is not installed")
            return

        try:
            factory = pylon.TlFactory.GetInstance()
            device = None
            if self.device_serial:
                for candidate in factory.EnumerateDevices():
                    if candidate.GetSerialNumber() == self.device_serial:
                        device = factory.CreateDevice(candidate)
                        break
            else:
                device = factory.CreateFirstDevice()

            if device is None:
                raise RuntimeError(f"Basler camera with serial '{self.device_serial}' not found")

            self.camera = pylon.InstantCamera(device)
            self.camera.Open()
            self.set_exposure_time(16000.0)
            self.camera.Gain.SetValue(0.0)
            info = self.camera.GetDeviceInfo()
            print("Using camera:", info.GetModelName(), info.GetSerialNumber())

            self.converter = pylon.ImageFormatConverter()
            self.converter.OutputPixelFormat = pylon.PixelType_RGB8packed
            self.converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

            self.connected = True
        except Exception as e:
            print(f"Camera initialization failed: {e}")
            self.close()

    @staticmethod
    def is_available():
        return pylon is not None

    @staticmethod
    def enumerate_devices():
        if pylon is None:
            return []

        devices = []
        try:
            for candidate in pylon.TlFactory.GetInstance().EnumerateDevices():
                serial_number = candidate.GetSerialNumber()
                model_name = candidate.GetModelName() or "Basler Camera"
                display_name = f"Basler {model_name}"
                if serial_number:
                    display_name = f"{display_name} ({serial_number})"
                devices.append({
                    "kind": "basler",
                    "id": serial_number,
                    "label": display_name,
                })
        except Exception as e:
            print(f"Basler enumeration failed: {e}")

        return devices

    def is_connected(self):
        return self.connected

    def get_exposure_time_range(self):
        if not self.camera:
            return 0, 1
        try:
            return (
                self.camera.ExposureTime.GetMin(),
                self.camera.ExposureTime.GetMax()
            )
        except Exception as e:
            print(f"Failed to get exposure time range: {e}")
            return None

    def set_exposure_time(self, exposure_time_us):
        if self.camera:
            self.camera.ExposureTime.SetValue(exposure_time_us)

    def set_gain(self, gain_db: float):
        if self.camera:
            self.camera.Gain.SetValue(gain_db)

    def start_grabbing(self, single_grab=True):
        if not self.camera or self.grabbing:
            return

        self.camera.StopGrabbing()
        if single_grab:
            self.camera.TriggerSelector.SetValue("FrameStart")
            self.camera.TriggerMode.SetValue("On")
            self.camera.TriggerSource.SetValue("Software")
        else:
            self.camera.TriggerMode.SetValue("Off")

        self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        self.grabbing = True

    def stop_grabbing(self):
        if self.camera and self.grabbing:
            self.camera.StopGrabbing()
            self.grabbing = False

    def grab_single_triggered(self, timeout_ms=1000):
        """
        Grab a single frame using software trigger after start_grabbing().
        Returns:
            np.ndarray or None
        """
        if not self.camera:
            return None

        try:
            self.camera.TriggerSoftware.Execute()
            result = self.camera.RetrieveResult(timeout_ms, pylon.TimeoutHandling_ThrowException)
            if result.GrabSucceeded():
                img = self.converter.Convert(result).GetArray()
                result.Release()
                return img
            result.Release()
        except Exception as e:
            print(f"Grab failed: {e}")
        return None

    def grab_one(self, timeout_ms=5000):
        """
        Grab a single image (blocking).
        Returns:
            np.ndarray or None
        """
        if not self.camera:
            return None

        try:
            result = self.camera.GrabOne(timeout_ms, pylon.TimeoutHandling_ThrowException)
            if result.GrabSucceeded():
                img = self.converter.Convert(result).GetArray()
                result.Release()
                return img
            result.Release()
        except Exception as e:
            print(f"Grab failed: {e}")
        return None

    def grab_loop(self, callback, timeout_ms=5000):
        """
        Continuously grabs images, calls callback(image) per frame.
        Stops if callback returns False.
        """
        if not self.camera:
            return

        self.start_grabbing(single_grab=False)
        try:
            while self.camera.IsGrabbing():
                result = self.camera.RetrieveResult(timeout_ms, pylon.TimeoutHandling_ThrowException)
                if result.GrabSucceeded():
                    img = self.converter.Convert(result).GetArray()
                    result.Release()
                    if callback(img) is False:
                        break
                else:
                    result.Release()
                    print("Grab failed")
        except Exception as e:
            print(f"Grab loop error: {e}")
        finally:
            self.stop_grabbing()

    def close(self):
        self.stop_grabbing()

        if self.camera:
            try:
                if self.camera.IsOpen():
                    self.camera.Close()
            except Exception:
                pass

        self.camera = None
        self.converter = None
        self.connected = False
