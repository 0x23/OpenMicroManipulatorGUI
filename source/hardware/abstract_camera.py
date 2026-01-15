# --------------------------------------------------------------------------------------
# Project: OpenMicroManipulator
# License: MIT (see LICENSE file for full description)
#          All text in here must be included in any redistribution.
# Author:  M. S. (diffraction limited)
# --------------------------------------------------------------------------------------

from abc import ABC, abstractmethod
import numpy as np

class AbstractCamera(ABC):
    @abstractmethod
    def get_exposure_time_range(self) -> tuple[float]:
        """Set the camera's exposure time."""
        pass

    @abstractmethod
    def set_exposure_time(self, exposure_time_us: float):
        """Set the camera's exposure time."""
        pass

    @abstractmethod
    def start_grabbing(self, single_grab: bool = True):
        """Start grabbing images, optionally in single trigger mode."""
        pass

    @abstractmethod
    def stop_grabbing(self):
        """Stop grabbing images."""
        pass

    @abstractmethod
    def grab_single_triggered(self, timeout_ms: int = 1000) -> np.ndarray:
        """Grab a single image using software trigger."""
        pass

    @abstractmethod
    def grab_one(self, timeout_ms: int = 5000) -> np.ndarray:
        """Grab a single image (blocking)."""
        pass

    @abstractmethod
    def grab_loop(self, callback, timeout_ms: int = 5000):
        """
        Continuously grab images and call the callback function for each frame.
        The callback must accept one argument (the image as np.ndarray) and
        may return False to stop the loop.
        """
        pass

    @abstractmethod
    def is_connected(self) -> bool:
        """
        Check if the camera is connected and ready to grab images
        """
        pass
