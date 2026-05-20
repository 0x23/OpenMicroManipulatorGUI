# --------------------------------------------------------------------------------------
# Project: OpenMicroManipulator
# License: MIT (see LICENSE file for full description)
#          All text in here must be included in any redistribution.
# Author:  M. S. (diffraction limited)
# --------------------------------------------------------------------------------------

import os
import time

from PySide6.QtWidgets import (
    QWidget,
    QApplication
)
from PySide6.QtCore import Qt, QObject, QEvent, QPoint, Signal, QMutex, QThread
from PySide6.QtGui import QCursor, QMouseEvent
from PySide6.QtUiTools import loadUiType
from hardware.open_micro_stage_api import OpenMicroStageInterface
import numpy as np

_ui_path = os.path.join(os.path.dirname(__file__), "realtime_controller_widget.ui")
Ui_RealtimeControllerWidget, _ = loadUiType(_ui_path)


class UpdateWorker(QThread):
    pose_changed = Signal(np.ndarray)  # send updated pose to main thread if needed

    def __init__(self, oms, motion_gain, motion_limits, lowpass_strength=0.5, update_frequency=240, parent=None):
        super().__init__(parent)
        self.oms = oms
        self.motion_gain =  np.array( motion_gain, dtype=np.float32)
        self.lowpass_strength = lowpass_strength
        self.running = True
        self.update_frequency = update_frequency
        self.motion_limits = np.array( motion_limits, dtype=np.float32)

        self.last_mouse_pos = QCursor.pos()
        self.relative_device_pos = np.zeros(3)
        self.relative_device_pos_lp = np.zeros(3)
        self.initial_device_pos = np.zeros(3)
        self.device_pos_offset = np.zeros(3)
        self.current_pose = np.zeros(3)

        self.mutex = QMutex()

        if self.oms.is_connected():
            self.initial_device_pos = np.array(self.oms.read_current_position(True))
            self.current_pose[:] = self.initial_device_pos

    def stop(self):
        self.running = False
        self.wait(1000)

    def set_motion_limits(self, limits):
        self.mutex.lock()
        self.motion_limits = np.array( limits, dtype=np.float32)
        self.mutex.unlock()

    def on_mouse_wheel(self, delta):
        self.mutex.lock()
        self.relative_device_pos[2] += delta * self.motion_gain[2]
        self.mutex.unlock()

    def move_relative(self, x, y, z):
        self.mutex.lock()
        self.relative_device_pos += (x, y, z)
        self.mutex.unlock()

    def set_position_offset(self, x ,y, z):
        self.mutex.lock()
        self.device_pos_offset[:] = (x, y, z)
        self.mutex.unlock()

    def get_current_pose(self):
        self.mutex.lock()
        pose = list(self.current_pose)
        self.mutex.unlock()
        return pose

    def run(self):
        while self.running:
            self.mutex.lock()
            pos = QCursor.pos()
            delta = pos - self.last_mouse_pos
            self.last_mouse_pos = pos
            self.mutex.unlock()

            if abs(delta.x()) > 150 or abs(delta.y()) > 150:
                delta = QPoint(0, 0)

            t = self.lowpass_strength
            self.relative_device_pos += np.array([delta.x(), delta.y(), 0.0]) * self.motion_gain
            self.relative_device_pos = np.clip(self.relative_device_pos, -self.motion_limits, self.motion_limits)
            self.relative_device_pos_lp = self.relative_device_pos * (1.0 - t) + self.relative_device_pos_lp * t
            self.current_pose[:] = self.initial_device_pos + self.relative_device_pos_lp + self.device_pos_offset

            if self.oms.is_connected():
                self.oms.set_pose(self.current_pose[0], self.current_pose[1], self.current_pose[2])

            # print(f"Move to: {p[0]:10.7f}, {p[1]:10.7f}, {p[2]:10.7f}")

            time.sleep(1.0 / max(self.update_frequency, 1))


class RealtimeControllerWidget(QWidget, Ui_RealtimeControllerWidget):
    stop_control_signal = Signal()
    start_control_signal = Signal()

    def __init__(self, base_widget: QWidget = None, oms: OpenMicroStageInterface = None, parent=None):
        super().__init__(parent)

        self.oms = oms
        self.base_widget = base_widget
        self.mouse_control_active = False
        self.lowpass_strength = 0.9
        self.update_frequency = 120 # Hz
        self.motion_gain = np.array( [-0.001, 0.001, -0.005], dtype=np.float32)
        self.motion_limits = np.array( [1.0, 1.0, 1.0], dtype=np.float32)

        self.update_thread = None
        self.setupUi(self)
        self.mouse_control_button.toggled.connect(self.on_mouse_control_toggled)

        if base_widget is not None:
            self.setup(base_widget, oms)

    def setup(self, base_widget: QWidget, oms: OpenMicroStageInterface):
        self.base_widget = base_widget
        self.oms = oms
        self.base_widget.setMouseTracking(True)
        self.base_widget.installEventFilter(self)

    def get_current_pose(self):
        if self.update_thread is None:
            return [0.0, 0.0, 0.0]
        return self.update_thread.get_current_pose()

    def is_running(self):
        return self.update_thread is not None and self.update_thread.running

    def read_gui_settings(self):
        lx = ly = self.spinbox_xy_range.value()
        lz = self.spinbox_z_range.value()
        self.motion_limits = np.array([lx, ly, lz], np.float32)

    def start_control(self):
        if not self.oms.is_connected():
            self.mouse_control_button.blockSignals(True)
            self.mouse_control_button.setChecked(False)
            self.mouse_control_button.blockSignals(False)
            self.mouse_control_active = False
            return

        self.start_control_signal.emit()
       # QApplication.setOverrideCursor(QCursor(Qt.CursorShape.BlankCursor))
        self.base_widget.setFocus()
        self.constrain_cursor()

        self.read_gui_settings()
        motion_gain = self.motion_limits * self.motion_gain

        self.update_thread = UpdateWorker(self.oms,
                                          motion_gain=motion_gain,
                                          motion_limits=self.motion_limits,
                                          lowpass_strength=self.lowpass_strength)

        self.update_thread.start()

    def stop_control(self):
        self.mouse_control_button.setChecked(False)
        self.mouse_control_active = False
        QApplication.restoreOverrideCursor()
        if self.update_thread is not None:
            self.update_thread.stop()
            self.update_thread = None
        self.stop_control_signal.emit()

    def on_mouse_control_toggled(self, checked):
        if checked:
            self.start_control()

        self.mouse_control_active = checked

    def constrain_cursor(self):
        local = self.base_widget.mapFromGlobal(QCursor.pos())
        w, h = self.base_widget.width(), self.base_widget.height()
        margin = 100

        # Wrap coordinates if out of bounds
        x = margin if local.x() > w-margin else w - margin if local.x() < margin else local.x()
        y = margin if local.y() > h-margin else h - margin if local.y() < margin else local.y()

        if (x, y) != (local.x(), local.y()):
            pos = self.base_widget.mapToGlobal(QPoint(x, y))
            QCursor.setPos(pos)

    # ---- Event Filter ----
    def eventFilter(self, watched: QObject, event: QEvent) -> bool:
        if self.mouse_control_active:
            if event.type() == QEvent.Type.MouseMove:
                return self.handle_mouse_move(event)
            if event.type() == QEvent.Type.Wheel:
                return self.handle_mouse_wheel(event)
            elif event.type() == QEvent.Type.KeyPress:
                return self.handle_key_press(event)
            elif event.type() == QEvent.Type.MouseButtonPress:
                return self.handle_mouse_press(event)
            elif event.type() == QEvent.Type.MouseButtonDblClick:
                return self.handle_mouse_press(event)
            elif event.type() == QEvent.Type.MouseButtonRelease:
                return self.handle_mouse_release(event)
            elif event.type() == QEvent.Type.Leave:
                self.constrain_cursor()

        return super().eventFilter(watched, event)

    # ---- Event Handlers ----

    def handle_mouse_move(self, event):
        self.constrain_cursor()
        return True

    def handle_mouse_wheel(self, event):
        delta = 1.0 if event.angleDelta().y() > 0 else -1.0
        if self.update_thread is not None:
            self.update_thread.on_mouse_wheel(delta)

        # print(f"Mouse wheel: {delta}")
        return True

    def handle_mouse_press(self, event: QMouseEvent):
        if event.button() == Qt.MouseButton.LeftButton and self.update_thread is not None:
            self.update_thread.set_position_offset(0, 0, 0.1)
        return True

    def handle_mouse_release(self, event: QMouseEvent):
        if event.button() == Qt.MouseButton.LeftButton and self.update_thread is not None:
            self.update_thread.set_position_offset(0, 0, 0.0)
        return True

    def handle_key_press(self, event):
        key = event.key()
        # modifiers = event.modifiers()

        if key == Qt.Key.Key_Escape:
            self.stop_control()
            return True

        return False
