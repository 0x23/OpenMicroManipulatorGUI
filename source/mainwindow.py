# --------------------------------------------------------------------------------------
# Project: OpenMicroManipulator
# License: MIT (see LICENSE file for full description)
#          All text in here must be included in any redistribution.
# Author:  M. S. (diffraction limited)
# --------------------------------------------------------------------------------------

import json
import os
from datetime import datetime

import cv2
import numpy as np

from gcode_runner import GCodeRunner
from gui_components.image_viewer_widget import ImageViewerWidget
from gui_components.realtime_controller_widget import RealtimeControllerWidget
from hardware.camera_basler import BaslerCamera
from hardware.camera_opencv import OpenCVCamera
from hardware.device_discovery import list_camera_devices, list_serial_devices
from hardware.open_micro_stage_api import OpenMicroStageInterface
from image_processing.image_point_tracker import ImagePointTracker
from optical_alignment import OpticalAlignment
from PySide6.QtCore import Qt, QThread, Signal
from PySide6.QtGui import QCloseEvent
from PySide6.QtUiTools import loadUiType
from PySide6.QtWidgets import (
    QButtonGroup,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
)

_ui_path = os.path.join(os.path.dirname(__file__), "mainwindow.ui")
Ui_DeviceControlMainWindow, _ = loadUiType(_ui_path)


class CameraStreamWorker(QThread):
    frame_ready = Signal(object)
    stream_error = Signal(str)

    def __init__(self, camera, parent=None):
        super().__init__(parent)
        self.camera = camera
        self.running = True

    def stop(self):
        self.running = False
        try:
            self.camera.stop_grabbing()
        except Exception:
            pass
        self.wait(1500)

    def _handle_frame(self, frame):
        if not self.running:
            return False

        self.frame_ready.emit(frame)
        return True

    def run(self):
        try:
            self.camera.grab_loop(callback=self._handle_frame, timeout_ms=500)
        except Exception as exc:
            if self.running:
                self.stream_error.emit(str(exc))


class DeviceControlMainWindow(QMainWindow, Ui_DeviceControlMainWindow):
    def __init__(self, oms: OpenMicroStageInterface, camera=None):
        super().__init__()

        self.oms = oms
        self.camera = camera
        self.camera_stream_worker = None
        self.gcode_runner = None

        self.pixel_per_mm = 2000.0
        self.connected_stage_label = None
        self.connected_camera_label = None
        self.serial_devices = []
        self.camera_devices = []
        self.stage_dependent_widgets = []
        self.camera_dependent_widgets = []

        self.last_frame = None
        self.draw_buffer = None
        self.current_pos = [0, 0, 0]
        self.step_sizes = [1.0, 0.1, 0.01, 0.001, 0.0001]
        self.feedrates = [50.0, 5.0, 5.0, 1.0, 0.1]
        self.step_size_idx = 1
        self.waypoints = []
        self.waypoint_idx = 1000000

        self.image_point_tracker = ImagePointTracker()

        self.init_ui()
        self.refresh_serial_devices()
        self.refresh_camera_devices()
        self.show_placeholder_frame()
        self.update_connection_state()

        if self.camera is not None and self.camera.is_connected():
            self.connected_camera_label = "Configured Camera"
            self.apply_camera_settings()
            self.start_camera_stream()

        if self.oms.is_connected():
            self.connected_stage_label = "Configured Device"
            self.on_stage_connected()

    def init_ui(self):
        self.setupUi(self)

        self.video_viewer.setAlignment(Qt.AlignmentFlag.AlignCenter)

        # Connect signals – connection buttons
        self.serial_refresh_button.clicked.connect(self.refresh_serial_devices)
        self.serial_connect_button.clicked.connect(self.toggle_stage_connection)
        self.camera_refresh_button.clicked.connect(self.refresh_camera_devices)
        self.camera_connect_button.clicked.connect(self.toggle_camera_connection)

        # Connect signals – movement buttons
        for name, axis, direction in [
            ("btn_y_minus", 1, -1), ("btn_z_plus", 2, +1),
            ("btn_x_minus", 0, -1), ("btn_x_plus", 0, +1),
            ("btn_y_plus", 1, +1), ("btn_z_minus", 2, -1),
        ]:
            btn = getattr(self, name)
            btn.clicked.connect(lambda checked=False, a=axis, d=direction: self.move_axis(a, d))

        # Spinbox signals
        self.accel_spinbox.editingFinished.connect(self.apply_acceleration_setting)
        self.tool_spinbox.valueChanged.connect(self.apply_tool_setting)
        self.exposure_spinbox.valueChanged.connect(self.apply_camera_settings)
        self.gain_spinbox.valueChanged.connect(self.apply_camera_settings)
        self.white_balance_spinbox.valueChanged.connect(self.apply_camera_settings)

        # Step size buttons
        self.step_button_group = QButtonGroup()
        self.step_button_group.setExclusive(True)
        for i in range(len(self.step_sizes)):
            btn = getattr(self, f"step_btn_{i}")
            btn.clicked.connect(lambda checked=False, idx=i: self.set_step_size(idx))
            self.step_button_group.addButton(btn, i)

        self.realtime_control_widget.setup(self.video_viewer.viewport(), self.oms)
        self.realtime_control_widget.stop_control_signal.connect(self.on_stop_realtime_control)

        self.register_stage_widget(self.device_controls_frame)
        self.register_stage_widget(self.AdvancedTab)
        self.register_stage_widget(self.PathTab)
        self.register_camera_widget(self.CameraTab)

        # Advanced tab
        self.btn_3point_alignment.clicked.connect(self.run_3point_alignment)
        self.btn_set_origin.clicked.connect(self.set_origin)
        self.btn_set_tracking.clicked.connect(self.set_tracking_point)
        self.btn_clear.clicked.connect(self.clear_draw_buffer)
        self.btn_load_transform.clicked.connect(self.load_transform)
        self.btn_save_transform.clicked.connect(self.save_transform)
        self.btn_fiber_alignment.clicked.connect(self.run_fiber_alignment)

        # Path / GCode tab
        self.btn_add_waypoint.clicked.connect(self.add_waypoint)
        self.btn_clear_waypoints.clicked.connect(self.clear_waypoints)
        self.btn_run_path.clicked.connect(self.run_path)
        self.btn_save_path.clicked.connect(self.save_path)
        self.run_gcode_button.clicked.connect(self.run_gcode_from_file)

        # Camera tab
        self.btn_save_screenshot.clicked.connect(self.save_screenshot)

        # Home button
        self.btn_home.clicked.connect(self.home)

        self.update_waypoint_info()

    def register_stage_widget(self, widget):
        self.stage_dependent_widgets.append(widget)
        return widget

    def register_camera_widget(self, widget):
        self.camera_dependent_widgets.append(widget)
        return widget

    def show_placeholder_frame(self):
        placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
        self.video_viewer.set_image(placeholder, pixel_per_mm=self.pixel_per_mm)

    def clear_visual_state(self):
        self.last_frame = None
        self.draw_buffer = None
        self.image_point_tracker.reset()

    def refresh_serial_devices(self):
        selected_id = self.serial_combo.currentData().get("id") if self.serial_combo.currentData() else None
        self.serial_devices = list_serial_devices()
        self.serial_combo.clear()

        for device in self.serial_devices:
            self.serial_combo.addItem(device["label"], device)

        if selected_id is not None:
            for index, device in enumerate(self.serial_devices):
                if device["id"] == selected_id:
                    self.serial_combo.setCurrentIndex(index)
                    break

        self.update_connection_state()

    def refresh_camera_devices(self):
        selected_id = self.camera_combo.currentData().get("id") if self.camera_combo.currentData() else None
        self.camera_devices = list_camera_devices()
        self.camera_combo.clear()

        for device in self.camera_devices:
            self.camera_combo.addItem(device["label"], device)

        if selected_id is not None:
            for index, device in enumerate(self.camera_devices):
                if device["id"] == selected_id:
                    self.camera_combo.setCurrentIndex(index)
                    break

        self.update_connection_state()

    def update_connection_state(self):
        stage_connected = self.oms.is_connected()
        camera_connected = self.camera is not None and self.camera.is_connected()

        for widget in self.stage_dependent_widgets:
            widget.setEnabled(stage_connected)

        for widget in self.camera_dependent_widgets:
            widget.setEnabled(camera_connected)

        self.serial_connect_button.setText("Close" if stage_connected else "Open")
        self.serial_connect_button.setEnabled(stage_connected or self.serial_combo.count() > 0)
        self.camera_connect_button.setText("Close" if camera_connected else "Open")
        self.camera_connect_button.setEnabled(camera_connected or self.camera_combo.count() > 0)

        stage_status = f"[{self.connected_stage_label}]" if stage_connected else "[Disconnected]"
        camera_status = f"[{self.connected_camera_label}]" if camera_connected else "[Disconnected]"
        self.serial_status_label.setText(stage_status)
        self.camera_status_label.setText(camera_status)

        for label, connected in (
            (self.serial_status_label, stage_connected),
            (self.camera_status_label, camera_connected),
        ):
            label.setProperty("connected", connected)
            label.style().unpolish(label)
            label.style().polish(label)

    def toggle_stage_connection(self):
        if self.oms.is_connected():
            self.disconnect_stage()
        else:
            self.connect_selected_stage()

    def toggle_camera_connection(self):
        if self.camera is not None and self.camera.is_connected():
            self.disconnect_camera()
        else:
            self.connect_selected_camera()

    def connect_selected_stage(self):
        if self.oms.is_connected():
            return

        device = self.serial_combo.currentData()
        if device is None:
            QMessageBox.warning(self, "No Serial Device", "No serial device is available.")
            return

        port = device["port"]
        if not self.oms.connect(port):
            QMessageBox.critical(self, "Connection Failed", f"Failed to connect to serial device:\n{port}")
            self.connected_stage_label = None
            self.update_connection_state()
            return

        self.connected_stage_label = device["label"]
        self.on_stage_connected()

    def on_stage_connected(self):
        position = self.oms.read_current_position(True)
        if position[0] is not None:
            self.current_pos = list(position)

        self.apply_acceleration_setting()
        self.apply_tool_setting()
        self.update_connection_state()

    def disconnect_stage(self):
        if self.realtime_control_widget.is_running():
            self.realtime_control_widget.stop_control()

        self.stop_gcode_runner()
        self.oms.disconnect()
        self.connected_stage_label = None
        self.current_pos = [0, 0, 0]
        self.update_connection_state()

    def create_camera_from_config(self, config):
        if config["kind"] == "opencv":
            return OpenCVCamera(
                camera_index=config["index"],
                backend=config.get("backend", cv2.CAP_ANY),
            )

        if config["kind"] == "basler":
            return BaslerCamera(device_serial=config["id"])

        raise ValueError(f"Unsupported camera kind: {config['kind']}")

    def connect_selected_camera(self):
        if self.camera is not None and self.camera.is_connected():
            return

        config = self.camera_combo.currentData()
        if config is None:
            QMessageBox.warning(self, "No Camera", "No camera is available.")
            return

        try:
            camera = self.create_camera_from_config(config)
        except Exception as exc:
            QMessageBox.critical(self, "Camera Error", str(exc))
            return

        if camera is None or not camera.is_connected():
            if camera is not None:
                camera.close()
            QMessageBox.critical(self, "Camera Error", f"Failed to connect to camera:\n{config['label']}")
            return

        self.disconnect_camera(show_placeholder=False)
        self.camera = camera
        self.connected_camera_label = config["label"]
        self.clear_visual_state()
        self.apply_camera_settings()
        self.start_camera_stream()
        self.update_connection_state()

    def apply_camera_settings(self):
        if self.camera is None or not self.camera.is_connected():
            return

        try:
            self.camera.set_exposure_time(self.exposure_spinbox.value())
        except Exception:
            pass

        try:
            self.camera.set_gain(self.gain_spinbox.value())
        except Exception:
            pass

        try:
            self.camera.set_white_balance(self.white_balance_spinbox.value())
        except Exception:
            pass

    def start_camera_stream(self):
        if self.camera is None or not self.camera.is_connected():
            return

        self.stop_camera_stream()
        self.camera_stream_worker = CameraStreamWorker(self.camera, self)
        self.camera_stream_worker.frame_ready.connect(self.on_frame_received)
        self.camera_stream_worker.stream_error.connect(self.on_camera_stream_error)
        self.camera_stream_worker.start()

    def stop_camera_stream(self):
        if self.camera_stream_worker is not None:
            self.camera_stream_worker.stop()
            self.camera_stream_worker = None

    def disconnect_camera(self, show_placeholder=True):
        self.stop_camera_stream()

        if self.camera is not None:
            self.camera.close()
            self.camera = None

        self.connected_camera_label = None
        self.clear_visual_state()
        if show_placeholder:
            self.show_placeholder_frame()
        self.update_connection_state()

    def on_camera_stream_error(self, message):
        self.disconnect_camera()
        QMessageBox.critical(self, "Camera Stream Error", message)

    def on_frame_received(self, frame):
        if frame is None:
            return

        if len(frame.shape) == 2 or (len(frame.shape) == 3 and frame.shape[2] == 1):
            vis_img = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)
        else:
            vis_img = frame.copy()

        self.update_controller(frame, vis_img, pixel_per_mm=self.pixel_per_mm)

    def stop_gcode_runner(self):
        if self.gcode_runner is not None:
            self.gcode_runner.stop()
            self.gcode_runner = None

        self.run_gcode_button.blockSignals(True)
        self.run_gcode_button.setChecked(False)
        self.run_gcode_button.blockSignals(False)

    def apply_acceleration_setting(self):
        if self.oms.is_connected():
            self.oms.set_max_acceleration(self.accel_spinbox.value(), 5000)

    def apply_tool_setting(self):
        if self.oms.is_connected():
            self.oms.set_tool_output(0, self.tool_spinbox.value(), immediate=True)

    def require_stage_connection(self):
        if self.oms.is_connected():
            return True

        QMessageBox.warning(self, "Manipulator Disconnected", "Connect a serial device before using this control.")
        return False

    def require_camera_connection(self):
        if self.camera is not None and self.camera.is_connected():
            return True

        QMessageBox.warning(self, "Camera Disconnected", "Connect a camera before using this control.")
        return False

    def run_fiber_alignment(self):
        if not self.require_stage_connection() or not self.require_camera_connection():
            return

        aligner = OpticalAlignment(self.oms, self.camera)
        pos, _ = aligner.optimize()
        self.camera.start_grabbing(False)
        self.current_pos = pos

    def on_stop_realtime_control(self):
        if self.oms.is_connected():
            position = self.oms.read_current_position(True)
            if position[0] is not None:
                self.current_pos[:] = position

    def home(self):
        if not self.require_stage_connection():
            return

        self.oms.home()
        position = self.oms.read_current_position(True)
        if position[0] is not None:
            self.current_pos = list(position)

    def move_axis(self, axis, direction):
        if not self.require_stage_connection():
            return

        flipped = (1, 1, 1)
        d = self.step_sizes[self.step_size_idx]
        self.current_pos[axis] += direction * d * flipped[axis]
        self.current_pos[axis] = max(min(self.current_pos[axis], 10), -10)
        self.oms.move_to(*self.current_pos, self.feedrates[self.step_size_idx])

    def add_waypoint(self):
        if not self.require_stage_connection():
            return

        if self.realtime_control_widget.is_running():
            self.current_pos[:] = self.realtime_control_widget.get_current_pose()

        self.waypoints.append([self.current_pos.copy(), self.feedrates[self.step_size_idx]])
        self.update_waypoint_info()

    def run_path(self):
        if not self.require_stage_connection():
            return

        self.waypoint_idx = 0

    def save_path(self):
        if len(self.waypoints) <= 0:
            QMessageBox.critical(self, "Save Error", "Waypoint list is empty")
            return

        path, _ = QFileDialog.getSaveFileName(
            self,
            "Save Path to G-code File",
            "",
            "G-code Files (*.g *.gcode);;Text Files (*.txt);;All Files (*.*)",
        )

        if not path:
            return

        try:
            with open(path, "w", encoding="utf-8") as handle:
                handle.write("; Generated by Open Micro Manipulator Controller\n")
                handle.write("G90 ; absolute positioning\n\n")

                for waypoint in self.waypoints:
                    (x, y, z), feedrate = waypoint
                    handle.write(f"G0 X{x:.10f} Y{y:.10f} Z{z:.10f} F{feedrate * 60:.3f}\n")
                    handle.write("G4 S0.100000\n")

                handle.write("\n; End of file\n")
        except Exception as exc:
            QMessageBox.critical(self, "Save Error", f"Failed to save G-code file:\n{exc}")

    def set_origin(self):
        if not self.require_stage_connection():
            return

        position = self.oms.read_current_position(True)
        if position[0] is None:
            QMessageBox.critical(self, "Read Error", "Failed to read current stage position.")
            return

        self.current_pos[:] = position
        transform = self.oms.get_workspace_transform()
        transform[0, 3] += self.current_pos[0]
        transform[1, 3] += self.current_pos[1]
        transform[2, 3] += self.current_pos[2]
        self.oms.set_workspace_transform(transform)
        self.current_pos = [0, 0, 0]
        self.oms.move_to(0, 0, 0, self.feedrates[self.step_size_idx])

    def clear_waypoints(self):
        self.waypoints.clear()
        self.update_waypoint_info()

    def update_waypoint_info(self):
        self.waypoint_info_label.setText(f"Path Control  [ {len(self.waypoints)} Waypoints ]")

    def set_step_size(self, index):
        self.step_size_idx = index

    def update_controller(self, frame, vis_image, pixel_per_mm):
        if self.oms.is_connected() and self.waypoint_idx <= len(self.waypoints) and len(self.waypoints) > 0:
            self.current_pos[:], feedrate = self.waypoints[self.waypoint_idx % len(self.waypoints)]
            self.oms.move_to(*self.current_pos, feedrate)
            self.oms.dwell(0.1, False)
            self.waypoint_idx += 1
        else:
            self.waypoint_idx = 1000000

        prev_pos = self.image_point_tracker.prev_pos
        current_pos = self.image_point_tracker.update(frame)

        if current_pos is not None:
            px, py = current_pos
            cv2.circle(vis_image, (px, py), 4, (255, 0, 0), thickness=-1)

            if self.draw_buffer is None:
                self.draw_buffer = np.zeros_like(vis_image)
            elif prev_pos is not None:
                cv2.line(self.draw_buffer, prev_pos, current_pos, color=(255, 255, 255), thickness=1)

            if self.draw_buffer is not None:
                mask = self.draw_buffer[:, :, 0] != 0
                vis_image[mask, :] = np.array((0, 255, 100), dtype=np.uint8)

        self.video_viewer.set_image(vis_image, pixel_per_mm=pixel_per_mm)
        self.last_frame = frame
        return vis_image

    def set_tracking_point(self):
        if not self.require_camera_connection():
            return

        if self.last_frame is None:
            return

        self.image_point_tracker.set_track_point(
            self.last_frame,
            self.last_frame.shape[1] // 2,
            self.last_frame.shape[0] // 2,
        )

    def clear_draw_buffer(self):
        if self.draw_buffer is not None:
            self.draw_buffer = None
            self.image_point_tracker.reset()

    def run_gcode_from_file(self, checked):
        if checked:
            if not self.require_stage_connection():
                self.run_gcode_button.blockSignals(True)
                self.run_gcode_button.setChecked(False)
                self.run_gcode_button.blockSignals(False)
                return

            path, _ = QFileDialog.getOpenFileName(
                self,
                "Open G-code File",
                "",
                "G-code Files (*.g *.gcode);;Text Files (*.txt);;All Files (*.*)",
            )

            if not path:
                self.run_gcode_button.blockSignals(True)
                self.run_gcode_button.setChecked(False)
                self.run_gcode_button.blockSignals(False)
                return

            try:
                with open(path, "r", encoding="utf-8") as handle:
                    gcode = handle.read()
            except Exception as exc:
                QMessageBox.critical(self, "Error", f"Failed to open file:\n{exc}")
                self.run_gcode_button.blockSignals(True)
                self.run_gcode_button.setChecked(False)
                self.run_gcode_button.blockSignals(False)
                return

            self.gcode_runner = GCodeRunner(gcode, self.oms, max_feedrate=0.5)

            def on_finished():
                self.gcode_runner = None
                self.run_gcode_button.blockSignals(True)
                self.run_gcode_button.setChecked(False)
                self.run_gcode_button.blockSignals(False)

            def on_iteration_finished():
                pass

            self.gcode_runner.run(on_finished, on_iteration_finished, loop_playback=False)
        elif self.gcode_runner is not None:
            self.gcode_runner.stop()

    def run_3point_alignment(self):
        if not self.require_stage_connection():
            return

        if len(self.waypoints) != 3:
            QMessageBox.critical(self, "Error", "Exactly 3 Waypoints need to be recorded for this function")
            return

        old_workspace_transform = self.oms.get_workspace_transform()

        p0 = np.array(self.waypoints[0][0])
        p1 = np.array(self.waypoints[1][0])
        p2 = np.array(self.waypoints[2][0])

        v1 = p1 - p0
        v2 = p2 - p0
        z_axis = np.cross(v1, v2)
        if z_axis[2] < 0.0:
            z_axis = -z_axis
        z_axis /= np.linalg.norm(z_axis)

        global_x = np.array([1.0, 0.0, 0.0])
        x_proj = global_x - np.dot(global_x, z_axis) * z_axis
        x_axis = x_proj / np.linalg.norm(x_proj)

        y_axis = np.cross(z_axis, x_axis)
        y_axis /= np.linalg.norm(y_axis)

        transform = np.eye(4)
        transform[:3, 0] = x_axis
        transform[:3, 1] = y_axis
        transform[:3, 2] = z_axis
        transform[:3, 3] = p0

        self.oms.set_workspace_transform(transform @ old_workspace_transform)
        QMessageBox.information(self, "Alignment Complete", "3-point alignment complete.")

        self.oms.move_to(0, 0, 0, self.feedrates[self.step_size_idx])
        self.current_pos = [0, 0, 0]

    def load_transform(self):
        if not self.require_stage_connection():
            return

        try:
            with open("transform.json", "r", encoding="utf-8") as handle:
                data = json.load(handle)
        except Exception as exc:
            QMessageBox.critical(self, "Load Error", f"Failed to load transform:\n{exc}")
            return

        transform = np.array(data)
        self.oms.set_workspace_transform(transform)
        self.oms.move_to(0, 0, 0, self.feedrates[self.step_size_idx])
        self.current_pos = [0, 0, 0]

    def save_transform(self, pressed=True, ask=True):
        if ask:
            confirmed = QMessageBox.question(
                None,
                "Save Transform",
                "Are you sure?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No,
            ) == QMessageBox.StandardButton.Yes
            if not confirmed:
                return

        try:
            transform = self.oms.get_workspace_transform()
            with open("transform.json", "w", encoding="utf-8") as handle:
                json.dump(transform.tolist(), handle)
        except Exception as exc:
            QMessageBox.critical(self, "Save Error", f"Failed to save transform:\n{exc}")

    def closeEvent(self, event: QCloseEvent):
        if self.realtime_control_widget.is_running():
            self.realtime_control_widget.stop_control()

        self.stop_gcode_runner()
        self.disconnect_camera()
        self.disconnect_stage()
        super().closeEvent(event)

    def save_screenshot(self):
        if self.last_frame is None:
            return

        os.makedirs("./screenshots", exist_ok=True)
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        filename = f"screenshot_{timestamp}.png"
        path = os.path.join("./screenshots", filename)
        img = cv2.cvtColor(self.last_frame, cv2.COLOR_RGB2BGR)
        cv2.imwrite(path, img)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key.Key_A:
            self.move_axis(0, -1)
        elif event.key() == Qt.Key.Key_D:
            self.move_axis(0, +1)
        elif event.key() == Qt.Key.Key_W:
            self.move_axis(1, -1)
        elif event.key() == Qt.Key.Key_S:
            self.move_axis(1, +1)
        elif event.key() == Qt.Key.Key_R:
            self.move_axis(2, +1)
        elif event.key() == Qt.Key.Key_F:
            self.move_axis(2, -1)
        elif event.key() == Qt.Key.Key_K:
            self.add_waypoint()
        elif event.key() == Qt.Key.Key_P:
            self.save_screenshot()
        elif event.key() == Qt.Key.Key_F11:
            self.tool_panel.setVisible(not self.tool_panel.isVisible())
        else:
            super().keyPressEvent(event)
