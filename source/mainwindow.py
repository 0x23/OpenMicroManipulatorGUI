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
from PySide6.QtCore import QMargins, Qt, QThread, Signal
from PySide6.QtGui import QCloseEvent, QFont
from PySide6.QtWidgets import (
    QButtonGroup,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSpacerItem,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)


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


class DeviceControlMainWindow(QMainWindow):
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
            self.connected_stage_label = "Configured Serial Device"
            self.on_stage_connected()

    def init_ui(self):
        self.setWindowTitle("Open Micro-Manipulator Control")
        self.setStyleSheet("background-color: #2b2b2b; color: white;")
        self.setMinimumSize(1280, 820)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_h_layout = QHBoxLayout(central_widget)
        main_h_layout.setContentsMargins(12, 12, 12, 12)
        main_h_layout.setSpacing(12)

        left_panel = QWidget()
        left_panel.setFixedWidth(380)
        main_layout = QVBoxLayout(left_panel)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(12)

        self.video_viewer = ImageViewerWidget()
        font = QFont("Noto Sans", 12)

        connection_layout = QGridLayout()
        connection_layout.setContentsMargins(QMargins(0, 0, 0, 0))
        connection_layout.setHorizontalSpacing(8)
        connection_layout.setVerticalSpacing(4)
        connection_layout.setColumnStretch(0, 1)
        connection_layout.setColumnStretch(1, 3)

        connection_layout.addWidget(self.create_label("Serial Device", font_size=11), 0, 0)
        self.serial_status_label = self.create_label(
            "Disconnected",
            alignment=Qt.AlignmentFlag.AlignRight,
            font_size=10,
        )
        connection_layout.addWidget(self.serial_status_label, 0, 1, 1, 3)
        self.serial_combo = QComboBox()
        connection_layout.addWidget(self.serial_combo, 1, 0, 1, 2)
        self.serial_refresh_button = self.create_compact_button("Scan", self.refresh_serial_devices)
        self.serial_connect_button = self.create_compact_button("Open", self.connect_selected_stage)
        self.serial_disconnect_button = self.create_compact_button("Close", self.disconnect_stage)
        connection_layout.addWidget(self.serial_refresh_button, 1, 2)
        connection_layout.addWidget(self.serial_connect_button, 1, 3)
        connection_layout.addWidget(self.serial_disconnect_button, 1, 4)

        connection_layout.addWidget(self.create_label("Camera", font_size=11), 2, 0)
        self.camera_status_label = self.create_label(
            "Disconnected",
            alignment=Qt.AlignmentFlag.AlignRight,
            font_size=10,
        )
        connection_layout.addWidget(self.camera_status_label, 2, 1, 1, 4)
        self.camera_combo = QComboBox()
        connection_layout.addWidget(self.camera_combo, 3, 0, 1, 2)
        self.camera_refresh_button = self.create_compact_button("Scan", self.refresh_camera_devices)
        self.camera_connect_button = self.create_compact_button("Open", self.connect_selected_camera)
        self.camera_disconnect_button = self.create_compact_button("Close", self.disconnect_camera)
        connection_layout.addWidget(self.camera_refresh_button, 3, 2)
        connection_layout.addWidget(self.camera_connect_button, 3, 3)
        connection_layout.addWidget(self.camera_disconnect_button, 3, 4)

        main_layout.addLayout(connection_layout)
        main_layout.addSpacing(8)

        grid = QGridLayout()
        grid.setSpacing(10)
        grid.addWidget(self.register_stage_widget(self.create_button("Y-", lambda: self.move_axis(1, -1), font)), 0, 1)
        grid.addWidget(self.register_stage_widget(self.create_button("Z+", lambda: self.move_axis(2, +1), font)), 0, 3)
        grid.addWidget(self.register_stage_widget(self.create_button("X-", lambda: self.move_axis(0, -1), font)), 1, 0)

        center_label = QLabel("•")
        center_label.setFont(QFont("Arial", 30))
        center_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        grid.addWidget(center_label, 1, 1)

        grid.addWidget(self.register_stage_widget(self.create_button("X+", lambda: self.move_axis(0, +1), font)), 1, 2)
        grid.addWidget(self.register_stage_widget(self.create_button("Y+", lambda: self.move_axis(1, +1), font)), 2, 1)
        grid.addWidget(self.register_stage_widget(self.create_button("Z-", lambda: self.move_axis(2, -1), font)), 2, 3)
        main_layout.addLayout(grid)

        main_layout.addSpacing(16)
        main_layout.addWidget(self.create_label("Step Size [µm]"))

        step_layout = QHBoxLayout()
        self.step_button_group = QButtonGroup()
        self.step_button_group.setExclusive(True)

        for i, val in enumerate(self.step_sizes):
            btn = self.register_stage_widget(
                self.create_button(str(val * 1000), lambda checked=False, idx=i: self.set_step_size(idx), font)
            )
            btn.setCheckable(True)
            if i == self.step_size_idx:
                btn.setChecked(True)
            self.step_button_group.addButton(btn, i)
            step_layout.addWidget(btn)

        main_layout.addLayout(step_layout)

        accel_layout, self.accel_spinbox = self.create_spinbox(
            label_text="Acceleration:",
            min_val=0.01,
            max_val=1000.0,
            step=1,
            default=50.0,
            decimals=2,
            update_immediately=False,
            callback=self.apply_acceleration_setting,
        )
        self.register_stage_widget(self.accel_spinbox)
        main_layout.addLayout(accel_layout)

        tool_layout, self.tool_spinbox = self.create_spinbox(
            label_text="Tool 1:",
            min_val=0.0,
            max_val=1.0,
            step=0.1,
            default=0.0,
            decimals=2,
            update_immediately=True,
            callback=self.apply_tool_setting,
        )
        self.register_stage_widget(self.tool_spinbox)
        main_layout.addLayout(tool_layout)

        main_layout.addSpacing(16)
        self.realtime_control_widget = self.register_stage_widget(
            RealtimeControllerWidget(self.video_viewer.viewport(), self.oms)
        )
        main_layout.addWidget(self.realtime_control_widget)
        self.realtime_control_widget.stop_control_signal.connect(self.on_stop_realtime_control)

        main_layout.addSpacing(8)
        tabs = QTabWidget()

        advanced_tab = QWidget()
        advanced_tab.setObjectName("AdvancedTab")
        advanced_layout = QGridLayout(advanced_tab)
        advanced_layout.setContentsMargins(QMargins(0, 11, 0, 0))

        advanced_layout.addWidget(
            self.register_stage_widget(self.create_button("3-Point Alignment", self.run_3point_alignment, font)),
            1,
            0,
        )
        advanced_layout.addWidget(
            self.register_stage_widget(self.create_button("Set Origin", self.set_origin, font)),
            1,
            1,
        )

        tracking_button = self.create_button("Set Tracking Point", self.set_tracking_point, font)
        self.register_camera_widget(tracking_button)
        advanced_layout.addWidget(tracking_button, 2, 0)

        clear_button = self.create_button("Clear", self.clear_draw_buffer, font)
        self.register_camera_widget(clear_button)
        advanced_layout.addWidget(clear_button, 2, 1)

        advanced_layout.addWidget(
            self.register_stage_widget(self.create_button("Load Transform", self.load_transform, font)),
            3,
            0,
        )
        advanced_layout.addWidget(
            self.register_stage_widget(self.create_button("Save Transform", self.save_transform, font)),
            3,
            1,
        )

        fiber_button = self.create_button("Fiber Alignment", self.run_fiber_alignment, font)
        self.register_stage_widget(fiber_button)
        self.register_camera_widget(fiber_button)
        advanced_layout.addWidget(fiber_button, 4, 0)
        advanced_layout.addItem(QSpacerItem(0, 10, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding))
        tabs.addTab(advanced_tab, "Advanced")

        path_tab = QWidget()
        path_tab.setObjectName("PathTab")
        path_layout = QVBoxLayout(path_tab)

        self.waypoint_info_label = self.create_label("Path Control")
        path_layout.addWidget(self.waypoint_info_label)

        wp_layout = QGridLayout()
        wp_layout.addWidget(
            self.register_stage_widget(self.create_button("Add Waypoint", self.add_waypoint, font)),
            0,
            0,
        )
        wp_layout.addWidget(
            self.register_stage_widget(self.create_button("Clear Waypoints", self.clear_waypoints, font)),
            0,
            1,
        )
        wp_layout.addWidget(
            self.register_stage_widget(self.create_button("Run Path", self.run_path, font)),
            1,
            0,
        )
        wp_layout.addWidget(
            self.register_stage_widget(self.create_button("Save Path", self.save_path, font)),
            1,
            1,
        )
        path_layout.addLayout(wp_layout)

        self.run_gcode_button = self.register_stage_widget(
            self.create_button("Run GCode", self.run_gcode_from_file, font)
        )
        self.run_gcode_button.setCheckable(True)
        path_layout.addWidget(self.run_gcode_button)
        tabs.addTab(path_tab, "Path / GCode")

        camera_tab = QWidget()
        camera_tab.setObjectName("CameraTab")
        camera_layout = QVBoxLayout(camera_tab)

        exposure_layout, self.exposure_spinbox = self.create_spinbox(
            label_text="Exposure:",
            min_val=-50,
            max_val=100000,
            step=1,
            default=1000,
            decimals=0,
            update_immediately=True,
            callback=self.apply_camera_settings,
        )
        self.register_camera_widget(self.exposure_spinbox)
        camera_layout.addLayout(exposure_layout)

        gain_layout, self.gain_spinbox = self.create_spinbox(
            label_text="Gain:",
            min_val=-24,
            max_val=100,
            step=1,
            default=0,
            decimals=0,
            update_immediately=True,
            callback=self.apply_camera_settings,
        )
        self.register_camera_widget(self.gain_spinbox)
        camera_layout.addLayout(gain_layout)

        wb_layout, self.white_balance_spinbox = self.create_spinbox(
            label_text="White Balance:",
            min_val=0,
            max_val=10000,
            step=100,
            default=3000,
            decimals=0,
            update_immediately=True,
            callback=self.apply_camera_settings,
        )
        self.register_camera_widget(self.white_balance_spinbox)
        camera_layout.addLayout(wb_layout)

        camera_layout.addItem(QSpacerItem(0, 10, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding))
        screenshot_button = self.create_button("Save Screenshot", self.save_screenshot, font)
        self.register_camera_widget(screenshot_button)
        camera_layout.addWidget(screenshot_button)
        tabs.addTab(camera_tab, "Camera")

        self.update_waypoint_info()
        main_layout.addWidget(tabs)

        home_button = self.register_stage_widget(self.create_button("Home", self.home, font))
        main_layout.addWidget(home_button)
        main_layout.addItem(QSpacerItem(0, 8, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding))

        self.video_viewer.setMinimumWidth(600)
        self.video_viewer.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.video_viewer.setStyleSheet("background-color: black; border: 2px solid #444;")

        left_panel_scroll = QScrollArea()
        left_panel_scroll.setWidgetResizable(True)
        left_panel_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        left_panel_scroll.setWidget(left_panel)
        left_panel_scroll.setFixedWidth(396)
        left_panel_scroll.setStyleSheet("QScrollArea { border: none; background-color: transparent; }")
        self.tool_panel = left_panel_scroll

        main_h_layout.addWidget(left_panel_scroll)
        main_h_layout.addWidget(self.video_viewer, stretch=1)

        self.set_stylesheet()

    @staticmethod
    def create_label(text, alignment=Qt.AlignmentFlag.AlignLeft, font_size=12):
        label = QLabel(text)
        label.setAlignment(alignment)
        label.setFont(QFont("Noto Sans", font_size))
        return label

    @staticmethod
    def create_button(label, slot, font):
        btn = QPushButton(label)
        btn.setMinimumSize(60, 40)
        btn.setMaximumHeight(40)
        btn.clicked.connect(slot)
        return btn

    @staticmethod
    def create_compact_button(label, slot):
        btn = QPushButton(label)
        btn.setMinimumHeight(30)
        btn.setMaximumHeight(30)
        btn.setMinimumWidth(54)
        btn.clicked.connect(slot)
        return btn

    @staticmethod
    def create_spinbox(label_text, min_val, max_val, step, default, decimals, update_immediately, callback):
        layout = QHBoxLayout()
        label = QLabel(label_text)
        spinbox = QDoubleSpinBox()
        spinbox.setRange(min_val, max_val)
        spinbox.setSingleStep(step)
        spinbox.setValue(default)
        spinbox.setDecimals(decimals)

        if update_immediately:
            spinbox.valueChanged.connect(callback)
        else:
            spinbox.editingFinished.connect(callback)

        layout.addWidget(label)
        layout.addWidget(spinbox)
        return layout, spinbox

    def register_stage_widget(self, widget):
        self.stage_dependent_widgets.append(widget)
        return widget

    def register_camera_widget(self, widget):
        self.camera_dependent_widgets.append(widget)
        return widget

    def set_stylesheet(self):
        self.setStyleSheet("""
            QTabWidget > QStackedWidget > QWidget,
            QTabWidget QLabel {
                background-color: #353535;
            }

            QWidget {
                background-color: #2b2b2b;
                color: white;
            }

            QPushButton, QComboBox, QDoubleSpinBox {
                background-color: #2d5291;
                color: white;
                border: 1px solid #202020;
                border-radius: 3px;
                padding: 6px;
            }

            QComboBox QAbstractItemView {
                background-color: #1f1f1f;
                color: white;
                selection-background-color: #3b72d1;
            }

            QPushButton:hover {
                background-color: #3b72d1;
            }

            QPushButton:checked,
            QPushButton:pressed {
                background-color: #32a877;
            }

            QWidget#AdvancedTab QPushButton {
                background-color: #4d5291;
            }

            QWidget#AdvancedTab QPushButton:hover {
                background-color: #5b72d1;
            }

            QWidget#AdvancedTab QPushButton:checked,
            QWidget#AdvancedTab QPushButton:pressed {
                background-color: #32a877;
            }

            QPushButton:disabled, QComboBox:disabled, QDoubleSpinBox:disabled {
                background-color: #4a4a4a;
                color: #b0b0b0;
            }

            QTabWidget::pane {
                border: none;
            }

            QTabBar::tab {
                background: #353535;
                padding: 6px;
                border: 1px solid #282828;
            }

            QTabBar::tab:selected {
                background: #856A51;
                border-bottom: none;
            }
        """)

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

        self.serial_connect_button.setEnabled(not stage_connected and self.serial_combo.count() > 0)
        self.serial_disconnect_button.setEnabled(stage_connected)
        self.camera_connect_button.setEnabled(not camera_connected and self.camera_combo.count() > 0)
        self.camera_disconnect_button.setEnabled(camera_connected)

        stage_status = self.connected_stage_label if stage_connected else "Disconnected"
        camera_status = self.connected_camera_label if camera_connected else "Disconnected"
        self.serial_status_label.setText(stage_status)
        self.camera_status_label.setText(camera_status)

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
