# --------------------------------------------------------------------------------------
# Project: OpenMicroManipulator
# License: MIT (see LICENSE file for full description)
#          All text in here must be included in any redistribution.
# Author:  M. S. (diffraction limited)
# --------------------------------------------------------------------------------------

import os
import json

import cv2
import numpy as np
from hardware.open_micro_stage_api import OpenMicroStageInterface
from image_processing.image_point_tracker import ImagePointTracker
from optical_alignment import OpticalAlignment
from gui_components.image_viewer_widget import ImageViewerWidget
from gui_components.realtime_controller_widget import RealtimeControllerWidget
from gcode_runner import GCodeRunner
from datetime import datetime
from PySide6.QtWidgets import QSpinBox

from PySide6.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QLabel,
    QGridLayout, QMessageBox, QButtonGroup, QDoubleSpinBox, QFileDialog,
    QMainWindow, QFrame, QSpacerItem, QSizePolicy, QTabWidget
)
from PySide6.QtCore import Qt, QMargins
from PySide6.QtGui import QFont


class DeviceControlMainWindow(QMainWindow):

    def __init__(self, oms: OpenMicroStageInterface, camera):
        super().__init__()

        self.oms = oms
        self.camera = camera
        self.gcode_runner = None

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

        if self.oms.is_connected():
            self.current_pos = list(self.oms.read_current_position(True))
            self.oms.set_max_acceleration(self.accel_spinbox.value(), 5000)

    def init_ui(self):

        self.setWindowTitle("Open Micro-Manipulator Control")
        self.setStyleSheet("background-color: #2b2b2b; color: white;")

        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_h_layout = QHBoxLayout(central_widget)

        # LEFT PANEL
        self.tool_panel = QWidget()
        self.tool_panel.setFixedWidth(350)
        main_layout = QVBoxLayout(self.tool_panel)

        self.video_viewer = ImageViewerWidget()

        font = QFont("Noto Sans", 12)

        # MOVEMENT GRID
        grid = QGridLayout()
        grid.setSpacing(10)

        grid.addWidget(self.create_button("Y-", lambda: self.move_axis(1, -1), font), 0, 1)
        grid.addWidget(self.create_button("Z+", lambda: self.move_axis(2, +1), font), 0, 3)
        grid.addWidget(self.create_button("X-", lambda: self.move_axis(0, -1), font), 1, 0)

        center_label = QLabel("•")
        center_label.setFont(QFont("Arial", 30))
        center_label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        grid.addWidget(center_label, 1, 1)

        grid.addWidget(self.create_button("X+", lambda: self.move_axis(0, +1), font), 1, 2)
        grid.addWidget(self.create_button("Y+", lambda: self.move_axis(1, +1), font), 2, 1)
        grid.addWidget(self.create_button("Z-", lambda: self.move_axis(2, -1), font), 2, 3)

        main_layout.addLayout(grid)

        # STEP SIZE
        main_layout.addSpacing(25)
        main_layout.addWidget(self.create_label("Step Size [µm]"))

        step_layout = QHBoxLayout()

        self.step_button_group = QButtonGroup()
        self.step_button_group.setExclusive(True)

        for i, val in enumerate(self.step_sizes):

            btn = self.create_button(str(val * 1000),
                                     lambda checked=False, idx=i: self.set_step_size(idx),
                                     font)

            btn.setCheckable(True)

            if i == self.step_size_idx:
                btn.setChecked(True)

            self.step_button_group.addButton(btn, i)
            step_layout.addWidget(btn)

        main_layout.addLayout(step_layout)

        # ACCELERATION
        accel_layout, self.accel_spinbox = self.create_spinbox(
            label_text="Acceleration:", min_val=0.01, max_val=1000.0, step=1, default=50.00, decimals=2,
            callback=lambda: self.oms.set_max_acceleration(self.accel_spinbox.value(), 5000),
            update_immediately=False
        )
        main_layout.addLayout(accel_layout)

        # TOOL 1
        tool_layout, self.tool_spinbox = self.create_spinbox(
            label_text="Tool 1:", min_val=0.0, max_val=1.0, step=0.1, default=0.00, decimals=2,
            callback=lambda: self.oms.set_tool_output(0, self.tool_spinbox.value(), immediate=True),
            update_immediately = True
        )
        main_layout.addLayout(tool_layout)

        # REALTIME CONTROL
        main_layout.addSpacing(25)
        rc_layout = QHBoxLayout()
        self.realtime_control_widget = RealtimeControllerWidget(
            self.video_viewer.viewport(),
            self.oms
        )

        rc_layout.addWidget(self.realtime_control_widget, 0)
        main_layout.addLayout(rc_layout)

        self.realtime_control_widget.stop_control_signal.connect(
            self.on_stop_realtime_control
        )

        # vertical stretch
        main_layout.addItem(QSpacerItem(0, 10, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding))

        # TAB WIDGET
        tabs = QTabWidget()

        # TAB 1 : ADVANCED
        advanced_tab = QWidget()
        advanced_tab.setObjectName("AdvancedTab")

        advanced_layout = QGridLayout(advanced_tab)
        advanced_layout.setContentsMargins(QMargins(0, 11, 0, 0))

        advanced_layout.addWidget(
            self.create_button("3-Point Alignment", self.run_3point_alignment, font),  1, 0)
        advanced_layout.addWidget(
            self.create_button("Set Origin", lambda: self.set_origin(), font), 1, 1)
        advanced_layout.addWidget(
            self.create_button("Set Tracking Point", self.set_tracking_point, font), 2, 0)
        advanced_layout.addWidget(
            self.create_button("Clear", self.clear_draw_buffer, font), 2, 1)
        advanced_layout.addWidget(
            self.create_button("Load Transform", self.load_transform, font), 3, 0)
        advanced_layout.addWidget(
            self.create_button("Save Transform", self.save_transform, font), 3, 1)
        advanced_layout.addWidget(
            self.create_button("Fiber Alignment", self.run_fiber_alignment, font), 4, 0)

        advanced_layout.addItem(QSpacerItem(0, 10, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding))

        tabs.addTab(advanced_tab, "Advanced")

        # TAB 2 : PATH / GCODE
        path_tab = QWidget()
        path_tab.setObjectName("PathTab")
        path_layout = QVBoxLayout(path_tab)

        self.waypoint_info_label = self.create_label("Path Control")

        path_layout.addWidget(self.waypoint_info_label)

        wp_layout = QGridLayout()
        wp_layout.addWidget(self.create_button("Add Waypoint", self.add_waypoint, font), 0, 0)
        wp_layout.addWidget(self.create_button("Clear Waypoints", self.clear_waypoints, font), 0, 1)
        wp_layout.addWidget(self.create_button("Run Path", self.run_path, font), 1, 0)
        wp_layout.addWidget(self.create_button("Save Path", self.save_path, font), 1, 1)

        path_layout.addLayout(wp_layout)

        self.run_gcode_button = self.create_button(
            "Run GCode",
            self.run_gcode_from_file,
            font
        )

        self.run_gcode_button.setCheckable(True)

        path_layout.addWidget(self.run_gcode_button)

        tabs.addTab(path_tab, "Path / GCode")

        # TAB 3 : CAMERA
        camera_tab = QWidget()
        camera_tab.setObjectName("CameraTab")
        camera_layout = QVBoxLayout(camera_tab)

        tmp_layout, self.exposure_spinbox = self.create_spinbox(label_text="Exposure:", min_val=-50, max_val=100000,
            step=1, default=1000, decimals=0, callback=lambda: self.camera.set_exposure_time(self.exposure_spinbox.value()),
            update_immediately=True
        )
        camera_layout.addLayout(tmp_layout)

        tmp_layout, self.gain_spinbox = self.create_spinbox(label_text="Gain:", min_val=-24, max_val=100,
            step=1, default=0, decimals=0, callback=lambda: self.camera.set_gain(self.gain_spinbox.value()),
            update_immediately=True
        )
        camera_layout.addLayout(tmp_layout)

        tmp_layout, self.white_balance_spinbox = self.create_spinbox(label_text="White Balance:", min_val=0, max_val=10000,
            step=100, default=3000, decimals=0, callback=lambda: self.camera.set_white_balance(self.white_balance_spinbox.value()),
            update_immediately=True
        )
        camera_layout.addLayout(tmp_layout)

        camera_layout.addItem(QSpacerItem(0, 10, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding))
        camera_layout.addWidget(self.create_button("Save Screenshot", self.save_screenshot, font))

        tabs.addTab(camera_tab, "Camera")

        #----------------------------------------------------------------------

        self.update_waypoint_info()

        main_layout.addSpacing(25)
        main_layout.addWidget(tabs)

        main_layout.addSpacing(25)
        main_layout.addWidget(self.create_button("Home", self.home, font))

        # -----------------------------------------------------
        # RIGHT PANEL (VIDEO)
        # -----------------------------------------------------

        self.video_viewer.setMinimumWidth(600)
        self.video_viewer.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.video_viewer.setStyleSheet("background-color: black; border: 2px solid #444;")

        main_h_layout.addWidget(self.tool_panel)
        main_h_layout.addWidget(self.video_viewer, stretch=1)

        self.set_stylesheet()

    # -----------------------------------------------------------
    # UI HELPERS
    # -----------------------------------------------------------

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

    def set_stylesheet(self):

        self.setStyleSheet("""
                
        QTabWidget > QStackedWidget > QWidget {
            background-color: #353535;
        }
        QTabWidget QLabel {
            background-color: #353535;
        }
        
        
        QWidget {
            background-color: #2b2b2b;
            color: white;
        }

        QPushButton {
            background-color: #2d5291;
            color: white;
            border: 1px solid #202020;
            border-radius: 3px;
            padding: 8px;
        }

        QPushButton:hover {
            background-color: #3b72d1;
        }

        QPushButton:checked {
            background-color: #32a877;
        }

        QPushButton:pressed {
            background-color: #32a877;
        }

        QWidget#AdvancedTab QPushButton { background-color: #4d5291; }
        QWidget#AdvancedTab QPushButton:hover { background-color: #5b72d1; }
        QWidget#AdvancedTab QPushButton:pressed { background-color: #32a877; }
        QWidget#AdvancedTab QPushButton:checked { background-color: #32a877; }
                
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
            border-bottom: None;
        }

        """)

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

    #-------------------------------------------------------------------------------------

    def run_fiber_alignment(self):
        aligner = OpticalAlignment(self.oms, self.camera)
        pos, _ = aligner.optimize()
        self.camera.start_grabbing(False)
        self.current_pos = pos

    def on_stop_realtime_control(self):
        self.current_pos[:] = self.oms.read_current_position(True)

    def home(self):
        self.oms.home()
        if self.oms.is_connected():
            self.current_pos = list(self.oms.read_current_position(True))

    def move_axis(self, axis, direction):
        flipped = (1, 1, 1)
        d = self.step_sizes[self.step_size_idx]
        self.current_pos[axis] += direction * d * flipped[axis]
        self.current_pos[axis] = max(min(self.current_pos[axis], 10), -10)
        self.oms.move_to(*self.current_pos, self.feedrates[self.step_size_idx])

    def add_waypoint(self):
        if self.realtime_control_widget.is_running():
            self.current_pos[:] = self.realtime_control_widget.get_current_pose()
            # self.current_pos[:] = self.oms.read_current_position(True)

        self.waypoints.append([self.current_pos.copy(), self.feedrates[self.step_size_idx]])
        self.update_waypoint_info()

    def run_path(self):
        self.waypoint_idx = 0

    def save_path(self):
        if len(self.waypoints) <= 0:
            QMessageBox.critical(self, "Save Error", f"Waypoint list is empty")
            return

        path, _ = QFileDialog.getSaveFileName(
            self, "Save Path to G-code File", "",
            "G-code Files (*.g *.gcode);;Text Files (*.txt);;All Files (*.*)"
        )

        if not path:
            return  # user cancelled

        try:
            with open(path, "w") as f:
                # Optional header
                f.write("; Generated by Open Micro Manipulator Controller\n")
                f.write("G90 ; absolute positioning\n\n")

                for waypoint in self.waypoints:
                    (x, y, z), feedrate = waypoint

                    f.write(f"G0 X{x:.10f} Y{y:.10f} Z{z:.10f} F{feedrate * 60:.3f}\n")
                    f.write(f"G4 S{0.1:.6f}\n")

                # Optional footer
                f.write("\n; End of file\n")

        except Exception as e:
            QMessageBox.critical(self, "Save Error", f"Failed to save G-code file:\n{e}")

    def set_origin(self):
        self.current_pos[:] = self.oms.read_current_position(True)
        t = self.oms.get_workspace_transform()
        t[0, 3] += self.current_pos[0]
        t[1, 3] += self.current_pos[1]
        t[2, 3] += self.current_pos[2]
        self.oms.set_workspace_transform(t)
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
        if self.waypoint_idx <= len(self.waypoints) and len(self.waypoints) > 0:
            self.current_pos[:], f = self.waypoints[self.waypoint_idx % len(self.waypoints)]
            self.oms.move_to(*self.current_pos, f)
            self.oms.dwell(0.1, False)
            self.waypoint_idx += 1
        else:
            self.waypoint_idx = 1000000

        px0, py0 = self.image_point_tracker.prev_pos
        px, py = self.image_point_tracker.update(frame)
        cv2.circle(vis_image, (px, py), 4, (0, 0, 255), thickness=-1)

        if self.draw_buffer is None:
            self.draw_buffer = np.zeros_like(frame)
        else:
            # cv2.circle(self.draw_buffer, (px, py), radius=0, color=(255,255,255), thickness=1)
            cv2.line(self.draw_buffer, (px0, py0), (px, py), color=(255, 255, 255), thickness=1)
            mask = self.draw_buffer[:, :, 0] != 0
            vis_image[mask, :] = np.array((0, 255, 100), dtype=np.uint8)
            # cv2.subtract(vis_image, self.draw_buffer, vis_image)

        self.video_viewer.set_image(vis_image, pixel_per_mm=pixel_per_mm)

        self.last_frame = frame
        return vis_image

    def set_tracking_point(self):
        if self.last_frame is None:
            return

        self.image_point_tracker.set_track_point(self.last_frame,
                                                 self.last_frame.shape[1] // 2,
                                                 self.last_frame.shape[0] // 2)

    def clear_draw_buffer(self):
        if self.draw_buffer is not None:
            self.draw_buffer = None
            self.image_point_tracker.reset()

    def run_gcode_from_file(self, checked):
        if checked:
            path, _ = QFileDialog.getOpenFileName(self, "Open G-code File",
                                                  "", "G-code Files (*.g *.gcode);;Text Files (*.txt);;All Files (*.*)")

            if not path:
                return
            try:
                gcode = open(path, 'r').read()
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to open file:\n{e}")
                return

            self.gcode_runner = GCodeRunner(gcode, self.oms, max_feedrate=0.5)

            def on_finished():
                self.gcode_runner = None
                self.run_gcode_button.blockSignals(True)
                self.run_gcode_button.setChecked(False)
                self.run_gcode_button.blockSignals(False)

            def on_iteration_finished():
                # self.draw_buffer = None
                pass

            # start gcode runner here
            self.gcode_runner.run(on_finished, on_iteration_finished, loop_playback=False)
        elif self.gcode_runner is not None:
            self.gcode_runner.stop()

    def run_3point_alignment(self):
        if len(self.waypoints) != 3:
            QMessageBox.critical(self, "Error", "Exactly 3 Waypoints need to be recorded for this function")
            return

        old_workspace_transform = self.oms.get_workspace_transform()

        p0 = np.array(self.waypoints[0][0])
        p1 = np.array(self.waypoints[1][0])
        p2 = np.array(self.waypoints[2][0])

        # Step 1: Compute Z-axis (normal to the plane)
        v1 = p1 - p0
        v2 = p2 - p0
        z_axis = np.cross(v1, v2)
        if z_axis[2] < 0.0:
            z_axis = -z_axis
        z_axis /= np.linalg.norm(z_axis)

        # Step 2: Project global X-axis onto the plane to get the local X-axis
        global_x = np.array([1.0, 0.0, 0.0])
        x_proj = global_x - np.dot(global_x, z_axis) * z_axis
        x_axis = x_proj / np.linalg.norm(x_proj)

        # Step 3: Compute Y-axis to complete right-handed system
        y_axis = np.cross(z_axis, x_axis)
        y_axis /= np.linalg.norm(y_axis)

        # Step 4: Build 4x4 transform matrix (from local plane frame to base frame)
        T = np.eye(4)
        T[:3, 0] = x_axis
        T[:3, 1] = y_axis
        T[:3, 2] = z_axis
        T[:3, 3] = p0

        # Step 5: set coordinate system
        self.oms.set_workspace_transform(T @ old_workspace_transform)
        QMessageBox.information(self, "Alignment Complete", "3-point alignment complete.")

        self.oms.move_to(0, 0, 0, self.feedrates[self.step_size_idx])
        self.current_pos = [0, 0, 0]

        # self.save_transform(ask=False)

    def load_transform(self):
        data = json.load(open("transform.json"))
        T = np.array(data)
        self.oms.set_workspace_transform(T)
        self.oms.move_to(0, 0, 0, self.feedrates[self.step_size_idx])
        self.current_pos = [0, 0, 0]

    def save_transform(self, pressed=True, ask=True):
        if ask:
            confirmed = QMessageBox.question(None, "Save Transform", "Are you sure?",
                                             QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                                             QMessageBox.StandardButton.No) == QMessageBox.StandardButton.Yes
            if not confirmed: return

        T = self.oms.get_workspace_transform()
        json.dump(T.tolist(), open("transform.json", "w"))

    def save_screenshot(self):
        if self.last_frame is not None:
            os.makedirs("./screenshots", exist_ok=True)

            # Create filename with date and time
            timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            filename = f"screenshot_{timestamp}.png"

            # save image
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