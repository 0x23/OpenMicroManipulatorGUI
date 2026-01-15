# --------------------------------------------------------------------------------------
# Project: OpenMicroManipulator
# License: MIT (see LICENSE file for full description)
#          All text in here must be included in any redistribution.
# Author:  M. S. (diffraction limited)
# --------------------------------------------------------------------------------------

import math
from PySide6.QtWidgets import QGraphicsView, QGraphicsScene, QGraphicsPixmapItem
from PySide6.QtGui import QPixmap, QImage, QPainter, QMouseEvent, QPen, QColor, QKeyEvent, QBrush
from PySide6.QtCore import Qt, QCoreApplication


class ImageViewerWidget(QGraphicsView):
    def __init__(self):
        super().__init__()

        self.pixel_per_mm = 1.0  # Base calibration

        # 1. Setup Scene and Item
        self._scene = QGraphicsScene(self)
        self._pixmap_item = QGraphicsPixmapItem()
        self._scene.addItem(self._pixmap_item)
        self.setScene(self._scene)
        self._image_size = (0, 0)

        # 2. Appearance & Rendering
        self.setBackgroundBrush(Qt.black)
        self.setFrameShape(QGraphicsView.NoFrame)
        self.setRenderHint(QPainter.SmoothPixmapTransform, False)
        self.setRenderHint(QPainter.Antialiasing, True)
        self.setRenderHint(QPainter.TextAntialiasing, True)
        self.setViewportUpdateMode(QGraphicsView.FullViewportUpdate)

        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setInteractive(True)

        # Enable key focus so we can catch key events
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

    def set_image(self, cv_img, pixel_per_mm):
        height, width, channel = cv_img.shape
        bytes_per_line = channel * width
        q_img = QImage(cv_img.data, width, height, bytes_per_line, QImage.Format_RGB888)

        self.pixel_per_mm = pixel_per_mm
        self._pixmap_item.setPixmap(QPixmap.fromImage(q_img))
        self.setSceneRect(0, 0, width, height)

        if self._image_size[0] != width or self._image_size[1] != height:
            self.fit_view()

        self._image_size = (width, height)

    def resizeEvent(self, event):
        self.fit_view()
        super().resizeEvent(event)

    def draw_center_cross(self, painter, line_length=40, color=(10, 220, 50), thickness=1, gap=10):
        width, height = self.viewport().width(), self.viewport().height()
        cx, cy = width // 2, height // 2
        q_color = QColor(color[2], color[1], color[0], 100)
        pen = QPen(q_color)
        pen.setWidth(thickness)
        painter.setPen(pen)
        painter.drawLine(cx - line_length, cy, cx - gap, cy)
        painter.drawLine(cx + gap, cy, cx + line_length, cy)
        painter.drawLine(cx, cy - line_length, cx, cy - gap)
        painter.drawLine(cx, cy + gap, cx, cy + line_length)
        painter.drawPoint(cx, cy)

    def draw_scale_bar(self, painter, current_px_per_mm, target_px_width=200):
        """
        Draws a scale bar with a constant pixel width,
        supporting mm, um, and nm steps.
        """
        # 1. Determine the real-world length that fits our target pixel width
        raw_length_mm = target_px_width / current_px_per_mm

        # 2. Define "nice" steps in Nanometers (nm)
        # 1 mm = 1,000,000 nm
        raw_nm = raw_length_mm * 1_000_000

        nice_steps_nm = [1, 2, 5, 10]
        nice_steps_nm = [t*math.pow(10, s) for s in range(7) for t in nice_steps_nm]

        # Pick the largest nice step that fits within our target width
        actual_nm = 1
        for step in nice_steps_nm:
            if step <= raw_nm:
                actual_nm = step
            else:
                break

        # Convert nm back to mm for coordinate calculation
        actual_mm = actual_nm / 1_000_000.0
        bar_px = int(actual_mm * current_px_per_mm)

        # 3. Determine Subdivision count (Ticks)
        if actual_nm % 5 == 0:
            num_sub = 5
        elif actual_nm % 4 == 0:
            num_sub = 4
        else:
            num_sub = 2

        # 4. Positioning (Bottom Right)
        margin = 40
        view_w, view_h = self.viewport().width(), self.viewport().height()
        x_start = view_w - margin - bar_px
        y_start = view_h - margin

        # 5. Drawing
        painter.setPen(QPen(Qt.white, 2, Qt.SolidLine, Qt.FlatCap))

        # Main bar
        painter.drawLine(x_start, y_start, x_start + bar_px, y_start)
        # End caps
        painter.drawLine(x_start, y_start - 5, x_start, y_start + 5)
        painter.drawLine(x_start + bar_px, y_start - 5, x_start + bar_px, y_start + 5)

        # Ticks
        sub_px = bar_px / num_sub
        for i in range(1, num_sub):
            tx = int(x_start + i * sub_px)
            painter.drawLine(tx, y_start, tx, y_start + 3)

        # 6. Dynamic Label Formatting
        if actual_nm < 1000:
            text = f"{actual_nm} nm"
        elif actual_nm < 1_000_000:
            text = f"{actual_nm / 1000:g} µm"
        else:
            text = f"{actual_nm / 1_000_000:g} mm"

        metrics = painter.fontMetrics()
        text_w = metrics.horizontalAdvance(text)
        painter.drawText(x_start + (bar_px - text_w) // 2, y_start - 8, text)

    def drawForeground(self, painter, rect):
        super().drawForeground(painter, rect)
        painter.save()
        painter.resetTransform()

        self.draw_center_cross(painter)

        # current_zoom = scale factor
        current_zoom = self.transform().m11()
        adjusted_px_per_mm = self.pixel_per_mm * current_zoom

        # We tell it to try to be ~200 pixels wide on screen
        self.draw_scale_bar(painter, adjusted_px_per_mm, target_px_width=200)

        painter.restore()

    def wheelEvent(self, event):
        zoom_in_factor = 1.1
        zoom_out_factor = 1 / zoom_in_factor
        rect = self.transform().mapRect(self._scene.sceneRect())
        view_rect = self.viewport().rect()

        if rect.width() < 1 or rect.height() < 1:
            return

        if event.angleDelta().y() > 0:
            if rect.width()/view_rect.width() < 10 and rect.height()/view_rect.height() < 10:
                self.scale(zoom_in_factor, zoom_in_factor)
        else:
            if rect.width()*zoom_out_factor > view_rect.width() or rect.height()*zoom_out_factor > view_rect.height():
                self.scale(zoom_out_factor, zoom_out_factor)
            else:
                zf = min(view_rect.width()/rect.width(), view_rect.height()/rect.height())
                self.scale(zf, zf)

        self.viewport().update()

    def fit_view(self):
        rect = self.transform().mapRect(self._scene.sceneRect())
        view_rect = self.viewport().rect()
        if rect.width() > 0 and rect.height() > 0:
           zf = min(view_rect.width() / rect.width(), view_rect.width() / rect.height())
           self.scale(zf, zf)

    def mousePressEvent(self, event):
        if event.button() == Qt.RightButton:
            self.setDragMode(QGraphicsView.ScrollHandDrag)
            fake_event = QMouseEvent(event.type(), event.pos(), Qt.LeftButton, Qt.LeftButton, event.modifiers())
            super().mousePressEvent(fake_event)
        else:
            super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.RightButton:
            fake_event = QMouseEvent(event.type(), event.pos(), Qt.LeftButton, Qt.LeftButton, event.modifiers())
            super().mouseReleaseEvent(fake_event)
            self.setDragMode(QGraphicsView.NoDrag)
        else:
            super().mouseReleaseEvent(event)

    # --------------------------
    # Key forwarding
    # --------------------------
    def keyPressEvent(self, event):
        # Forward to viewport
        if event.spontaneous():
            self.forward_key_to_viewport(event)
        else:
            super().keyPressEvent(event)

    def keyReleaseEvent(self, event: QKeyEvent):
        # Forward to viewport
        if event.spontaneous():
            self.forward_key_to_viewport(event)
        else:
            super().keyReleaseEvent(event)

    def forward_key_to_viewport(self, event: QKeyEvent):
        """
        Forwards a key event (press or release) to the viewport widget.
        """
        key_event_copy = QKeyEvent(
            event.type(),
            event.key(),
            event.modifiers(),
            event.text(),
            event.isAutoRepeat(),
            event.count()
        )
        QCoreApplication.sendEvent(self.viewport(), key_event_copy)
