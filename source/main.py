# --------------------------------------------------------------------------------------
# Project: OpenMicroManipulator
# License: MIT (see LICENSE file for full description)
#          All text in here must be included in any redistribution.
# Author:  M. S. (diffraction limited)
# --------------------------------------------------------------------------------------

import os

# Disable scaling
os.environ['QT_SCALE_FACTOR'] = '1'
os.environ['QT_AUTO_SCREEN_SCALE_FACTOR'] = '0'
os.environ['GDK_SCALE'] = '1'
os.environ['GDK_DPI_SCALE'] = '1'

from PySide6.QtWidgets import QApplication

from hardware.open_micro_stage_api import OpenMicroStageInterface
from mainwindow import DeviceControlMainWindow


def main():
    oms = OpenMicroStageInterface(show_communication=False, show_log_messages=True)
    app = QApplication()
    gui = DeviceControlMainWindow(oms)
    gui.show()
    app.exec()

if __name__ == "__main__":
    main()
