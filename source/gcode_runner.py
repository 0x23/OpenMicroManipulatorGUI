# --------------------------------------------------------------------------------------
# Project: OpenMicroManipulator
# License: MIT (see LICENSE file for full description)
#          All text in here must be included in any redistribution.
# Author:  M. S. (diffraction limited)
# --------------------------------------------------------------------------------------

from hardware.open_micro_stage_api import OpenMicroStageInterface
import re
import time
import threading

class GCodeRunner:
    def __init__(self, gcode: str, oms: OpenMicroStageInterface, max_feedrate, scale=1.0):
        """
        :param gcode: Full G-code string
        :param oms: the serial interface to the device
        :param scale: Scale factor for unit conversion
        """
        self.lines = gcode.splitlines()
        self.current_line_idx = 0
        self.oms = oms
        self.scale = (scale, scale, 0.0)
        self.max_feedrate = max_feedrate
        self.state = [0.0, 0.0, 0.0, 1.0]  # x, y, z, f

        self._running = False
        self._thread = None
        self._on_finished = None  # Optional callback
        self.on_iteration_finished = None  # Optional callback
        self.gcode_scale_factor = 1.0


    def update(self):
        """
        Process a single G-code command.
        Returns True if a command was processed, False if no more commands.
        """
        if not self.oms.is_connected():
            return True

        def parse_gcode_line(line):
            matches = re.findall(r'([XYZF])([-+]?[0-9]*\.?[0-9]+)', line.upper())
            return {k: float(v) for k, v in matches}

        while self.current_line_idx < len(self.lines):
            line = self.lines[self.current_line_idx].strip()
            self.current_line_idx += 1

            if line.startswith('SCALE='):
                self.gcode_scale_factor = float(line.split('=', 1)[1])
                print('G-Code scale:', self.gcode_scale_factor)
                continue

            if line.startswith('G4'):
                args = parse_gcode_line(line)
                wait_time = args.get('S', 0.1)
                self.oms.dwell(wait_time, blocking=True, timeout=5)

            if line.startswith(('G0', 'G1')):
                args = parse_gcode_line(line)
                s = (args.get('X', self.state[0]),
                     args.get('Y', self.state[1]),
                     args.get('Z', self.state[2]),
                     args.get('F', self.state[3]))

                command_accepted = self.oms.move_to(s[0]*self.scale[0]*self.gcode_scale_factor,
                                                    s[1]*self.scale[1]*self.gcode_scale_factor,
                                                    s[2]*self.scale[2]*self.gcode_scale_factor,
                                                    min(s[3]/60*self.gcode_scale_factor, self.max_feedrate),
                                                    blocking=True, timeout=5)
                if command_accepted:
                    self.state = s

                time.sleep(0.01)
                return False # not finished yet

        # self.serial_interface.move_to(0,0,0,1)
        print("G-Code finished")
        return True # finished

    def stop(self):
        self._running = False

    def run(self, on_finished=None, on_iteration_finished=None, loop_playback=False):
        """
        Run G-code processing in a background thread.
        Optionally pass a callback to be called when finished.
        """
        if self._running:
            print("GCodeRunner is already running.")
            return

        self._running = True
        self._on_finished = on_finished
        self.on_iteration_finished = on_iteration_finished
        self.gcode_scale_factor = 1.0

        def loop():
            while self._running:
                finished = self.update()
                if finished:
                    self.oms.wait_for_stop(100)
                    if self.on_iteration_finished:
                        self.on_iteration_finished()

                    self.current_line_idx = 0 # reset playback index
                    if not loop_playback:
                        self._running = False
                        break
                time.sleep(0.005)  # Control update rate

            if self._on_finished:
                self._on_finished()

        self._thread = threading.Thread(target=loop, daemon=True)
        self._thread.start()

