# --------------------------------------------------------------------------------------
# Project: OpenMicroManipulator
# License: MIT (see LICENSE file for full description)
#          All text in here must be included in any redistribution.
# Author:  M. S. (diffraction limited)
# --------------------------------------------------------------------------------------

from hardware.open_micro_stage_api import OpenMicroStageInterface
from PySide6.QtCore import QObject, Signal
import re
import time
import threading
import math

class GCodeRunner(QObject):
    progress_updated = Signal(float)  # progress fraction (0.0-1.0), safe to connect across threads

    def __init__(self, gcode: str, oms: OpenMicroStageInterface, max_feedrate, scale=0.001):
        """
        :param gcode: Full G-code string
        :param oms: the serial interface to the device
        :param scale: Scale factor for unit conversion
        """
        super().__init__()
        self.lines = gcode.splitlines()
        self.current_line_idx = 0
        self.oms = oms
        self.initial_scale = scale

        self.max_feedrate = max_feedrate
        self.state = [0.0, 0.0, 0.0, 1.0]  # x, y, z, f

        self.tool_nr = 0           # tool used as the binary print nozzle
        self.tool_power = 1.0      # power value sent when the nozzle should extrude
        self.extrude_relative = False  # M82 (absolute, default) / M83 (relative)
        self.last_e = 0.0          # last absolute extruder position
        self.tool_on = False       # current binary nozzle state

        self.running = False
        self.thread = None
        self.on_finished = None  # Optional callback
        self.on_iteration_finished = None  # Optional callback
        self.gcode_scale_factor = [self.initial_scale]*4  # x, y, z, s


    def update(self):
        """
        Process a single G-code command.
        Returns True if a command was processed, False if no more commands.
        """
        if not self.oms.is_connected():
            return True

        def parse_gcode_line(line):
            matches = re.findall(r'([XYZEFS])([-+]?[0-9]*\.?[0-9]+)', line.upper())
            return {k: float(v) for k, v in matches}

        def set_tool_on(turn_on):
            if turn_on == self.tool_on:
                return
            self.tool_on = turn_on
            self.oms.set_tool_output(self.tool_nr, self.tool_power if turn_on else 0.0, False)

        while self.current_line_idx < len(self.lines):
            line = self.lines[self.current_line_idx].strip()
            self.current_line_idx += 1

            if line.startswith('SCALE'):
                args = parse_gcode_line(line)
                if args:
                    self.gcode_scale_factor = [
                        args.get('X', self.gcode_scale_factor[0]),
                        args.get('Y', self.gcode_scale_factor[1]),
                        args.get('Z', self.gcode_scale_factor[2]),
                        args.get('F', self.gcode_scale_factor[3]),
                    ]
                else:
                    value = float(line.split('SCALE', 1)[1].lstrip('= '))
                    self.gcode_scale_factor = [value] * 4
                print('G-Code scale:', self.gcode_scale_factor)
                continue

            if line.startswith('G4'):
                args = parse_gcode_line(line)
                wait_time = args.get('S', 0.1)
                self.oms.dwell(wait_time, blocking=True, timeout=5)

            if line.startswith('M82'):
                self.extrude_relative = False

            if line.startswith('M83'):
                self.extrude_relative = True

            if line.startswith('G92'):
                args = parse_gcode_line(line)
                if 'E' in args:
                    self.last_e = args['E']

            if line.startswith('M3'):
                args = parse_gcode_line(line)
                tool_nr = int(args.get('T', 0))
                tool_value = args.get('S', 0.0)
                self.oms.set_tool_output (tool_nr, tool_value, False)

            if line.startswith(('G0', 'G1')):
                args = parse_gcode_line(line)
                s = (args.get('X', self.state[0]),
                     args.get('Y', self.state[1]),
                     args.get('Z', self.state[2]),
                     args.get('F', self.state[3]))

                if 'E' in args:
                    if self.extrude_relative:
                        delta_e = args['E']
                    else:
                        delta_e = args['E'] - self.last_e
                        self.last_e = args['E']
                    set_tool_on(delta_e > 0)

                command_accepted = self.oms.move_to(s[0]*self.gcode_scale_factor[0],
                                                    s[1]*self.gcode_scale_factor[1],
                                                    s[2]*self.gcode_scale_factor[2],
                                                    min(s[3]/60*self.gcode_scale_factor[3], self.max_feedrate),
                                                    blocking=True, timeout=5)
                if command_accepted:
                    self.state = s
                else:
                    print(f'Line not accepted: {line}')

                # time.sleep(0.01)
                return False # not finished yet

        # self.serial_interface.move_to(0,0,0,1)
        print("G-Code finished")
        return True # finished
    
    def stop(self):
        self.running = False

    def run(self, on_finished=None, on_iteration_finished=None, loop_playback=False, tool_power=0.0):
        """
        Run G-code processing in a background thread.
        Optionally pass a callback to be called when finished.
        :param tool_power: power value sent when the nozzle should extrude (e.g. from the GUI tool spinbox).

        Progress updates are emitted via the progress_updated signal (connect to it before calling run()).
        """
        if self.running:
            print("GCodeRunner is already running.")
            return

        if any(line.strip().startswith('G91') for line in self.lines):
            print("ERROR: G-code contains G91 (relative positioning), which is not supported. "
                  "X/Y/Z moves are always interpreted as absolute.")
            return

        # self.diagnose_gcode(0.1e-3, 50e-3)
        # return

        self.running = True
        self.on_finished = on_finished
        self.on_iteration_finished = on_iteration_finished
        self.tool_power = tool_power
        self.gcode_scale_factor = [self.initial_scale] * 4

        def loop():
            while self.running:
                finished = self.update()
                if self.lines:
                    self.progress_updated.emit(self.current_line_idx / len(self.lines))
                if finished:
                    self.oms.wait_for_stop(100)
                    if self.on_iteration_finished:
                        self.on_iteration_finished()

                    self.current_line_idx = 0 # reset playback index
                    if not loop_playback:
                        self.running = False
                        break
                time.sleep(0.005)  # Control update rate

            if self.on_finished:
                self.on_finished()

            # disable all tools
            self.oms.set_tool_output(0, 0.0, True);
            self.oms.set_tool_output(1, 0.0, True);
            print("G-Code runner stopped")

        self.thread = threading.Thread(target=loop, daemon=True)
        self.thread.start()


    def diagnose_gcode(self, min_length=0.1, min_feedrate=50):
        """
        Scan the loaded G-code for short move segments and slow feedrates,
        applying SCALE directives the same way update() does.

        :param min_length: moves shorter than this (in scaled units) are flagged
        :param min_feedrate: moves with a feedrate (scaled units/s) below this are
            flagged; pass None to skip the feedrate check
        :return: list of dicts with keys 'line', 'text', and optionally
            'length' and/or 'feedrate' for whichever thresholds were tripped
        """
        def parse_gcode_line(line):
            matches = re.findall(r'([XYZEFS])([-+]?[0-9]*\.?[0-9]+)', line.upper())
            return {k: float(v) for k, v in matches}

        issues = []
        state = [0.0, 0.0, 0.0, 1.0]
        scale = [self.initial_scale] * 4

        for line_nr, raw_line in enumerate(self.lines, start=1):
            line = raw_line.strip()

            if line.startswith('SCALE'):
                args = parse_gcode_line(line)
                if args:
                    scale = [
                        args.get('X', scale[0]),
                        args.get('Y', scale[1]),
                        args.get('Z', scale[2]),
                        args.get('F', scale[3]),
                    ]
                else:
                    value = float(line.split('SCALE', 1)[1].lstrip('= '))
                    scale = [value] * 4
                continue

            if not line.startswith(('G0', 'G1')):
                continue

            args = parse_gcode_line(line)
            if not ('X' in args or 'Y' in args or 'Z' in args):
                continue  # no actual move, e.g. a feedrate-only or extrude-only line

            target = (args.get('X', state[0]),
                      args.get('Y', state[1]),
                      args.get('Z', state[2]),
                      args.get('F', state[3]))

            dx = (target[0] - state[0]) * scale[0]
            dy = (target[1] - state[1]) * scale[1]
            dz = (target[2] - state[2]) * scale[2]
            length = math.sqrt(dx * dx + dy * dy + dz * dz)
            feedrate = target[3] / 60 * scale[3]

            issue = {}
            if length < min_length:
                issue['length'] = length
            if min_feedrate is not None and feedrate < min_feedrate:
                issue['feedrate'] = feedrate

            if issue:
                issues.append({'line': line_nr, 'text': line, **issue})

            state = list(target)

        for issue in issues:
            parts = [f"line {issue['line']}: {issue['text']}"]
            if 'length' in issue:
                parts.append(f"length={issue['length']:.5f}")
            if 'feedrate' in issue:
                parts.append(f"feedrate={issue['feedrate']:.5f}")
            print(" | ".join(parts))

        return issues
