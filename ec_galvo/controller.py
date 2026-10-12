"""Galvo controller for JCZ LMC (EZCAD2-era) boards driving a MOPA fiber source.

Trimmed from MeerK40t meerk40t/balormk/controller.py (MIT). Kept: the command
sequencing (list packets vs single commands, execute after the third packet),
board start-up, mark/jump/light, pause/resume/abort, and the fiber conversions
for power, frequency and MOPA pulse width.

Changed on purpose:
- No MeerK40t kernel/service. Settings come in as plain dataclasses.
- Connection is opened explicitly by connect(); no auto-connect, no clone-board
  firmware load, no silent reconnect mid-job.
- Moves outside the field raise instead of being skipped.
- A missing or bad lens correction raises instead of loading a blank table.
- The log carries state changes only. It never carries coordinates or packets.
- Waits time out instead of spinning forever.
- Fiber source only (CO2/UV paths removed).
"""

from __future__ import annotations

import struct
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass

from . import protocol as p
from .connection import Connection
from .correction import CorrectionTable
from .units import GALVO_RANGE, Field

STATE_RAPID = "rapid"
STATE_LIGHT = "light"
STATE_PROGRAM = "program"

_NOP = struct.pack("<6H", p.listEndOfList, 0, 0, 0, 0, 0)
_EMPTY_LIST = _NOP * (p.PACKET_LIST // 12)

LIGHT_BIT = 8  # red-dot pointer output pin
MAIN_BIT = 0


@dataclass(frozen=True)
class BoardSetup:
    """Values sent once when the board is opened. Defaults are MeerK40t's."""

    first_pulse_killer: int = 200
    pwm_half_period: int = 125
    pwm_pulse_width: int = 125
    standby: tuple[int, int] = (2000, 20)
    timing_mode: int = 1
    delay_mode: int = 1
    laser_mode: int = 1
    control_mode: int = 0
    # MeerK40t declares p4 = 100 but sends p3 for both (upstream typo). We send
    # what MeerK40t actually sends, so phase 0 results carry over unchanged.
    fpk2: tuple[int, int, int, int] = (0xFFB, 1, 409, 409)
    fly_res: tuple[int, int, int, int] = (0, 99, 1000, 25)
    open_mo_delay_ms: float = 8.0
    analog_port_1: int = 0x7FF
    default_jump_speed_mm_s: float = 2000.0


@dataclass(frozen=True)
class MarkSettings:
    """Per-layer settings, already in physical units."""

    power_pct: float
    speed_mm_s: float
    frequency_khz: float
    pulse_width_ns: int
    laser_on_delay_us: float
    laser_off_delay_us: float
    polygon_delay_us: float
    jump_speed_mm_s: float
    end_delay_us: float = 300.0


class GalvoController:
    def __init__(
        self,
        connection: Connection,
        field: Field,
        correction: CorrectionTable,
        setup: BoardSetup | None = None,
        expected_serial: bytes | None = None,
        log: Callable[[str], None] | None = None,
        wait_timeout_s: float = 30.0,
    ):
        self.connection = connection
        self.field = field
        self.correction = correction
        self.setup = setup or BoardSetup()
        self.expected_serial = expected_serial
        self._log = log or (lambda msg: None)
        self.wait_timeout_s = wait_timeout_s

        self.mode = STATE_RAPID
        self.paused = False
        self.serial_number: bytes | None = None

        self._last_x = 0x8000
        self._last_y = 0x8000
        self._port_bits = 0
        self._list_lock = threading.RLock()
        self._send_lock = threading.RLock()
        self._active_list: bytearray | None = None
        self._active_index = 0
        self._list_executing = False
        self._number_of_list_packets = 0
        self._forget_settings()

    def _forget_settings(self):
        # Cached values let us skip resending unchanged settings. Every new list
        # starts with none cached, so the first layer always sends everything.
        self._speed = None
        self._travel_speed = None
        self._frequency = None
        self._power = None
        self._pulse_width = None
        self._delay_on = None
        self._delay_off = None
        self._delay_poly = None

    #######################
    # CONNECTION
    #######################

    def connect(self) -> None:
        self.connection.open()
        try:
            self._init_board()
        except Exception:
            self.connection.close()
            raise

    def disconnect(self) -> None:
        self.connection.close()

    def _send(self, data: bytes, read: bool = True):
        if not self.connection.is_open():
            raise ConnectionError("board not connected")
        with self._send_lock:
            self.connection.write(data)
            if read:
                return self.connection.read()
        return None

    def _command(self, command, v1=0, v2=0, v3=0, v4=0, v5=0, read=True):
        cmd = struct.pack("<6H", int(command), int(v1), int(v2), int(v3), int(v4), int(v5))
        r = self._send(cmd, read=read)
        return struct.unpack("<4H", r) if read else None

    def _init_board(self):
        s = self.setup
        self.serial_number = self.get_serial_number()
        if self.expected_serial is not None and self.serial_number != self.expected_serial:
            raise ConnectionRefusedError("board serial number does not match the machine file")
        self.get_version()
        self.reset()
        self._write_correction(self.correction)
        self.enable_laser()
        self.set_control_mode(s.control_mode)
        self.set_laser_mode(s.laser_mode)
        self.set_delay_mode(s.delay_mode)
        self.set_timing(s.timing_mode)
        self.set_standby(*s.standby)
        self.set_first_pulse_killer(s.first_pulse_killer)
        self.set_pwm_half_period(s.pwm_half_period)
        self.set_pwm_pulse_width(s.pwm_pulse_width)
        self.set_fiber_mo(0)
        self.set_fpk_param_2(*s.fpk2)
        self.set_fly_res(*s.fly_res)
        self.enable_z()
        self.write_analog_port_1(s.analog_port_1)
        self.enable_z()
        time.sleep(0.05)
        self._log("board ready")

    def _write_correction(self, table: CorrectionTable):
        if table.blank:
            self.write_cor_table(False)
            self._log("lens correction: none (explicitly requested)")
            return
        self.write_cor_table(True)
        for i, (dx, dy) in enumerate(table.entries):
            self._command(p.WriteCorLine, dx, dy, 0 if i == 0 else 1, read=False)
        self._log("lens correction loaded")

    #######################
    # STATUS AND WAITS
    #######################

    def status(self) -> int:
        return self.get_version()[3]

    def is_busy(self) -> bool:
        return bool(self.status() & p.BUSY)

    def is_ready(self) -> bool:
        return bool(self.status() & p.READY)

    def _wait(self, done: Callable[[], bool], what: str):
        deadline = time.monotonic() + self.wait_timeout_s
        while not done():
            if time.monotonic() > deadline:
                raise TimeoutError(f"board did not become {what}")
            time.sleep(0.01)

    def wait_ready(self):
        self._wait(self.is_ready, "ready")

    def wait_idle(self):
        self._wait(lambda: not self.is_busy(), "idle")

    #######################
    # MODES
    #######################

    def program_mode(self):
        """Start a marking list."""
        if self.mode == STATE_PROGRAM:
            return
        if self.mode == STATE_LIGHT:
            self.mode = STATE_PROGRAM
            self._light(False)
            self._port(MAIN_BIT, True)
            self.write_port()
            self.set_fiber_mo(1)
            return
        self.mode = STATE_PROGRAM
        self.reset_list()
        self._port(MAIN_BIT, True)
        self.write_port()
        self.set_fiber_mo(1)
        self._forget_settings()
        self.list_ready()
        if self.setup.open_mo_delay_ms:
            self.list_delay_time(int(self.setup.open_mo_delay_ms * 100))
        self.list_write_port()
        self.list_jump_speed(self.setup.default_jump_speed_mm_s)

    def light_mode(self):
        """Start a red-light framing list."""
        if self.mode == STATE_LIGHT:
            return
        if self.mode == STATE_PROGRAM:
            self.set_fiber_mo(0)
            self._port(MAIN_BIT, False)
            self._port(LIGHT_BIT, True)
            self.write_port()
        else:
            self._forget_settings()
            self.reset_list()
            self.list_ready()
            self._port(MAIN_BIT, False)
            self._port(LIGHT_BIT, True)
            self.list_write_port()
        self.mode = STATE_LIGHT

    def rapid_mode(self):
        """Finish the current list, wait for it to run out, return to idle."""
        if self.mode == STATE_RAPID:
            return
        self.list_end_of_list()
        self._list_end()
        if not self._list_executing and self._number_of_list_packets:
            self.execute_list()
        self._list_executing = False
        self._number_of_list_packets = 0
        self.wait_idle()
        self.set_fiber_mo(0)
        self._port(MAIN_BIT, False)
        self.write_port()
        self.mode = STATE_RAPID

    def abort(self):
        self.stop_execute()
        self.paused = False
        self.set_fiber_mo(0)
        self.reset_list()
        self._list_new()
        self.list_end_of_list()
        self._list_end()
        if not self._list_executing:
            self.execute_list()
        self._list_executing = False
        self._number_of_list_packets = 0
        self.set_fiber_mo(0)
        self._port(MAIN_BIT, False)
        self.write_port()
        self.mode = STATE_RAPID
        self._log("aborted")

    def pause(self):
        self.paused = True
        self.stop_list()
        self._log("paused")

    def resume(self):
        self.restart_list()
        self.paused = False
        self._log("resumed")

    #######################
    # LAYER SETTINGS
    #######################

    def set_settings(self, s: MarkSettings):
        """Queue one layer's settings. Unchanged values are not resent."""
        self.list_fiber_ylpm_pulse_width(int(s.pulse_width_ns))
        self.list_jump_speed(s.jump_speed_mm_s)
        self.frequency(s.frequency_khz)
        self.power(s.power_pct)
        self.list_mark_speed(s.speed_mm_s)
        self.list_laser_on_delay(s.laser_on_delay_us)
        self.list_laser_off_delay(s.laser_off_delay_us)
        self.list_polygon_delay(s.polygon_delay_us)

    def power(self, power_pct: float):
        if not 0 <= power_pct <= 100:
            raise ValueError("power must be 0-100 %")
        if self._power == power_pct:
            return
        self._power = power_pct
        self.list_mark_current(round(power_pct * 0xFFF / 100.0))

    def frequency(self, frequency_khz: float):
        if frequency_khz <= 0:
            raise ValueError("frequency must be positive")
        if self._frequency == frequency_khz:
            return
        self._frequency = frequency_khz
        self.list_qswitch_period(round(20000.0 / frequency_khz) & 0xFFFF)

    #######################
    # MOVES (mm)
    #######################

    def jump_mm(self, x: float, y: float):
        self.goto(*self.field.to_board(x, y))

    def mark_mm(self, x: float, y: float):
        self.mark(*self.field.to_board(x, y))

    def light_mm(self, x: float, y: float):
        self.light(*self.field.to_board(x, y))

    #######################
    # MOVES (board units)
    #######################

    @staticmethod
    def _check(x, y):
        if not (0 <= x <= GALVO_RANGE and 0 <= y <= GALVO_RANGE):
            raise ValueError("move outside board range")

    def mark(self, x: int, y: int):
        self._check(x, y)
        if (x, y) == (self._last_x, self._last_y):
            return
        self.list_mark(x, y)

    def goto(self, x: int, y: int):
        self._check(x, y)
        if (x, y) == (self._last_x, self._last_y):
            return
        self.list_jump(x, y)

    def light(self, x: int, y: int):
        """Jump with the red dot on (framing)."""
        self._check(x, y)
        if self._light(True):
            self.list_write_port()
        self.list_jump(x, y)

    def dark(self, x: int, y: int):
        """Jump with the red dot off."""
        self._check(x, y)
        if self._light(False):
            self.list_write_port()
        self.list_jump(x, y)

    def get_last_xy(self) -> tuple[int, int]:
        return self._last_x, self._last_y

    #######################
    # PORT BITS
    #######################

    def _port(self, bit: int, on: bool):
        if on:
            self._port_bits |= 1 << bit
        else:
            self._port_bits &= ~(1 << bit)

    def _light(self, on: bool) -> bool:
        """Set the red-dot bit. Returns True if it changed."""
        was = bool(self._port_bits & (1 << LIGHT_BIT))
        self._port(LIGHT_BIT, on)
        return was != on

    #######################
    # LIST PACKETS
    #######################

    def _list_new(self):
        with self._list_lock:
            self._active_list = bytearray(_EMPTY_LIST)
            self._active_index = 0

    def _list_end(self):
        with self._list_lock:
            if not self._active_list or not self._active_index:
                return
            self.wait_ready()
            while self.paused:
                time.sleep(0.3)
            self._send(bytes(self._active_list), read=False)
            self.set_end_of_list(0)
            self._number_of_list_packets += 1
            self._active_list = None
            self._active_index = 0
            if self._number_of_list_packets > 2 and not self._list_executing:
                self.execute_list()
                self._list_executing = True

    def _list_write(self, command, v1=0, v2=0, v3=0, v4=0, v5=0):
        with self._list_lock:
            if self._active_index >= p.PACKET_LIST:
                self._list_end()
            if self._active_list is None:
                self._list_new()
            i = self._active_index
            self._active_list[i : i + 12] = struct.pack(
                "<6H", int(command), int(v1), int(v2), int(v3), int(v4), int(v5)
            )
            self._active_index += 12

    #######################
    # LIST COMMANDS
    #######################

    def _distance(self, x, y) -> int:
        return min(0xFFFF, int(abs(complex(x, y) - complex(self._last_x, self._last_y))))

    def list_jump(self, x: int, y: int):
        self._list_write(p.listJumpTo, int(x), int(y), 0, self._distance(x, y))
        self._last_x, self._last_y = int(x), int(y)

    def list_mark(self, x: int, y: int):
        self._list_write(p.listMarkTo, int(x), int(y), 0, self._distance(x, y))
        self._last_x, self._last_y = int(x), int(y)

    def list_end_of_list(self):
        self._list_write(p.listEndOfList)

    def list_laser_on_point(self, dwell_10us: int):
        self._list_write(p.listLaserOnPoint, dwell_10us)

    def list_delay_time(self, delay_10us: int):
        """Delay in 10 microsecond units."""
        self._list_write(p.listDelayTime, abs(delay_10us))

    def list_jump_speed(self, mm_per_s: float):
        if self._travel_speed == mm_per_s:
            return
        self._travel_speed = mm_per_s
        self._list_write(p.listJumpSpeed, self.field.speed(mm_per_s))

    def list_mark_speed(self, mm_per_s: float):
        if self._speed == mm_per_s:
            return
        self._speed = mm_per_s
        self._list_write(p.listMarkSpeed, self.field.speed(mm_per_s))

    @staticmethod
    def _signed(delay):
        return int(abs(delay)), 0x0000 if delay >= 0 else 0x8000

    def list_laser_on_delay(self, delay_us: float):
        if self._delay_on == delay_us:
            return
        self._delay_on = delay_us
        self._list_write(p.listLaserOnDelay, *self._signed(delay_us))

    def list_laser_off_delay(self, delay_us: float):
        if self._delay_off == delay_us:
            return
        self._delay_off = delay_us
        self._list_write(p.listLaserOffDelay, *self._signed(delay_us))

    def list_jump_delay(self, delay_us: float):
        self._list_write(p.listJumpDelay, *self._signed(delay_us))

    def list_polygon_delay(self, delay_us: float):
        """Polygon delay is sent in 10 microsecond units."""
        value = int(min(65535, max(0, delay_us / 10.0)))
        if self._delay_poly == value:
            return
        self._delay_poly = value
        self._list_write(p.listPolygonDelay, value)

    def list_write_port(self):
        self._list_write(p.listWritePort, self._port_bits)

    def list_mark_current(self, current: int):
        self._list_write(p.listMarkCurrent, current)

    def list_qswitch_period(self, period: int):
        self._list_write(p.listQSwitchPeriod, period)

    def list_fiber_ylpm_pulse_width(self, pulse_width_ns: int):
        """MOPA pulse width in ns (sent as-is, as MeerK40t does)."""
        if self._pulse_width == pulse_width_ns:
            return
        self._pulse_width = pulse_width_ns
        self._list_write(p.listFiberYLPMPulseWidth, pulse_width_ns)

    def list_ready(self):
        self._list_write(p.listReadyMark)

    #######################
    # SINGLE COMMANDS
    #######################

    def enable_laser(self):
        return self._command(p.EnableLaser)

    def disable_laser(self):
        return self._command(p.DisableLaser)

    def execute_list(self):
        return self._command(p.ExecuteList)

    def set_pwm_pulse_width(self, v):
        return self._command(p.SetPwmPulseWidth, v)

    def get_version(self):
        return self._command(p.GetVersion)

    def get_serial_number(self) -> bytes:
        cmd = struct.pack("<6H", p.GetSerialNo, 0, 0, 0, 0, 0)
        return bytes(self._send(cmd))

    def get_list_status(self):
        return self._command(p.GetListStatus)

    def get_position_xy(self):
        return self._command(p.GetPositionXY)

    def goto_xy(self, x: int, y: int):
        """Move the galvos immediately (outside any list)."""
        self._check(x, y)
        distance = self._distance(x, y)
        self._last_x, self._last_y = x, y
        return self._command(p.GotoXY, x, y, 0, distance)

    def reset_list(self):
        return self._command(p.ResetList)

    def restart_list(self):
        return self._command(p.RestartList)

    def write_cor_table(self, table: bool = True):
        return self._command(p.WriteCorTable, int(table))

    def set_control_mode(self, v):
        return self._command(p.SetControlMode, v)

    def set_delay_mode(self, v):
        return self._command(p.SetDelayMode, v)

    def set_end_of_list(self, v):
        return self._command(p.SetEndOfList, v)

    def set_first_pulse_killer(self, v):
        return self._command(p.SetFirstPulseKiller, v)

    def set_laser_mode(self, v):
        return self._command(p.SetLaserMode, v)

    def set_timing(self, v):
        return self._command(p.SetTiming, v)

    def set_standby(self, v1, v2):
        return self._command(p.SetStandby, v1, v2)

    def set_pwm_half_period(self, v):
        return self._command(p.SetPwmHalfPeriod, v)

    def stop_execute(self):
        return self._command(p.StopExecute)

    def stop_list(self):
        return self._command(p.StopList)

    def write_port(self):
        return self._command(p.WritePort, self._port_bits)

    def write_analog_port_1(self, v):
        return self._command(p.WriteAnalogPort1, v)

    def read_port(self):
        return self._command(p.ReadPort)

    def set_fpk_param_2(self, v1, v2, v3, v4):
        return self._command(p.SetFpkParam2, v1, v2, v3, v4)

    def set_fiber_mo(self, mo: int):
        """0 closes, 1 opens the fiber master oscillator."""
        return self._command(p.Fiber_SetMo, mo)

    def enable_z(self):
        return self._command(p.EnableZ)

    def reset(self):
        return self._command(p.Reset)

    def get_mark_time(self):
        """Always called with 3, as MeerK40t does; meaning of the payload unknown."""
        return self._command(p.GetMarkTime, 3)

    def set_fly_res(self, v1, v2, v3, v4):
        return self._command(p.SetFlyRes, v1, v2, v3, v4)
