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
- Waits time out only while the board is neither busy nor in the wanted
  state; while BUSY they wait as long as the job runs, polling stop_check.
- Fiber source only (CO2/UV paths removed).
"""

from __future__ import annotations

import math
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
    light_speed_mm_s: float = 3000.0  # red-dot framing (MeerK40t redlight_speed)
    # Pause after a frequency or pulse-width change before marking on. JPT's M7
    # manual gives a 4-8 ms "frequency switching time" (8 recommended); a new
    # pulse width travels as a 4-byte serial frame (~3.2 ms at 10 kHz) and takes
    # effect once it has arrived. Not in MeerK40t; whether LightBurn waits is
    # unconfirmed.
    settle_delay_ms: float = 8.0
    # Send each mark's turn angle in MarkTo word 3 (0x8000 = 90 degrees, 0 after
    # a jump), as EZCad does. MeerK40t always sends 0.
    send_turn_angles: bool = True

    def __post_init__(self):
        def word(name, v):
            if isinstance(v, bool) or not isinstance(v, int) or not 0 <= v <= 0xFFFF:
                raise ValueError(f"{name} must be a whole number 0-65535")

        for name in (
            "first_pulse_killer",
            "pwm_half_period",
            "pwm_pulse_width",
            "timing_mode",
            "delay_mode",
            "laser_mode",
            "control_mode",
            "analog_port_1",
        ):
            word(name, getattr(self, name))
        for name, n in (("standby", 2), ("fpk2", 4), ("fly_res", 4)):
            v = getattr(self, name)
            if not isinstance(v, tuple) or len(v) != n:
                raise ValueError(f"{name} must be {n} values")
            for x in v:
                word(name, x)
        if not isinstance(self.send_turn_angles, bool):
            raise ValueError("send_turn_angles must be true or false")  # noqa: TRY004 - one error type for bad setup
        for name in ("open_mo_delay_ms", "settle_delay_ms"):
            v = getattr(self, name)
            # sent in 10 us units: only exact values
            if not (math.isfinite(v) and 0 <= v <= 655.35) or abs(v * 100 - round(v * 100)) > 1e-9:
                raise ValueError(f"{name} must be 0-655.35 ms in 0.01 ms steps")
        for name in ("default_jump_speed_mm_s", "light_speed_mm_s"):
            v = getattr(self, name)
            if not (math.isfinite(v) and v > 0):
                raise ValueError(f"{name} must be positive")


class StopRequested(Exception):
    """The caller's stop check returned True while the driver was waiting on the board."""


class AbortIncomplete(RuntimeError):
    """One or more abort steps failed; the board may not be fully stopped."""


@dataclass(frozen=True)
class MarkSettings:
    """Per-layer settings, already in physical units. Checked on creation.

    end_delay_us (End TC; default 300 us is MeerK40t's delay_end default) is
    not sent by set_settings: the caller sends list_delay_time(end_delay_us // 10)
    after each object, as EZCad does (burner's mark_plan does this per path).
    """

    power_pct: float
    speed_mm_s: float
    frequency_khz: float
    pulse_width_ns: int
    laser_on_delay_us: float
    laser_off_delay_us: float
    polygon_delay_us: float
    jump_speed_mm_s: float
    end_delay_us: float = 300.0

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            v = getattr(self, name)
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
                raise ValueError(f"{name} must be a finite number")
        power_word(self.power_pct)
        qswitch_period(self.frequency_khz)
        if self.speed_mm_s <= 0 or self.jump_speed_mm_s <= 0:
            raise ValueError("speeds must be positive")
        if (
            self.pulse_width_ns != int(self.pulse_width_ns)
            or not 1 <= self.pulse_width_ns <= 0xFFFF
        ):
            raise ValueError("pulse_width_ns must be a whole number from 1 to 65535")
        for name in ("laser_on_delay_us", "laser_off_delay_us"):
            v = getattr(self, name)
            if v != int(v) or abs(v) > 0x7FFF:
                raise ValueError(f"{name} must be a whole number of us within +-32767")
        # Sent in 10 us units; how LightBurn rounds is unknown, so only exact values.
        for name, top in (("polygon_delay_us", 655350), ("end_delay_us", 655350)):
            v = getattr(self, name)
            if not 0 <= v <= top or v % 10:
                raise ValueError(f"{name} must be a multiple of 10 us from 0 to {top}")


TURN_WORD_MAX = 0x8000  # 90 degrees: the sharpest turn seen in any EZCad capture


def _turn_degrees(prev: tuple[int, int], new: tuple[int, int]) -> float:
    a = math.hypot(*prev) * math.hypot(*new)
    if a == 0:
        return 0.0
    cos = max(-1.0, min(1.0, (prev[0] * new[0] + prev[1] * new[1]) / a))
    return math.degrees(math.acos(cos))


def turn_angle_word(prev: tuple[int, int], new: tuple[int, int]) -> int:
    """Turn between two mark directions as EZCad sends it: degrees * 65536 / 180.

    Capped at 0x8000 (90 degrees). No capture shows a sharper turn, and at this
    scale 180 degrees would not fit in 16 bits, so how EZCad encodes 90-180
    degrees (acute corners, reversals) is unknown; sending more than was ever
    observed risks the board reading it some other way.
    """
    return min(TURN_WORD_MAX, round(_turn_degrees(prev, new) * 65536 / 180))


def power_word(power_pct: float) -> int:
    """Power in percent -> the listMarkCurrent value (12-bit)."""
    if not 0 <= power_pct <= 100:
        raise ValueError("power must be 0-100 %")
    return round(power_pct * 0xFFF / 100.0)


def qswitch_period(frequency_khz: float) -> int:
    """Frequency in kHz -> the listQSwitchPeriod value (MeerK40t: 20000 / kHz)."""
    if not frequency_khz > 0:
        raise ValueError("frequency must be positive")
    period = round(20000.0 / frequency_khz)
    if not 1 <= period <= 0xFFFF:
        raise ValueError("frequency out of range for the board")
    return period


@dataclass(frozen=True)
class EncodedSettings:
    """What the board receives for one MarkSettings (for matching in simulations)."""

    mark_speed: int
    power: int
    qswitch_period: int
    pulse_width: int


def encode_settings(field: Field, s: MarkSettings) -> EncodedSettings:
    return EncodedSettings(
        mark_speed=field.speed(s.speed_mm_s),
        power=power_word(s.power_pct),
        qswitch_period=qswitch_period(s.frequency_khz),
        pulse_width=int(s.pulse_width_ns),
    )


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

        # Where the galvos are. None = unknown (after connecting or an abort), in
        # which case the next move must be a jump.
        self._last_x: int | None = None
        self._last_y: int | None = None
        # Polled while waiting on the board; returning True raises StopRequested.
        self.stop_check: Callable[[], bool] | None = None
        self._aborting = False  # during abort, waits never extend for a busy board
        self.capped_turns = 0  # corners sharper than 90 degrees sent as 90 (see turn_angle_word)
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
        self._prev_dir = None  # direction of the last mark in the current run

    #######################
    # CONNECTION
    #######################

    def connect(self) -> None:
        # A fresh connection knows nothing about where the galvos are or what the
        # board holds, even if this controller was connected before.
        self._last_x = self._last_y = None
        self._prev_dir = None
        self.capped_turns = 0
        self.mode = STATE_RAPID
        self.paused = False
        self._active_list = None
        self._active_index = 0
        self._list_executing = False
        self._number_of_list_packets = 0
        self.connection.open()
        try:
            self._init_board()
        except BaseException:
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

    def _wait(self, flag: int, want: bool, what: str):
        """Wait until the status flag reaches `want`.

        While the board reports BUSY it is executing a list, which can take as
        long as the job needs, so the timeout only runs while it is neither
        busy nor in the wanted state (a stuck board). stop_check is polled
        throughout.
        """
        deadline = time.monotonic() + self.wait_timeout_s
        while True:
            status = self.status()
            if bool(status & flag) == want:
                return
            if self.stop_check is not None and self.stop_check():
                raise StopRequested
            if status & p.BUSY and not self._aborting:
                deadline = time.monotonic() + self.wait_timeout_s
            elif time.monotonic() > deadline:
                raise TimeoutError(f"board did not become {what}")
            time.sleep(0.01)

    def wait_ready(self):
        self._wait(p.READY, True, "ready")

    def wait_idle(self):
        self._wait(p.BUSY, False, "idle")

    #######################
    # MODES
    #######################

    def program_mode(self):
        """Start a marking list."""
        if self.mode == STATE_PROGRAM:
            return
        self.capped_turns = 0  # counted per job
        if self.mode == STATE_LIGHT:
            self.mode = STATE_PROGRAM
            self._light(False)
            self._port(MAIN_BIT, True)
            self.write_port()
            self.set_fiber_mo(1)
            self.list_jump_speed(self.setup.default_jump_speed_mm_s)
            return
        self.mode = STATE_PROGRAM
        self.reset_list()
        self._port(MAIN_BIT, True)
        self.write_port()
        self.set_fiber_mo(1)
        self._forget_settings()
        self.list_ready()
        if self.setup.open_mo_delay_ms:
            self.list_delay_time(round(self.setup.open_mo_delay_ms * 100))
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
        # Upstream leaves this to the caller; without it framing runs at whatever
        # speed the board last had.
        self.list_jump_speed(self.setup.light_speed_mm_s)
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
        self._light(False)  # upstream leaves the red dot on after framing
        self.write_port()
        self.mode = STATE_RAPID

    def abort(self):
        """Stop the board. Every step is tried even if an earlier one fails;
        AbortIncomplete lists the steps that failed."""
        self.paused = False
        self.stop_check = None
        failed: list[str] = []
        interrupted: BaseException | None = None

        def attempt(name, fn):
            nonlocal interrupted
            try:
                fn()
            except Exception:  # noqa: BLE001 - every remaining step must still run
                failed.append(name)
            except BaseException as e:  # noqa: BLE001 - Ctrl-C mid-abort: finish, then re-raise
                failed.append(name)
                interrupted = interrupted or e

        def flush_dummy_list():
            self._list_new()
            self.list_end_of_list()
            self._list_end()
            if not self._list_executing:
                self.execute_list()

        def ports_off():
            self._port(MAIN_BIT, False)
            self._light(False)
            self.write_port()

        self._aborting = True
        try:
            attempt("stop execute", self.stop_execute)
            attempt("close MO", lambda: self.set_fiber_mo(0))
            attempt("reset list", self.reset_list)
            attempt("flush list", flush_dummy_list)
            attempt("close MO again", lambda: self.set_fiber_mo(0))
            attempt("ports off", ports_off)
        finally:
            self._aborting = False
        self._active_list = None
        self._active_index = 0
        self._list_executing = False
        self._number_of_list_packets = 0
        self._last_x = self._last_y = None  # where the galvos stopped is unknown
        self._prev_dir = None
        self.mode = STATE_RAPID
        if failed:
            self._log(f"abort incomplete: {', '.join(failed)}")
            incomplete = AbortIncomplete(", ".join(failed))
            if interrupted is not None:
                raise interrupted from incomplete
            raise incomplete
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
        """Queue one layer's settings. Unchanged values are not resent.

        After a frequency or pulse-width change the list waits settle_delay_ms
        before anything else happens.
        """
        changed = self.list_fiber_ylpm_pulse_width(int(s.pulse_width_ns))
        self.list_jump_speed(s.jump_speed_mm_s)
        changed |= self.frequency(s.frequency_khz)
        self.power(s.power_pct)
        self.list_mark_speed(s.speed_mm_s)
        self.list_laser_on_delay(s.laser_on_delay_us)
        self.list_laser_off_delay(s.laser_off_delay_us)
        self.list_polygon_delay(s.polygon_delay_us)
        if changed and self.setup.settle_delay_ms:
            self.list_delay_time(round(self.setup.settle_delay_ms * 100))

    def power(self, power_pct: float) -> bool:
        word = power_word(power_pct)
        if self._power == power_pct:
            return False
        self._power = power_pct
        self.list_mark_current(word)
        return True

    def frequency(self, frequency_khz: float) -> bool:
        period = qswitch_period(frequency_khz)
        if self._frequency == frequency_khz:
            return False
        self._frequency = frequency_khz
        self.list_qswitch_period(period)
        return True

    #######################
    # MOVES (mm)
    #######################

    def jump_mm(self, x: float, y: float, force: bool = False):
        self.goto(*self.field.to_board(x, y), force=force)

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
        if self._last_x is None:
            raise RuntimeError("mark with the galvo position unknown; jump first")
        if (x, y) == (self._last_x, self._last_y):
            return
        self.list_mark(x, y)

    def goto(self, x: int, y: int, force: bool = False):
        """Jump. Skipped when already there, unless force (each object starts
        with a jump, as EZCad does) or the position is unknown."""
        self._check(x, y)
        if not force and (x, y) == (self._last_x, self._last_y):
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
                if self.stop_check is not None and self.stop_check():
                    raise StopRequested
                time.sleep(0.05)
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
        if self._last_x is None:
            return 0xFFFF  # unknown start
        return min(0xFFFF, int(abs(complex(x, y) - complex(self._last_x, self._last_y))))

    def list_jump(self, x: int, y: int):
        self._list_write(p.listJumpTo, int(x), int(y), 0, self._distance(x, y))
        self._last_x, self._last_y = int(x), int(y)
        self._prev_dir = None

    def list_mark(self, x: int, y: int):
        if self._last_x is None:
            raise RuntimeError("mark with the galvo position unknown; jump first")
        x, y = int(x), int(y)
        d = (x - self._last_x, y - self._last_y)
        angle = 0
        if self.setup.send_turn_angles and self._prev_dir is not None:
            angle = turn_angle_word(self._prev_dir, d)
            if angle == TURN_WORD_MAX and _turn_degrees(self._prev_dir, d) > 90.0 + 1e-9:
                self.capped_turns += 1
        self._list_write(p.listMarkTo, x, y, angle, self._distance(x, y))
        self._last_x, self._last_y = x, y
        self._prev_dir = d

    def list_end_of_list(self):
        self._list_write(p.listEndOfList)

    def list_laser_on_point(self, dwell_10us: int):
        self._list_write(p.listLaserOnPoint, dwell_10us)
        self._prev_dir = None

    def list_delay_time(self, delay_10us: int):
        """Delay in 10 microsecond units. Ends the current mark run."""
        if delay_10us != int(delay_10us) or not 0 <= delay_10us <= 0xFFFF:
            raise ValueError("list delay must be 0-65535 in 10 us units")
        self._list_write(p.listDelayTime, int(delay_10us))
        self._prev_dir = None

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
        if delay != int(delay) or abs(delay) > 0x7FFF:
            raise ValueError("delay must be a whole number of us within +-32767")
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
        if not 0 <= delay_us <= 655350 or delay_us % 10:
            raise ValueError("polygon delay must be a multiple of 10 us from 0 to 655350")
        value = int(delay_us // 10)
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

    def list_fiber_ylpm_pulse_width(self, pulse_width_ns: int) -> bool:
        """MOPA pulse width in ns (sent as-is, as MeerK40t does; JPT takes ns)."""
        if self._pulse_width == pulse_width_ns:
            return False
        self._pulse_width = pulse_width_ns
        self._list_write(p.listFiberYLPMPulseWidth, pulse_width_ns)
        return True

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
        self._prev_dir = None
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
