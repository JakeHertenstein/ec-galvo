import math

import pytest
from hypothesis import given
from hypothesis import strategies as st

from ec_galvo import (
    CorrectionTable,
    Field,
    GalvoController,
    MarkSettings,
    OutOfFieldError,
    RecordingConnection,
)
from ec_galvo import protocol as p
from ec_galvo.controller import qswitch_period

LAYER = MarkSettings(
    power_pct=40,
    speed_mm_s=1500,
    frequency_khz=60,
    pulse_width_ns=200,
    laser_on_delay_us=100,
    laser_off_delay_us=120,
    polygon_delay_us=50,
    jump_speed_mm_s=4000,
)


def make(correction=None, logs=None, **kw):
    conn = RecordingConnection()
    c = GalvoController(
        conn,
        Field(size_mm=150.0),
        correction or CorrectionTable.no_correction(),
        log=(logs.append if logs is not None else None),
        **kw,
    )
    c.connect()
    return c, conn


def list_cmds(conn, opcode):
    return [c for c in conn.commands() if c.in_list and c.opcode == opcode]


def test_connect_sends_startup_sequence_in_order():
    _, conn = make()
    names = [c.name for c in conn.commands()]
    assert names == [
        "GetSerialNo", "GetVersion", "Reset", "WriteCorTable", "EnableLaser",
        "SetControlMode", "SetLaserMode", "SetDelayMode", "SetTiming", "SetStandby",
        "SetFirstPulseKiller", "SetPwmHalfPeriod", "SetPwmPulseWidth", "Fiber_SetMo",
        "SetFpkParam2", "SetFlyRes", "EnableZ", "WriteAnalogPort1", "EnableZ",
    ]  # fmt: skip


def test_correction_table_is_written_line_by_line():
    entries = tuple((i % 7, i % 11) for i in range(65 * 65))
    _, conn = make(correction=CorrectionTable(entries=entries))
    lines = [c for c in conn.commands() if c.opcode == p.WriteCorLine]
    assert len(lines) == 65 * 65
    assert lines[0].params[:3] == (0, 0, 0)
    assert lines[1].params[:3] == (1, 1, 1)
    table_cmd = next(c for c in conn.commands() if c.opcode == p.WriteCorTable)
    assert table_cmd.params[0] == 1


def test_wrong_serial_refuses_and_closes():
    with pytest.raises(ConnectionRefusedError):
        make(expected_serial=b"OTHERBRD")


def test_pulse_width_is_sent_and_resent_on_every_new_list():
    # Regression for galvoplotter's stale pulse width: each job starts clean.
    c, conn = make()
    for _ in range(2):
        c.program_mode()
        c.set_settings(LAYER)
        c.set_settings(LAYER)  # unchanged: not resent within the list
        c.jump_mm(10, 10)
        c.mark_mm(20, 10)
        c.rapid_mode()
    sent = list_cmds(conn, p.listFiberYLPMPulseWidth)
    assert [s.params[0] for s in sent] == [200, 200]


def test_pulse_width_change_is_sent():
    c, conn = make()
    c.program_mode()
    c.set_settings(LAYER)
    c.set_settings(MarkSettings(**{**LAYER.__dict__, "pulse_width_ns": 4}))
    c.rapid_mode()
    assert [s.params[0] for s in list_cmds(conn, p.listFiberYLPMPulseWidth)] == [200, 4]


def test_layer_conversions():
    c, conn = make()
    c.program_mode()
    c.set_settings(LAYER)
    c.rapid_mode()
    (q,) = list_cmds(conn, p.listQSwitchPeriod)
    assert q.params[0] == 333  # 60 kHz
    (cur,) = list_cmds(conn, p.listMarkCurrent)
    assert cur.params[0] == round(40 * 0xFFF / 100)
    (poly,) = list_cmds(conn, p.listPolygonDelay)
    assert poly.params[0] == 5  # 50 us in 10 us units
    (speed,) = list_cmds(conn, p.listMarkSpeed)
    assert speed.params[0] == int(1500 * 0xFFFF / 150 / 1000)


def test_out_of_field_raises_instead_of_skipping():
    c, _ = make()
    c.program_mode()
    with pytest.raises(OutOfFieldError):
        c.mark_mm(151, 10)
    with pytest.raises(OutOfFieldError):
        c.jump_mm(-0.1, 10)


def test_bad_power_raises():
    c, _ = make()
    c.program_mode()
    with pytest.raises(ValueError):
        c.power(101)


def test_list_executes_after_third_packet_and_at_end():
    c, conn = make()
    c.program_mode()
    c.set_settings(LAYER)
    c.goto(1000, 1000)
    for i in range(1000):  # > 3 packets of 256 entries
        c.mark(1000 + i, 1000 + (i % 2))
    c.rapid_mode()
    singles = [x.name for x in conn.commands() if not x.in_list]
    assert singles.count("ExecuteList") == 1
    assert sum(1 for pk in conn.packets if len(pk) == p.PACKET_LIST) >= 4


def test_log_never_contains_coordinates():
    logs = []
    c, _ = make(logs=logs)
    c.program_mode()
    c.set_settings(LAYER)
    c.jump_mm(12.345, 67.891)
    c.mark_mm(98.765, 43.21)
    c.rapid_mode()
    c.abort()
    text = "\n".join(logs)
    bx, by = c.field.to_board(98.765, 43.21)
    for needle in ("12.345", "67.891", "98.765", "43.21", str(bx), str(by)):
        assert needle not in text


def test_light_mode_frames_with_red_dot_bit():
    c, conn = make()
    c.light_mode()
    c.light_mm(10, 10)
    c.light_mm(20, 10)
    c.rapid_mode()
    ports = list_cmds(conn, p.listWritePort)
    assert ports and ports[0].params[0] & (1 << 8)


def test_abort_closes_mo_and_returns_to_rapid():
    c, conn = make()
    c.program_mode()
    c.goto(1000, 1000)
    c.mark(2000, 2000)
    c.abort()
    tail = [x.name for x in conn.commands() if not x.in_list][-3:]
    assert tail == ["ExecuteList", "Fiber_SetMo", "WritePort"]
    assert c.mode == "rapid"


@given(
    st.floats(0, 150, allow_nan=False),
    st.floats(0, 150, allow_nan=False),
    st.booleans(),
    st.booleans(),
    st.booleans(),
)
def test_field_maps_inside_range(x, y, fx, fy, sw):
    bx, by = Field(150.0, flip_x=fx, flip_y=fy, swap_xy=sw).to_board(x, y)
    assert 0 <= bx <= 0xFFFF and 0 <= by <= 0xFFFF


def test_field_corners_and_flips():
    f = Field(150.0)
    assert f.to_board(0, 0) == (0, 0)
    assert f.to_board(150, 150) == (0xFFFF, 0xFFFF)
    assert Field(150.0, flip_x=True).to_board(0, 0) == (0xFFFF, 0)
    assert Field(150.0, swap_xy=True).to_board(150, 0) == (0, 0xFFFF)
    assert Field(150.0, offset_mm=(1.0, 0)).to_board(0, 0)[0] == round(0xFFFF / 150)


def test_light_mode_sets_its_own_jump_speed():
    c, conn = make()
    c.light_mode()
    c.light_mm(10, 10)
    c.rapid_mode()
    (speed,) = list_cmds(conn, p.listJumpSpeed)
    assert speed.params[0] == c.field.speed(c.setup.light_speed_mm_s)


def test_light_then_program_switches_back_to_job_jump_speed():
    c, conn = make()
    c.light_mode()
    c.light_mm(10, 10)
    c.program_mode()
    c.jump_mm(20, 20)
    c.rapid_mode()
    speeds = [s.params[0] for s in list_cmds(conn, p.listJumpSpeed)]
    assert speeds == [c.field.speed(3000.0), c.field.speed(2000.0)]


def test_red_dot_is_off_after_framing_and_in_the_next_job():
    c, conn = make()
    c.light_mode()
    c.light_mm(10, 10)
    c.rapid_mode()
    assert not c._port_bits & (1 << 8)
    c.program_mode()
    c.set_settings(LAYER)
    c.mark_mm(20, 20)
    c.rapid_mode()
    singles = [x for x in conn.commands() if not x.in_list and x.opcode == p.WritePort]
    assert singles and all(not w.params[0] & (1 << 8) for w in singles)
    ports = list_cmds(conn, p.listWritePort)
    assert ports[0].params[0] & (1 << 8)  # framing list: on
    assert all(not w.params[0] & (1 << 8) for w in ports[1:])  # marking list: off


@pytest.mark.parametrize(
    "change",
    [
        {"power_pct": 100.5},
        {"power_pct": float("nan")},
        {"frequency_khz": 0.2},  # period would overflow 16 bits
        {"frequency_khz": 0},
        {"speed_mm_s": 0},
        {"pulse_width_ns": 4.5},
        {"pulse_width_ns": 0},
        {"laser_on_delay_us": 40000},
        {"laser_on_delay_us": 100.5},
        {"polygon_delay_us": -10},
        {"polygon_delay_us": 55},
        {"end_delay_us": -10},
        {"end_delay_us": 305},
    ],
)
def test_mark_settings_reject_bad_values(change):
    with pytest.raises(ValueError):
        MarkSettings(**{**LAYER.__dict__, **change})


def test_encode_settings_matches_what_is_sent():
    from ec_galvo import encode_settings

    c, conn = make()
    c.program_mode()
    c.set_settings(LAYER)
    c.rapid_mode()
    enc = encode_settings(c.field, LAYER)
    assert list_cmds(conn, p.listMarkSpeed)[0].params[0] == enc.mark_speed
    assert list_cmds(conn, p.listMarkCurrent)[0].params[0] == enc.power
    assert list_cmds(conn, p.listQSwitchPeriod)[0].params[0] == enc.qswitch_period
    assert list_cmds(conn, p.listFiberYLPMPulseWidth)[0].params[0] == enc.pulse_width


def test_turn_angles_follow_ezcad():
    c, conn = make()
    c.program_mode()
    c.set_settings(LAYER)
    c.jump_mm(10, 10)
    for pt in [(20, 10), (20, 20), (10, 20), (10, 10)]:  # square, 90 degree corners
        c.mark_mm(*pt)
    c.jump_mm(30, 30)
    c.mark_mm(40, 30)
    c.mark_mm(30, 30)  # straight back: 180 degrees
    c.rapid_mode()
    words = [m.params[2] for m in list_cmds(conn, p.listMarkTo)]
    assert words == [0, 0x8000, 0x8000, 0x8000, 0, 0x8000]  # reversal capped at 90 degrees


def test_turn_angle_word_scale():
    from ec_galvo.controller import turn_angle_word

    assert turn_angle_word((1, 0), (1, 0)) == 0  # straight on
    assert turn_angle_word((1, 0), (1, 1)) == 0x4000  # 45 degrees (90 = 0x8000)
    assert turn_angle_word((100, 0), (100, 13)) == round(
        math.degrees(math.atan2(13, 100)) * 65536 / 180
    )
    assert turn_angle_word((1, 0), (-1, 1)) == 0x8000  # 135 degrees, capped


def test_turn_angles_can_be_switched_off():
    from ec_galvo import BoardSetup

    c, conn = make(setup=BoardSetup(send_turn_angles=False))
    c.program_mode()
    c.set_settings(LAYER)
    c.jump_mm(10, 10)
    c.mark_mm(20, 10)
    c.mark_mm(20, 20)
    c.rapid_mode()
    assert [m.params[2] for m in list_cmds(conn, p.listMarkTo)] == [0, 0]


def test_settle_delay_after_frequency_or_pulse_width_change_only():
    c, conn = make()
    c.program_mode()
    c.set_settings(LAYER)  # first: both change
    c.set_settings(MarkSettings(**{**LAYER.__dict__, "power_pct": 50}))  # power only
    c.set_settings(MarkSettings(**{**LAYER.__dict__, "power_pct": 50, "pulse_width_ns": 4}))
    c.set_settings(
        MarkSettings(
            **{**LAYER.__dict__, "power_pct": 50, "pulse_width_ns": 4, "frequency_khz": 100}
        )
    )
    c.rapid_mode()
    delays = [d.params[0] for d in list_cmds(conn, p.listDelayTime)]
    assert delays == [800, 800, 800, 800]  # open MO, first settings, pulse change, frequency change


def test_list_delay_rejects_bad_values():
    c, _ = make()
    c.program_mode()
    for bad in (-1, 70000, 1.5):
        with pytest.raises(ValueError):
            c.list_delay_time(bad)


# --- position, abort and waits -------------------------------------------------


def test_position_is_unknown_after_connect_and_abort():
    c, conn = make()
    c.program_mode()
    with pytest.raises(RuntimeError, match="unknown"):
        c.mark(2000, 2000)
    c.goto(2000, 2000)
    c.mark(3000, 2000)
    c.abort()
    c.program_mode()
    c.set_settings(LAYER)
    c.goto(3000, 2000)  # same point as before the abort: must still jump
    c.mark(4000, 2000)
    c.rapid_mode()
    cmds = [x for x in conn.commands() if x.in_list and x.opcode in (p.listJumpTo, p.listMarkTo)]
    last_jump = max(i for i, x in enumerate(cmds) if x.opcode == p.listJumpTo)
    assert cmds[last_jump].params[:2] == (3000, 2000)
    assert cmds[last_jump].params[3] == 0xFFFF  # distance unknown


def test_forced_jump_and_delay_end_the_turn_chain():
    c, conn = make()
    c.program_mode()
    c.set_settings(LAYER)
    c.goto(1000, 1000)
    c.mark(2000, 1000)
    c.list_delay_time(30)
    c.mark(2000, 2000)  # after a delay: a new run, so no turn angle
    c.goto(2000, 2000, force=True)
    c.mark(1000, 2000)
    c.rapid_mode()
    jumps = list_cmds(conn, p.listJumpTo)
    assert [j.params[:2] for j in jumps] == [(1000, 1000), (2000, 2000)]
    assert [m.params[2] for m in list_cmds(conn, p.listMarkTo)] == [0, 0, 0]


class FailingConnection(RecordingConnection):
    def __init__(self, fail_opcode):
        super().__init__()
        self.fail_opcode = fail_opcode

    def write(self, packet):
        import struct

        if len(packet) == 12 and struct.unpack_from("<H", packet)[0] == self.fail_opcode:
            raise ConnectionError("simulated failure")
        super().write(packet)


def test_abort_tries_every_step_and_reports_failures():
    from ec_galvo import AbortIncomplete

    conn = FailingConnection(p.StopExecute)
    c = GalvoController(conn, Field(150.0), CorrectionTable.no_correction())
    c.connect()
    c.program_mode()
    c.goto(1000, 1000)
    c.mark(2000, 1000)
    with pytest.raises(AbortIncomplete, match="stop execute"):
        c.abort()
    names = [x.name for x in conn.commands() if not x.in_list]
    tail = names[names.index("ResetList", len(names) - 12) - 1 :]
    assert "Fiber_SetMo" in tail and tail[-1] == "WritePort"
    assert c.mode == "rapid" and c.get_last_xy() == (None, None)


def test_wait_polls_stop_and_keeps_waiting_while_busy(monkeypatch):
    from ec_galvo import StopRequested

    c, _ = make(wait_timeout_s=0.05)
    states = iter([p.BUSY] * 20 + [p.READY])
    monkeypatch.setattr(c, "status", lambda: next(states))
    c.wait_ready()  # 20 busy polls take longer than the timeout, but busy means alive
    monkeypatch.setattr(c, "status", lambda: p.BUSY)
    c.stop_check = lambda: True
    with pytest.raises(StopRequested):
        c.wait_idle()
    c.stop_check = None
    monkeypatch.setattr(c, "status", lambda: 0)  # neither ready nor busy: stuck
    with pytest.raises(TimeoutError):
        c.wait_ready()


def test_speed_that_would_reach_the_board_as_zero_raises():
    f = Field(150.0)
    with pytest.raises(ValueError):
        f.speed(2.0)
    assert f.speed(3.0) == 1
    with pytest.raises(ValueError):
        f.speed(200000)


def test_board_setup_is_checked():
    from ec_galvo import BoardSetup

    with pytest.raises(ValueError):
        BoardSetup(open_mo_delay_ms=-8)
    with pytest.raises(ValueError):
        BoardSetup(light_speed_mm_s=0)


def test_abort_does_not_hang_on_a_board_that_stays_busy(monkeypatch):
    from ec_galvo import AbortIncomplete

    c, _ = make(wait_timeout_s=0.05)
    c.program_mode()
    c.goto(1000, 1000)
    c.mark(2000, 1000)
    monkeypatch.setattr(c, "status", lambda: p.BUSY)
    with pytest.raises(AbortIncomplete, match="flush list"):
        c.abort()
    assert c.mode == "rapid"


def test_reconnect_forgets_the_position():
    c, conn = make()
    c.program_mode()
    c.set_settings(LAYER)
    c.goto(2000, 1000)
    c.mark(3000, 1000)
    c.rapid_mode()
    c.disconnect()
    c.connect()
    c.program_mode()
    c.set_settings(LAYER)
    c.goto(3000, 1000)  # where the last job ended: still sent after a reconnect
    c.mark(4000, 1000)
    c.rapid_mode()
    jumps = [j.params[:2] for j in list_cmds(conn, p.listJumpTo)]
    assert jumps == [(2000, 1000), (3000, 1000)]


def test_stop_while_paused():
    from ec_galvo import StopRequested

    c, _ = make()
    c.program_mode()
    c.paused = True
    c.stop_check = lambda: True
    c.goto(1000, 1000)
    with pytest.raises(StopRequested):
        for i in range(400):  # fills a packet; sending it waits on the pause
            c.goto(1000 + i, 1000)


def test_inexact_board_delays_and_polygon_values_raise():
    from ec_galvo import BoardSetup

    with pytest.raises(ValueError):
        BoardSetup(open_mo_delay_ms=0.295)
    c, _ = make()
    c.program_mode()
    with pytest.raises(ValueError):
        c.list_polygon_delay(55)
    with pytest.raises(ValueError):
        c.list_polygon_delay(700000)


def test_ctrl_c_during_abort_still_runs_every_step():
    c, conn = make()
    c.program_mode()
    c.goto(1000, 1000)
    c.mark(2000, 1000)
    real = c.stop_execute

    def interrupted():
        real()
        raise KeyboardInterrupt

    c.stop_execute = interrupted
    with pytest.raises(KeyboardInterrupt):
        c.abort()
    names = [x.name for x in conn.commands() if not x.in_list]
    assert names[-1] == "WritePort" and "Fiber_SetMo" in names[-6:]
    assert c.mode == "rapid"


def test_capped_turns_are_counted():
    c, _ = make()
    c.program_mode()
    c.set_settings(LAYER)
    c.jump_mm(10, 10)
    c.mark_mm(20, 10)
    c.mark_mm(20, 20)  # 90: not capped
    c.mark_mm(10, 11)  # sharper than 90: capped
    c.rapid_mode()
    assert c.capped_turns == 1


def test_public_delay_setters_refuse_bad_values():
    c, _ = make()
    c.program_mode()
    for bad in (100.7, -40000):
        with pytest.raises(ValueError):
            c.list_laser_on_delay(bad)
    with pytest.raises(ValueError):
        c.list_jump_delay(40000)


def test_recording_connection_repr_hides_packets():
    _, conn = make()
    assert "packets" not in repr(conn)


def test_board_setup_refuses_inexact_types():
    from ec_galvo import BoardSetup

    for bad in (
        {"first_pulse_killer": 200.7},
        {"timing_mode": True},
        {"fpk2": (0xFFB, 1, 409.9, 409)},
        {"standby": (2000,)},
        {"send_turn_angles": "no"},
        {"analog_port_1": -1},
    ):
        with pytest.raises(ValueError):
            BoardSetup(**bad)


def test_capped_turns_count_per_job():
    c, _ = make()
    for _ in range(2):
        c.program_mode()
        c.set_settings(LAYER)
        c.jump_mm(10, 10)
        c.mark_mm(20, 10)
        c.mark_mm(10, 11)
        c.rapid_mode()
        assert c.capped_turns == 1


@pytest.mark.parametrize("size", [0.0, -1.0, float("inf"), float("nan"), 5e-324])
def test_field_size_must_be_a_usable_number(size):
    with pytest.raises(ValueError):
        Field(size)


@pytest.mark.parametrize("offset", [(float("inf"), 0.0), (0.0, float("nan")), (0.0,)])
def test_field_offset_must_be_two_finite_numbers(offset):
    with pytest.raises(ValueError):
        Field(150.0, offset_mm=offset)


def test_huge_numbers_raise_plain_errors_not_overflow():
    field = Field(150.0)
    with pytest.raises(OutOfFieldError):
        field.to_board(1e308, 0.0)
    with pytest.raises(ValueError, match="outside what the board can express"):
        field.speed(1e308)
    with pytest.raises(ValueError):
        field.speed(float("nan"))
    for khz in (5e-324, float("inf"), float("nan")):
        with pytest.raises(ValueError):
            qswitch_period(khz)
