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
