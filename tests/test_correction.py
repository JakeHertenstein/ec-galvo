import struct

import pytest

from ec_galvo.correction import (
    FLOAT_FILE_LEN,
    INT_FILE_LEN,
    POINTS,
    CorrectionFileError,
    CorrectionTable,
)


def int_cor(values, scale=150.0):
    head = "LMC1COR".encode("utf-16-le").ljust(0x16, b"\0")  # not the float label
    head += b"\0" * 6 + struct.pack("<d", scale)
    return head + struct.pack(f"<{POINTS * 2}i", *values)


def float_cor(values, scale=150.0):
    head = "LMC1COR_1.0".encode("utf-16-le")
    doubles = [0.0] * 63
    doubles[43] = scale
    head += b"\0\0" + struct.pack("<63d", *doubles)
    return head + struct.pack(f"<{POINTS * 2}d", *values)


def test_lengths_match_upstream_layout():
    assert len(int_cor([0] * POINTS * 2)) == INT_FILE_LEN
    assert len(float_cor([0.0] * POINTS * 2)) == FLOAT_FILE_LEN


@pytest.mark.parametrize("make", [int_cor, float_cor])
def test_parses_and_encodes_sign_magnitude(make):
    values = [0] * POINTS * 2
    values[0], values[1] = 5, -5
    values[-2], values[-1] = -0x7FFF, 0x7FFF
    table = CorrectionTable.from_bytes(make(values, scale=163.2), allow_int_layout=True)
    assert len(table.entries) == POINTS
    assert table.entries[0] == (5, 0x8005)
    assert table.entries[-1] == (0xFFFF, 0x7FFF)
    assert table.scale == pytest.approx(163.2)


def test_wrong_length_raises():
    data = int_cor([0] * POINTS * 2)
    with pytest.raises(CorrectionFileError):
        CorrectionTable.from_bytes(data[:-1], allow_int_layout=True)
    with pytest.raises(CorrectionFileError):
        CorrectionTable.from_bytes(data + b"\0", allow_int_layout=True)
    with pytest.raises(CorrectionFileError):
        CorrectionTable.from_bytes(b"")
    data = float_cor([0.0] * POINTS * 2)
    with pytest.raises(CorrectionFileError):
        CorrectionTable.from_bytes(data[:-1])


def test_integer_layout_only_when_asked():
    # The integer layout has no label of its own: without the opt-in, any file of
    # the right size (all zeros, an archive) would pass for one.
    for data in (int_cor([0] * POINTS * 2), bytes(INT_FILE_LEN), b"PK\x03\x04".ljust(INT_FILE_LEN)):
        with pytest.raises(CorrectionFileError, match="not an LMC1COR_1.0"):
            CorrectionTable.from_bytes(data)


def test_non_finite_offset_raises():
    for bad in (float("inf"), float("-inf"), float("nan")):
        values = [0.0] * POINTS * 2
        values[7] = bad
        with pytest.raises(CorrectionFileError):
            CorrectionTable.from_bytes(float_cor(values))


def test_out_of_range_offset_raises():
    values = [0] * POINTS * 2
    values[0] = 0x8000
    with pytest.raises(CorrectionFileError):
        CorrectionTable.from_bytes(int_cor(values), allow_int_layout=True)
