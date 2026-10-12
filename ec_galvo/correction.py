"""Lens correction (.cor) files.

Parses the two EZCAD2 .cor layouts MeerK40t knows, from bytes already in
memory (the caller verifies the bundle digest first, then hands over those
bytes). Anything unexpected raises CorrectionFileError; there is no silent
fallback to a blank table.

Format logic from MeerK40t meerk40t/balormk/controller.py (MIT).
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

GRID = 65
POINTS = GRID * GRID

_LABEL_LEN = 0x16
_FLOAT_LABEL = "LMC1COR_1.0"
_FLOAT_HEADER_LEN = 0x1FA
_INT_HEADER_LEN = 0xE

FLOAT_FILE_LEN = _LABEL_LEN + _FLOAT_HEADER_LEN + POINTS * 16
INT_FILE_LEN = _LABEL_LEN + _INT_HEADER_LEN + POINTS * 8


class CorrectionFileError(ValueError):
    pass


def _encode(v: float) -> int:
    """Board wants sign-magnitude: bit 15 is the sign, low 15 bits the size."""
    n = round(v)
    if abs(n) > 0x7FFF:
        raise CorrectionFileError(f"correction offset {n} out of range")
    return n if n >= 0 else (-n + 0x8000)


@dataclass(frozen=True)
class CorrectionTable:
    """65 x 65 grid of (dx, dy) already encoded for the board."""

    entries: tuple[tuple[int, int], ...]
    scale: float | None = None
    blank: bool = False

    def __post_init__(self):
        if not self.blank and len(self.entries) != POINTS:
            raise CorrectionFileError(f"expected {POINTS} entries, got {len(self.entries)}")

    @classmethod
    def no_correction(cls) -> CorrectionTable:
        """Explicitly run without lens correction (tests, bench work)."""
        return cls(entries=(), blank=True)

    @classmethod
    def from_bytes(cls, data: bytes) -> CorrectionTable:
        if len(data) < _LABEL_LEN:
            raise CorrectionFileError("file too short to be a .cor file")
        try:
            label = data[:_LABEL_LEN].decode("utf-16")
        except UnicodeDecodeError:
            label = data[:_LABEL_LEN].decode("utf-8", errors="ignore")

        if label == _FLOAT_LABEL:
            if len(data) != FLOAT_FILE_LEN:
                raise CorrectionFileError(
                    f"{_FLOAT_LABEL} file should be {FLOAT_FILE_LEN} bytes, got {len(data)}"
                )
            header = data[_LABEL_LEN : _LABEL_LEN + 2 + 0x1F8]
            scale = struct.unpack_from("<63d", header, 2)[43]
            values = struct.unpack_from(f"<{POINTS * 2}d", data, _LABEL_LEN + _FLOAT_HEADER_LEN)
        else:
            if len(data) != INT_FILE_LEN:
                raise CorrectionFileError(
                    f"unrecognised .cor file: label {label!r}, {len(data)} bytes "
                    f"(expected {INT_FILE_LEN} for the integer layout)"
                )
            scale = struct.unpack_from("<d", data, _LABEL_LEN + 6)[0]
            values = struct.unpack_from(f"<{POINTS * 2}i", data, _LABEL_LEN + _INT_HEADER_LEN)

        entries = tuple(
            (_encode(values[i]), _encode(values[i + 1])) for i in range(0, len(values), 2)
        )
        return cls(entries=entries, scale=scale)
