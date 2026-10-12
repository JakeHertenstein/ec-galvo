"""Millimetres to board units.

The board addresses the lens field as 0..0xFFFF on each axis. As in MeerK40t,
the full range spans the lens field size, (0, 0) is one corner, and
flip/swap options match how the galvos are wired.
"""

from __future__ import annotations

from dataclasses import dataclass

GALVO_RANGE = 0xFFFF


class OutOfFieldError(ValueError):
    pass


@dataclass(frozen=True)
class Field:
    size_mm: float
    flip_x: bool = False
    flip_y: bool = False
    swap_xy: bool = False
    offset_mm: tuple[float, float] = (0.0, 0.0)  # jig offset, added before mapping

    def __post_init__(self):
        if self.size_mm <= 0:
            raise ValueError("field size must be positive")

    @property
    def units_per_mm(self) -> float:
        return GALVO_RANGE / self.size_mm

    def to_board(self, x_mm: float, y_mm: float) -> tuple[int, int]:
        """Map a point in mm to board units. Raises OutOfFieldError, never clips."""
        x = x_mm + self.offset_mm[0]
        y = y_mm + self.offset_mm[1]
        if self.swap_xy:
            x, y = y, x
        if self.flip_x:
            x = self.size_mm - x
        if self.flip_y:
            y = self.size_mm - y
        bx = round(x * self.units_per_mm)
        by = round(y * self.units_per_mm)
        if not (0 <= bx <= GALVO_RANGE and 0 <= by <= GALVO_RANGE):
            raise OutOfFieldError("point is outside the lens field")
        return bx, by

    def speed(self, mm_per_s: float) -> int:
        """mm/s to the board's units per ms, capped at 0xFFFF."""
        return min(0xFFFF, abs(int(mm_per_s * self.units_per_mm / 1000.0)))
