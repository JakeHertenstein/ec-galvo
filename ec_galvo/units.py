"""Millimetres to board units.

The board addresses the lens field as 0..0xFFFF on each axis. As in MeerK40t,
the full range spans the lens field size, (0, 0) is one corner, and
flip/swap options match how the galvos are wired. Flips are applied before the
swap, in MeerK40t's order (see Field).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

GALVO_RANGE = 0xFFFF


class OutOfFieldError(ValueError):
    pass


@dataclass(frozen=True)
class Field:
    """The lens field, and how mm map to board units.

    to_board adds offset_mm, then mirrors the x given (flip_x) and the y given
    (flip_y), and only then swaps x and y. This is MeerK40t's
    order (its View.transform, which balormk uses). The order matters only with
    swap_xy and exactly one flip: the other order turns the result 180 degrees
    about the field centre. Confirm a new machine's values by framing an
    off-centre, asymmetric L shape, not a centred box.
    """

    size_mm: float
    flip_x: bool = False
    flip_y: bool = False
    swap_xy: bool = False
    offset_mm: tuple[float, float] = (0.0, 0.0)  # jig offset, added before mapping

    def __post_init__(self):
        if not (math.isfinite(self.size_mm) and self.size_mm > 0):
            raise ValueError("field size must be a positive number")
        if not math.isfinite(GALVO_RANGE / self.size_mm):
            raise ValueError("field size is too small")
        if len(self.offset_mm) != 2 or not all(math.isfinite(v) for v in self.offset_mm):
            raise ValueError("field offset must be two finite numbers")

    @property
    def units_per_mm(self) -> float:
        return GALVO_RANGE / self.size_mm

    def to_board(self, x_mm: float, y_mm: float) -> tuple[int, int]:
        """Map a point in mm to board units. Raises OutOfFieldError, never clips."""
        x = x_mm + self.offset_mm[0]
        y = y_mm + self.offset_mm[1]
        if self.flip_x:
            x = self.size_mm - x
        if self.flip_y:
            y = self.size_mm - y
        if self.swap_xy:  # after the flips, as in MeerK40t
            x, y = y, x
        fx, fy = x * self.units_per_mm, y * self.units_per_mm
        if not (math.isfinite(fx) and math.isfinite(fy)):
            raise OutOfFieldError("point is outside the lens field")
        bx, by = round(fx), round(fy)
        if not (0 <= bx <= GALVO_RANGE and 0 <= by <= GALVO_RANGE):
            raise OutOfFieldError("point is outside the lens field")
        return bx, by

    def speed(self, mm_per_s: float) -> int:
        """mm/s to the board's units per ms, truncated as MeerK40t does.

        Raises instead of sending 0 (speeds below about 2.3 mm/s on a 150 mm
        field) or a value the 16-bit word can't hold.
        """
        units = mm_per_s * self.units_per_mm / 1000.0
        word = int(units) if math.isfinite(units) else 0
        if not 1 <= word <= 0xFFFF:
            raise ValueError(
                f"speed {mm_per_s:g} mm/s is outside what the board can express on this field"
            )
        return word
