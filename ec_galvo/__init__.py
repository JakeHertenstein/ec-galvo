"""ec-galvo: a small driver for JCZ LMC (EZCAD2-era) galvo laser boards.

Trimmed from MeerK40t's balormk driver (MIT). See UPSTREAM.md.
"""

from .connection import USBConnection
from .controller import (
    AbortIncomplete,
    BoardSetup,
    EncodedSettings,
    GalvoController,
    MarkSettings,
    StopRequested,
    encode_settings,
)
from .correction import CorrectionFileError, CorrectionTable
from .recording import RecordingConnection
from .units import Field, OutOfFieldError

# Bump in any change that alters what is sent to the board (burner records it
# with every card).
__version__ = "0.2.0"

__all__ = [
    "AbortIncomplete",
    "BoardSetup",
    "CorrectionFileError",
    "CorrectionTable",
    "EncodedSettings",
    "Field",
    "GalvoController",
    "MarkSettings",
    "OutOfFieldError",
    "RecordingConnection",
    "StopRequested",
    "USBConnection",
    "encode_settings",
]
