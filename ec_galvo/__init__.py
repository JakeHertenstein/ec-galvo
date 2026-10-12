"""ec-galvo: a small driver for JCZ LMC (EZCAD2-era) galvo laser boards.

Trimmed from MeerK40t's balormk driver (MIT). See UPSTREAM.md.
"""

from .connection import USBConnection
from .controller import BoardSetup, GalvoController, MarkSettings
from .correction import CorrectionFileError, CorrectionTable
from .recording import RecordingConnection
from .units import Field, OutOfFieldError

__version__ = "0.1.0"

__all__ = [
    "BoardSetup",
    "CorrectionFileError",
    "CorrectionTable",
    "Field",
    "GalvoController",
    "MarkSettings",
    "OutOfFieldError",
    "RecordingConnection",
    "USBConnection",
]
