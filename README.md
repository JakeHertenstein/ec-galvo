# ec-galvo

A small Python driver for JCZ LMC (EZCAD2-era) galvo laser boards, USB vendor
`0x9588` product `0x9899`, with a MOPA fiber source. Trimmed from MeerK40t's
`balormk` driver (MIT). No GUI, no MeerK40t kernel, one dependency (`pyusb`).

```python
from ec_galvo import CorrectionTable, Field, GalvoController, MarkSettings, USBConnection

cor = CorrectionTable.from_bytes(open("lens.cor", "rb").read())
laser = GalvoController(USBConnection(), Field(size_mm=150.0), cor)
laser.connect()

laser.light_mode()                      # red-dot framing
for x, y in [(10, 10), (60, 10), (60, 60), (10, 60), (10, 10)]:
    laser.light_mm(x, y)
laser.rapid_mode()

laser.program_mode()                    # marking
laser.set_settings(MarkSettings(power_pct=40, speed_mm_s=1500, frequency_khz=60,
                                pulse_width_ns=200, laser_on_delay_us=100,
                                laser_off_delay_us=120, polygon_delay_us=50,
                                jump_speed_mm_s=4000))
laser.jump_mm(10, 10, force=True)
laser.mark_mm(60, 10)
laser.rapid_mode()                      # waits for the list to finish
```

`RecordingConnection` stands in for the board: it keeps every packet in memory
and gives a readable command log for golden-file tests.

## Behaviour that differs from MeerK40t

- Moves outside the lens field raise `OutOfFieldError` (MeerK40t skips them).
- A bad `.cor` raises `CorrectionFileError` (MeerK40t loads a blank table).
  Running uncorrected must be asked for with `CorrectionTable.no_correction()`.
- USB errors raise; there is no silent reconnect mid-job, and no clone-board
  firmware load.
- The log never contains coordinates or packet contents.
- After every connect (including a reconnect) or an abort the galvo position is unknown, so the next move
  must be a jump; abort tries every shutdown step and reports any that failed
  (`AbortIncomplete`).
- Waits (including a paused wait) poll an optional `stop_check` and only time
  out when the board is neither busy nor ready; a board stuck at BUSY waits until
  stopped.
- Settings are checked on creation (`MarkSettings`, `BoardSetup`, `Field.speed`).
- EZCad-style turn-angle words on corners and an 8 ms settle after frequency or
  pulse-width changes (see UPSTREAM.md).
- Fiber source only.

## Tests

    pip install -e '.[test]' && pytest

See `UPSTREAM.md` for where each file came from.
