# ec-galvo

A small Python driver for JCZ LMC (EZCAD2-era) galvo laser boards, USB vendor
`0x9588` product `0x9899`, with a MOPA fiber source. Trimmed from MeerK40t's
`balormk` driver (MIT). No GUI, no MeerK40t kernel, one dependency (`pyusb`).

```python
from ec_galvo import CorrectionTable, Field, GalvoController, MarkSettings, USBConnection

cor = CorrectionTable.from_bytes(open("lens.cor", "rb").read())
laser = GalvoController(USBConnection(), Field(size_mm=150.0), cor)
laser.connect()
settings = MarkSettings(power_pct=40, speed_mm_s=1500, frequency_khz=60,
                        pulse_width_ns=200, laser_on_delay_us=100,
                        laser_off_delay_us=120, polygon_delay_us=50,
                        jump_speed_mm_s=4000)

try:
    laser.light_mode()                  # red-dot framing
    for x, y in [(10, 10), (60, 10), (60, 60), (10, 60), (10, 10)]:
        laser.light_mm(x, y)
    laser.rapid_mode()                  # always between framing and marking

    laser.program_mode()                # marking
    laser.set_settings(settings)
    laser.jump_mm(10, 10, force=True)
    laser.mark_mm(60, 10)
    laser.list_delay_time(int(settings.end_delay_us // 10))  # End TC, after each object
    laser.rapid_mode()                  # waits for the list to finish
except BaseException:                   # any error, Ctrl-C included: stop the board
    laser.abort()
    raise
```

Drive a controller from one thread. To stop a job from another thread (a stop
button, say), make `stop_check` return True: the job thread's next wait raises
`StopRequested`, and that thread calls `abort()`.

### Wiring: flips and swap

`Field` applies `flip_x` and `flip_y` first and then `swap_xy`, as MeerK40t
does. A centred box looks the same under several wrong settings, so confirm a
new machine's values by framing an off-centre, lopsided L (for example a long
leg along +x and a short one along +y from a corner near (20, 20) mm) and
checking it lands where and how the job file shows it.

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
- Settings are checked on creation (`MarkSettings`, `BoardSetup`, `Field.speed`);
  a `BoardSetup` jump or framing speed the field can't express fails when the
  controller is created.
- Calls that don't fit the current mode raise `RuntimeError` instead of being
  queued: list calls (moves, settings, delays) need `program_mode()` or
  `light_mode()` first, marks need `program_mode()`, `goto_xy()` only works
  outside a list, and going between framing and marking needs `rapid_mode()` in
  between.
- Board-unit calls take whole numbers 0-65535 (`list_mark_current` 0-4095);
  anything else raises `ValueError` instead of being truncated. A board reply of
  the wrong length raises `ConnectionError`.
- A layer whose settings can't be sent queues nothing (`set_settings` works out
  every value first).
- EZCad-style turn-angle words on corners and an 8 ms settle after frequency or
  pulse-width changes (see UPSTREAM.md).
- Fiber source only.

## Tests

    pip install -e '.[test]' && pytest

See `UPSTREAM.md` for where each file came from.
