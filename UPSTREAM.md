# Upstream

Forked from MeerK40t (https://github.com/meerk40t/meerk40t, MIT), `main` at
commit `d511944e08788e88077727f682957759884ff5ca`. The full MeerK40t tree is
still on this fork's `main` branch for reference.

| ec-galvo file | From |
| --- | --- |
| `ec_galvo/protocol.py` | `meerk40t/balormk/controller.py` command tables, copied verbatim |
| `ec_galvo/controller.py` | `meerk40t/balormk/controller.py` `GalvoController`, trimmed |
| `ec_galvo/correction.py` | `controller.py` `.cor` reading, rewritten to parse bytes strictly |
| `ec_galvo/connection.py` | `meerk40t/balormk/usb_connection.py`, trimmed |
| `ec_galvo/recording.py` | new; replaces `meerk40t/balormk/mock_connection.py` |
| `ec_galvo/units.py` | new; mapping matches `meerk40t/balormk/device.py` (`0xFFFF` over the lens size) |

Board start-up values (`BoardSetup`) are MeerK40t's device defaults.

## Known upstream quirks kept on purpose

- `SetFpkParam2`: MeerK40t declares p4 = 100 but sends p3 (409) twice. We send
  409, 409 so the phase 0 MeerK40t coupon still describes what we send.
  Revisit once coupons are compared.
- MOPA pulse width goes out as `listFiberYLPMPulseWidth` (0x8026) with the
  value in ns, unconverted, as MeerK40t does. JPT's M7 manual says the source
  takes a plain ns value; the phase 0 coupon confirms the board passes it through.

## Deliberate changes from upstream

- Framing (`light_mode`) now sends its own jump speed (`BoardSetup.light_speed_mm_s`,
  MeerK40t's `redlight_speed` default of 3000 mm/s). Upstream sets it from the
  live-light job, which we did not keep.
- Ending a list (`rapid_mode`) or aborting turns the red dot off. Upstream
  leaves the light bit set, so the next marking list would run with it on.
- MarkTo word 3 carries the turn angle at the start of each mark
  (degrees x 65536 / 180, 0x8000 = 90 degrees, 0 after a jump), as EZCad sends it
  (Bryce Schroeder's EZCad captures). MeerK40t always sends 0. The board may scale
  the polygon delay by it. Capped at 0x8000: no capture shows a turn sharper than
  90 degrees, so larger values are never sent. `BoardSetup.send_turn_angles = False` restores
  MeerK40t's behaviour.
- After a frequency or pulse-width change the list waits
  `BoardSetup.settle_delay_ms` (8 ms): JPT's M7 manual recommends 4-8 ms after a
  frequency switch, and the pulse-width frame takes ~3 ms to reach the source.
  Not in MeerK40t.
- `MarkSettings` checks every value on creation: power 0-100 %, frequencies the
  board can express (MeerK40t's `& 0xFFFF` silently wrapped below 0.31 kHz),
  whole-ns pulse widths, whole-us laser on/off delays, and polygon/end delays in
  exact 10 us steps (they are sent in 10 us units; LightBurn's rounding is unknown).
- `power_word`, `qswitch_period` and `encode_settings` expose the conversions so
  simulators can match marks to settings. Power is rounded as MeerK40t does; EZCad
  truncates (50 % -> 2047, not 2048). Same result at 32 % and 35 %.
- End TC: MeerK40t sends it once at the end of a job. EZCad sends it once per
  object, after the last mark and before closing the MO, and leaves it out when it
  is 0; every captured object was a single outline, so whether it also follows
  each hatch line is untested. ec-galvo leaves this to the caller
  (`list_delay_time`); burner sends it after every path (unconfirmed for hatches).
- Polygon delay in 10 us units (`/10`) is MeerK40t's since 2025-08 (commit
  3616dfe5d, unsourced; before that it sent raw us). An EZCad capture sending 10
  with a probably-default 100 us Polygon TC supports /10. Kept, unconfirmed.

- The galvo position is unknown after every connect (including a reconnect) and after an abort (StopExecute
  leaves the galvos wherever they were): the next move must be a jump, which is
  then always sent. Upstream keeps the last queued point, so a job starting there
  could skip its first jump and mark a stray line from wherever the galvos
  stopped. `goto(..., force=True)` sends a jump even to the current point, as
  EZCad starts each object; burner does this for every path.
- `abort()` tries every step (stop, close MO, reset, flush, close MO again, ports
  off) even if one
  fails, and raises `AbortIncomplete` naming the failed steps. During an abort,
  waits use the hard timeout.
- Waits: while the board reports BUSY it is executing, which can take as long as
  the job needs, so the timeout only runs while it is neither busy nor ready.
  `stop_check` is polled throughout, including while paused, and raises
  `StopRequested`. A board stuck at BUSY therefore waits until stopped.
- `Field.speed` raises instead of sending 0 (below ~2.3 mm/s on 150 mm) or
  clamping to 0xFFFF. `BoardSetup` checks its delays (exact 0.01 ms steps) and
  speeds on creation; `list_polygon_delay` raises instead of clamping.
- A list delay ends a mark run, so the next mark carries no turn angle.

## Not carried over

GUI, kernel/service integration, CO2 and UV sources, clone-board firmware
loader (`clone_loader.py`), rotary/cylinder, on-the-fly, Z axis moves, foot
pedal, live light jobs.
