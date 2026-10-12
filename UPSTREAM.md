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
  value in ns, unconverted, as MeerK40t does. The phase 0 coupon confirms it.

- Framing (`light_mode`) now sends its own jump speed (`BoardSetup.light_speed_mm_s`,
  MeerK40t's `redlight_speed` default of 3000 mm/s). Upstream sets it from the
  live-light job, which we did not keep.

- Ending a list (`rapid_mode`) or aborting turns the red dot off. Upstream
  leaves the light bit set, so the next marking list would run with it on.

## Not carried over

GUI, kernel/service integration, CO2 and UV sources, clone-board firmware
loader (`clone_loader.py`), rotary/cylinder, on-the-fly, Z axis moves, foot
pedal, live light jobs.
