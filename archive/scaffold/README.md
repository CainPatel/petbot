# Archived scaffold sketches

These are the Arduino-IDE sketches written when the repo was scaffolded on
2026-09-01, before the hardware was finished. They were superseded by the
PlatformIO sources that actually ran the demo:

| Scaffold | Replaced by |
|---|---|
| `firmware/cdpr_controller/cdpr_controller.ino` | `firmware/uno_winches.cpp` |
| `esp32/platform_node/platform_node.ino` (gravity treat gate) | `esp32/esp32_claw.cpp` |
| `esp32/claw_node/claw_node.ino` (non-blocking two-servo claw) | `esp32/esp32_claw.cpp` |

They differ from the live code in ways that matter, winch 4 DIR on D8
instead of D11, an `ok`/`err` serial reply format the host no longer speaks,
no OLED, no OTA. Kept for reference only; nothing builds them.
