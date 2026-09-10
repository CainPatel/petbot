#!/usr/bin/env python3
"""Thin pyserial wrapper around the Arduino's command protocol.

The firmware (firmware/uno_winches.cpp) prints "ready" once on boot and
replies to each command with one line whose prefix identifies the command:

    M x y z   ->  "moved: x y z"      (after the blocking move completes)
    R         ->  "pos: x y z"
    H x y z   ->  "home: x y z"
    D         ->  "disabled"
    E         ->  "enabled"

A move outside the firmware's safe box is NOT refused. The firmware prints
"WARN: outside safe workspace, cables may go slack" and then moves anyway, so
the host must do its own clamping (see control.py / control/mission.py). The
warning line is captured in `last_warning` for the caller to surface.

    from control.serial_link import SerialLink

    with SerialLink("/dev/ttyACM0") as link:
        link.set_home(2232, 1829, 1600)
        link.move(1500, 1750, 2400)
        print(link.report_parsed())

Opening a serial port to an Uno resets the board, so the constructor waits for
"ready" rather than assuming the firmware is already running. Moves block on
the Arduino side -- it does not service serial while the motors run -- so
move() uses a longer timeout than the other commands.
"""

from __future__ import annotations

import time

import serial

# Reply prefix the firmware sends for each command letter.
REPLY_PREFIX = {
    "M": "moved:",
    "R": "pos:",
    "H": "home:",
    "D": "disabled",
    "E": "enabled",
}


class SerialLinkError(RuntimeError):
    """The Arduino did not reply, or replied with something unexpected."""


class SerialLink:
    def __init__(
        self,
        port: str,
        baud: int = 115200,
        timeout_s: float = 2.0,
        move_timeout_s: float = 60.0,
        boot_timeout_s: float = 10.0,
        wait_ready: bool = True,
    ):
        self.port = port
        self.baud = baud
        self.timeout_s = timeout_s
        self.move_timeout_s = move_timeout_s
        self.boot_timeout_s = boot_timeout_s
        self.last_warning: str | None = None

        self.ser = serial.Serial(port, baud, timeout=timeout_s)

        if wait_ready:
            self._wait_ready()

    # -- lifecycle ----------------------------------------------------------

    def _wait_ready(self) -> None:
        """Block until the firmware announces itself.

        Opening the port toggles DTR, which resets the Uno; the bootloader then
        takes a moment before setup() runs. Anything the port emits before
        "ready" is boot noise and is discarded.
        """
        deadline = time.time() + self.boot_timeout_s
        while time.time() < deadline:
            if self._readline() == "ready":
                return
        raise SerialLinkError(
            f"no 'ready' from {self.port} within {self.boot_timeout_s}s. "
            f"Check the port, the baud rate ({self.baud}), and that the "
            f"sketch is actually flashed."
        )

    def close(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()

    def __enter__(self) -> "SerialLink":
        return self

    def __exit__(self, exc_type, exc, tb) -> bool:
        # Close the port even if the caller blew up mid-move, so the next run
        # is not locked out of the device.
        self.close()
        return False

    # -- plumbing -----------------------------------------------------------

    def _readline(self) -> str:
        return self.ser.readline().decode("utf-8", errors="replace").strip()

    def _command(self, text: str, timeout_s: float | None = None) -> str:
        """Send one command and return the reply line for it.

        Raises SerialLinkError on timeout.
        """
        expect = REPLY_PREFIX[text[0]]
        timeout = self.timeout_s if timeout_s is None else timeout_s
        self.last_warning = None

        self.ser.reset_input_buffer()
        self.ser.write((text + "\n").encode("ascii"))
        self.ser.flush()

        deadline = time.time() + timeout
        while time.time() < deadline:
            line = self._readline()
            if not line:
                continue
            if line.startswith("WARN"):
                self.last_warning = line
                continue
            if line.startswith(expect):
                return line
            # Anything else is unsolicited chatter; keep waiting.

        raise SerialLinkError(f"no '{expect}' reply to {text!r} within {timeout}s")

    @staticmethod
    def _xyz(line: str) -> tuple[float, float, float]:
        """'moved: 1500.00 1750.00 2400.00' -> (1500.0, 1750.0, 2400.0)."""
        parts = line.split(":", 1)[1].split()
        if len(parts) != 3:
            raise SerialLinkError(f"could not parse position from {line!r}")
        return tuple(float(p) for p in parts)  # type: ignore[return-value]

    # -- commands -----------------------------------------------------------

    def move(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        """Move the platform. Blocks until the Arduino says it arrived."""
        line = self._command(f"M {x:.2f} {y:.2f} {z:.2f}",
                             timeout_s=self.move_timeout_s)
        return self._xyz(line)

    def report(self) -> str:
        """Return the raw 'pos: x y z' line."""
        return self._command("R")

    def report_parsed(self) -> dict:
        x, y, z = self._xyz(self.report())
        return {"x": x, "y": y, "z": z}

    def set_home(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        """Tell the firmware where the platform currently is."""
        return self._xyz(self._command(f"H {x:.2f} {y:.2f} {z:.2f}"))

    def enable(self) -> str:
        return self._command("E")

    def disable(self) -> str:
        """Release the motors. The platform will sag under its own weight."""
        return self._command("D")


def from_config(cfg: dict, **overrides) -> SerialLink:
    """Build a SerialLink from the `serial:` block of config.yaml."""
    s = cfg["serial"]
    kwargs = dict(
        port=s["port"],
        baud=int(s.get("baud", 115200)),
        timeout_s=float(s.get("timeout_s", 2.0)),
        move_timeout_s=float(s.get("move_timeout_s", 60.0)),
    )
    kwargs.update(overrides)
    return SerialLink(**kwargs)
