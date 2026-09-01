#!/usr/bin/env python3
"""Thin pyserial wrapper around the Arduino's command protocol.

The firmware (firmware/cdpr_controller/cdpr_controller.ino) prints "ready" once
on boot and replies to every command with exactly one line starting "ok" or
"err". This class enforces that contract: each method sends one command and
blocks until its acknowledgement arrives.

    from control.serial_link import SerialLink

    with SerialLink("/dev/ttyACM0") as link:
        link.set_home(1000, 1000, 1200)
        link.move(1200, 1000, 1200)
        print(link.report())
        link.disable()

Opening a serial port to an Uno resets the board, so the constructor waits for
"ready" rather than assuming the firmware is already running.

Moves are blocking on the Arduino side -- it does not service serial while the
motors are running -- so move() uses a longer timeout than the other commands.
"""

from __future__ import annotations

import time

import serial


class SerialLinkError(RuntimeError):
    """The Arduino replied with an error, or did not reply at all."""


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
            line = self._readline()
            if line == "ready":
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
        """Send one command, return its acknowledgement line.

        Raises SerialLinkError on an "err" reply or on timeout.
        """
        timeout = self.timeout_s if timeout_s is None else timeout_s

        self.ser.reset_input_buffer()
        self.ser.write((text + "\n").encode("ascii"))
        self.ser.flush()

        deadline = time.time() + timeout
        while time.time() < deadline:
            line = self._readline()
            if not line:
                continue
            if line.startswith("ok"):
                return line
            if line.startswith("err"):
                raise SerialLinkError(f"{text!r} rejected: {line}")
            # Anything else is unsolicited chatter; keep waiting for the ack.

        raise SerialLinkError(f"no reply to {text!r} within {timeout}s")

    # -- commands -----------------------------------------------------------

    def move(self, x: float, y: float, z: float) -> str:
        """Move the platform. Blocks until the Arduino says it arrived."""
        return self._command(
            f"M {x:.2f} {y:.2f} {z:.2f}", timeout_s=self.move_timeout_s
        )

    def report(self) -> str:
        """Return the raw position report line."""
        return self._command("R")

    def report_parsed(self) -> dict:
        """Parse the report line into a dict of floats and flags."""
        line = self.report()
        out: dict[str, float] = {}
        for token in line.split()[1:]:      # skip the leading "ok"
            if "=" not in token:
                continue
            key, _, value = token.partition("=")
            try:
                out[key] = float(value)
            except ValueError:
                pass
        return out

    def set_home(self, x: float, y: float, z: float) -> str:
        """Tell the firmware where the platform currently is."""
        return self._command(f"H {x:.2f} {y:.2f} {z:.2f}")

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
