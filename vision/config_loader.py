"""Shared config loading for the vision and control scripts.

Kept separate so that vision/ and control/ resolve `config.yaml` and repo-
relative paths identically. Nothing here talks to hardware, so it imports on a
laptop as well as on the Pi.
"""

from __future__ import annotations

import os
from pathlib import Path

import yaml

# .../cdpr-petbot
REPO_ROOT = Path(__file__).resolve().parent.parent

DEFAULT_CONFIG = REPO_ROOT / "vision" / "config.yaml"
EXAMPLE_CONFIG = REPO_ROOT / "vision" / "config.example.yaml"


def load_config(path: str | os.PathLike | None = None) -> dict:
    """Load config.yaml, with a useful error if it has not been created yet."""
    cfg_path = Path(path) if path else DEFAULT_CONFIG

    if not cfg_path.exists():
        raise FileNotFoundError(
            f"{cfg_path} not found.\n"
            f"Copy the example and edit it:\n"
            f"    cp {EXAMPLE_CONFIG} {cfg_path}"
        )

    with open(cfg_path, "r") as fh:
        cfg = yaml.safe_load(fh)

    if not isinstance(cfg, dict):
        raise ValueError(f"{cfg_path} did not parse to a mapping")

    return cfg


def repo_path(relative: str) -> Path:
    """Resolve a repo-relative path from the config against the repo root."""
    p = Path(relative)
    return p if p.is_absolute() else REPO_ROOT / p


def anchors_from_config(cfg: dict) -> list[list[float]]:
    """Return anchors as [[x, y, z], ...] in A1..A4 order."""
    a = cfg["anchors"]
    return [[float(v) for v in a[key]] for key in ("a1", "a2", "a3", "a4")]
