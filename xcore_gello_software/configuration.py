"""Defaults for the local GELLO read entry point from ``config/default.toml``."""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, fields
from pathlib import Path
from typing import Any

try:
    import tomllib
except ModuleNotFoundError:  # Python 3.8-3.10 remain supported.
    import tomli as tomllib


def _default_config_path() -> Path | None:
    package = Path(__file__).resolve().parent
    # This package lives at <repository>/xcore_gello_software, unlike the src/
    # layouts of the other subprojects.
    checkout = package.parent / "config/default.toml"
    if checkout.exists():
        return checkout
    bundled = package / "default.toml"
    if bundled.exists():
        return bundled
    return None


DEFAULT_CONFIG = _default_config_path()


@dataclass(frozen=True)
class ReadConfig:
    gello_port: str = (
        "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB4C7PQ-if00-port0"
    )
    hz: float = 10.0
    samples: int = 0
    gripper_open_deg: float = 0.0
    gripper_close_deg: float = 0.0
    gripper_use_library_endpoints: bool = True


@dataclass(frozen=True)
class Config:
    path: Path | None
    read: ReadConfig = ReadConfig()

    def validate(self) -> None:
        sections = {"read": self.read}
        for section, values in sections.items():
            for item in fields(values):
                value = getattr(values, item.name)
                default = item.default
                kind = type(default)
                valid = type(value) is kind
                if kind is float:
                    valid = (
                        not isinstance(value, bool)
                        and isinstance(value, (int, float))
                        and math.isfinite(value)
                    )
                if not valid:
                    raise ValueError(
                        f"{section}.{item.name} has an invalid type or non-finite value"
                    )
        if not isinstance(self.read.gello_port, str) or not self.read.gello_port:
            raise ValueError("read.gello_port must be a non-empty string")
        if self.read.hz <= 0:
            raise ValueError("read.hz must be positive")
        if self.read.samples < 0:
            raise ValueError("read.samples must be nonnegative")
        open_deg = self.read.gripper_open_deg
        close_deg = self.read.gripper_close_deg
        if not self.read.gripper_use_library_endpoints and open_deg == close_deg:
            raise ValueError(
                "read gripper endpoints must differ unless library defaults are used"
            )


def load_config(path: str | Path | None = None) -> Config:
    override = os.environ.get("XCORE_GELLO_CONFIG")
    selected = (
        Path(path or override).resolve() if (path or override) else DEFAULT_CONFIG
    )
    if selected is None:
        data: dict[str, Any] = {}
    else:
        with Path(selected).open("rb") as stream:
            data = tomllib.load(stream)
    unknown = data.keys() - {"read"}
    if unknown:
        raise ValueError(f"unknown configuration sections: {sorted(unknown)}")
    values = data.get("read", {})
    if not isinstance(values, dict):
        raise TypeError("read must be a TOML table")
    unknown = values.keys() - {field.name for field in fields(ReadConfig)}
    if unknown:
        raise ValueError(f"unknown read options: {sorted(unknown)}")
    try:
        read = ReadConfig(**values)
    except TypeError as exc:
        raise ValueError(f"invalid read options: {exc}") from exc
    result = Config(
        path=Path(selected) if selected is not None else None,
        read=read,
    )
    result.validate()
    return result
