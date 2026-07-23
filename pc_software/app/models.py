from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class ControlMode(str, Enum):
    STOP = "STOP"
    NORMAL = "NORMAL"
    PROGRAM = "PROGRAM"


class ControlOwner(str, Enum):
    NONE = "NONE"
    PANEL = "PANEL"
    PC = "PC"


@dataclass
class ProgramParams:
    start: float = 25.0
    hold: int = 60
    rate: float = 2.0
    next: float = 35.0
    wait: int = 30
    repeat: int = 3


@dataclass
class CellState:
    cell: int
    mode: ControlMode = ControlMode.STOP
    owner: ControlOwner = ControlOwner.NONE
    running: bool = False
    target: float = 25.0
    command: float = 25.0
    current: float = 25.0
    t0: float = 25.0
    t1: float = 25.0
    duty: float = 0.0
    error: int = 0
    phase: int = 0
    program: ProgramParams = None  # type: ignore[assignment]

    def __post_init__(self) -> None:
        if self.program is None:
            self.program = ProgramParams()

    def update_from_fields(self, fields: dict[str, str]) -> None:
        if "mode" in fields:
            self.mode = ControlMode(fields["mode"])
        if "owner" in fields:
            self.owner = ControlOwner(fields["owner"])
        if "running" in fields:
            self.running = fields["running"] == "1"
        if "target" in fields:
            self.target = float(fields["target"])
        if "command" in fields:
            self.command = float(fields["command"])
        if "current" in fields:
            self.current = float(fields["current"])
        if "t0" in fields:
            self.t0 = float(fields["t0"])
        if "t1" in fields:
            self.t1 = float(fields["t1"])
        if "duty" in fields:
            self.duty = float(fields["duty"])
        if "error" in fields:
            self.error = int(fields["error"])
        if "phase" in fields:
            self.phase = int(fields["phase"])
