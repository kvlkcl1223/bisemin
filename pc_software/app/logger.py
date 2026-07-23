from __future__ import annotations

import csv
from dataclasses import dataclass, asdict
from pathlib import Path


@dataclass
class LogRow:
    mcu_time_ms: int
    pool: str
    mode: str
    phase: int
    target: float
    current: float
    duty: float
    error: int


class DataLogger:
    def __init__(self) -> None:
        self.rows: list[LogRow] = []
        self.active = False

    def start(self) -> None:
        self.rows.clear()
        self.active = True

    def stop(self) -> None:
        self.active = False

    def append_data_fields(self, fields: dict[str, str]) -> LogRow | None:
        if not self.active:
            return None
        cell = int(fields.get("cell", "0"))
        row = LogRow(
            mcu_time_ms=int(fields.get("t", "0")),
            pool=f"Pool {cell + 1}",
            mode=fields.get("mode", ""),
            phase=int(fields.get("phase", "0")),
            target=float(fields.get("target", "0")),
            current=float(fields.get("current", "0")),
            duty=float(fields.get("duty", "0")),
            error=int(fields.get("error", "0")),
        )
        self.rows.append(row)
        return row

    def export_csv(self, path: str | Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="", encoding="utf-8-sig") as f:
            writer = csv.DictWriter(f, fieldnames=list(asdict(LogRow(0, "", "", 0, 0, 0, 0, 0)).keys()))
            writer.writeheader()
            for row in self.rows:
                writer.writerow(asdict(row))