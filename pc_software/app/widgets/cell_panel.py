from __future__ import annotations

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import (
    QDoubleSpinBox,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QLabel,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from app.models import CellState


TEMP_MIN_C = -10.0
TEMP_MAX_C = 110.0
RAMP_RATE_MIN_C_PER_MIN = 0.1
RAMP_RATE_MAX_C_PER_MIN = 60.0


class CellPanel(QWidget):
    start_normal = pyqtSignal(int, float)
    stop_cell = pyqtSignal(int)
    set_program = pyqtSignal(int, float, int, float, float, int, int)
    start_program = pyqtSignal(int)

    def __init__(self, cell: int, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.cell = cell

        root = QVBoxLayout(self)

        self.box = QGroupBox(f"Cell {cell}")
        root.addWidget(self.box)
        layout = QVBoxLayout(self.box)

        status_grid = QGridLayout()
        self.mode_label = QLabel("STOP")
        self.owner_label = QLabel("NONE")
        self.running_label = QLabel("0")
        self.current_label = QLabel("25.0")
        self.target_label = QLabel("25.0")
        self.duty_label = QLabel("0.000")
        self.error_label = QLabel("0")

        items = [
            ("Mode", self.mode_label),
            ("Owner", self.owner_label),
            ("Running", self.running_label),
            ("Current", self.current_label),
            ("Target", self.target_label),
            ("Duty", self.duty_label),
            ("Error", self.error_label),
        ]
        for row, (name, widget) in enumerate(items):
            status_grid.addWidget(QLabel(name), row, 0)
            status_grid.addWidget(widget, row, 1)
        layout.addLayout(status_grid)

        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        layout.addWidget(line)

        normal_box = QGroupBox("Normal")
        normal_layout = QFormLayout(normal_box)
        self.normal_temp = QDoubleSpinBox()
        self.normal_temp.setRange(TEMP_MIN_C, TEMP_MAX_C)
        self.normal_temp.setDecimals(1)
        self.normal_temp.setValue(37.5)
        self.normal_temp.setSuffix(" °C")
        self.start_normal_btn = QPushButton("Start Normal")
        self.stop_btn = QPushButton("Stop")
        normal_layout.addRow("Target", self.normal_temp)
        normal_layout.addRow(self.start_normal_btn)
        normal_layout.addRow(self.stop_btn)
        layout.addWidget(normal_box)

        program_box = QGroupBox("Program")
        program_layout = QFormLayout(program_box)
        self.start_temp = QDoubleSpinBox()
        self.start_temp.setRange(TEMP_MIN_C, TEMP_MAX_C)
        self.start_temp.setDecimals(1)
        self.start_temp.setValue(25.0)
        self.start_temp.setSuffix(" °C")
        self.hold_s = QSpinBox()
        self.hold_s.setRange(0, 36000)
        self.hold_s.setValue(60)
        self.hold_s.setSuffix(" s")
        self.rate = QDoubleSpinBox()
        self.rate.setRange(RAMP_RATE_MIN_C_PER_MIN, RAMP_RATE_MAX_C_PER_MIN)
        self.rate.setDecimals(1)
        self.rate.setValue(2.0)
        self.rate.setSuffix(" °C/min")
        self.next_temp = QDoubleSpinBox()
        self.next_temp.setRange(TEMP_MIN_C, TEMP_MAX_C)
        self.next_temp.setDecimals(1)
        self.next_temp.setValue(35.0)
        self.next_temp.setSuffix(" °C")
        self.wait_s = QSpinBox()
        self.wait_s.setRange(0, 36000)
        self.wait_s.setValue(30)
        self.wait_s.setSuffix(" s")
        self.repeat = QSpinBox()
        self.repeat.setRange(0, 999)
        self.repeat.setValue(3)
        self.set_program_btn = QPushButton("Set Program")
        self.start_program_btn = QPushButton("Start Program")
        self.stop_program_btn = QPushButton("Stop Program")

        program_layout.addRow("Start", self.start_temp)
        program_layout.addRow("Hold", self.hold_s)
        program_layout.addRow("Rate", self.rate)
        program_layout.addRow("Next", self.next_temp)
        program_layout.addRow("Wait", self.wait_s)
        program_layout.addRow("Repeat", self.repeat)
        program_layout.addRow(self.set_program_btn)
        program_layout.addRow(self.start_program_btn)
        program_layout.addRow(self.stop_program_btn)
        layout.addWidget(program_box)

        self.start_normal_btn.clicked.connect(self._emit_start_normal)
        self.stop_btn.clicked.connect(lambda: self.stop_cell.emit(self.cell))
        self.set_program_btn.clicked.connect(self._emit_set_program)
        self.start_program_btn.clicked.connect(lambda: self.start_program.emit(self.cell))
        self.stop_program_btn.clicked.connect(lambda: self.stop_cell.emit(self.cell))

    def apply_state(self, state: CellState) -> None:
        self.mode_label.setText(state.mode.value)
        self.owner_label.setText(state.owner.value)
        self.running_label.setText("1" if state.running else "0")
        self.current_label.setText(f"{state.current:.1f} °C")
        self.target_label.setText(f"{state.target:.1f} °C")
        self.duty_label.setText(f"{state.duty:.3f}")
        self.error_label.setText(str(state.error))

    def _emit_start_normal(self) -> None:
        self.start_normal.emit(self.cell, self.normal_temp.value())

    def _emit_set_program(self) -> None:
        start = self.start_temp.value()
        next_temp = self.next_temp.value()
        repeat = self.repeat.value()
        error = self._program_range_error(start, next_temp, self.rate.value(), repeat)
        if error:
            QMessageBox.warning(self, "Invalid Program", error)
            return

        self.set_program.emit(
            self.cell,
            start,
            self.hold_s.value(),
            self.rate.value(),
            next_temp,
            self.wait_s.value(),
            repeat,
        )

    def _program_range_error(
        self,
        start: float,
        next_temp: float,
        rate: float,
        repeat: int,
    ) -> str | None:
        if not TEMP_MIN_C <= start <= TEMP_MAX_C:
            return f"Start must be between {TEMP_MIN_C:.1f} and {TEMP_MAX_C:.1f} °C."
        if not TEMP_MIN_C <= next_temp <= TEMP_MAX_C:
            return f"Next must be between {TEMP_MIN_C:.1f} and {TEMP_MAX_C:.1f} °C."
        if not RAMP_RATE_MIN_C_PER_MIN <= rate <= RAMP_RATE_MAX_C_PER_MIN:
            return (
                f"Rate must be between {RAMP_RATE_MIN_C_PER_MIN:.1f} and "
                f"{RAMP_RATE_MAX_C_PER_MIN:.1f} °C/min."
            )

        step = next_temp - start
        final_target = next_temp + step * repeat
        if not TEMP_MIN_C <= final_target <= TEMP_MAX_C:
            return (
                "Program range exceeds the temperature limit. "
                f"Final target would be {final_target:.1f} °C, but allowed range is "
                f"{TEMP_MIN_C:.1f} to {TEMP_MAX_C:.1f} °C."
            )

        return None
