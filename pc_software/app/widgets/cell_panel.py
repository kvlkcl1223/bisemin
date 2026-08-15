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

from app.models import MODE_TEXT, OWNER_TEXT, CellState


TEMP_MIN_C = -20.0
TEMP_MAX_C = 120.0
RAMP_RATE_MIN_C_PER_MIN = 0.1
RAMP_RATE_MAX_C_PER_MIN = 60.0
PROGRAM_HEAT_RAMP_MAX_C_PER_MIN = 60.0
PROGRAM_COOL_RAMP_MAX_C_PER_MIN = 5.0


class CellPanel(QWidget):
    start_normal = pyqtSignal(int, float)
    stop_cell = pyqtSignal(int)
    set_program = pyqtSignal(int, float, int, float, float, int, int)
    start_program = pyqtSignal(int)

    def __init__(self, cell: int, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.cell = cell

        root = QVBoxLayout(self)

        self.box = QGroupBox(f"单元 {cell}")
        root.addWidget(self.box)
        layout = QVBoxLayout(self.box)

        status_grid = QGridLayout()
        self.mode_label = QLabel("停止")
        self.owner_label = QLabel("无")
        self.running_label = QLabel("0")
        self.current_label = QLabel("25.0")
        self.aux_temp_label = QLabel("-")
        self.target_label = QLabel("25.0")
        self.duty_label = QLabel("0.000")
        self.error_label = QLabel("0")

        items = [
            ("模式", self.mode_label),
            ("控制来源", self.owner_label),
            ("运行状态", self.running_label),
            ("当前温度", self.current_label),
            ("环境/水温", self.aux_temp_label),
            ("目标温度", self.target_label),
            ("输出占空比", self.duty_label),
            ("错误码", self.error_label),
        ]
        for row, (name, widget) in enumerate(items):
            status_grid.addWidget(QLabel(name), row, 0)
            status_grid.addWidget(widget, row, 1)
        layout.addLayout(status_grid)

        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        layout.addWidget(line)

        normal_box = QGroupBox("普通控温")
        normal_layout = QFormLayout(normal_box)
        self.normal_temp = QDoubleSpinBox()
        self.normal_temp.setRange(TEMP_MIN_C, TEMP_MAX_C)
        self.normal_temp.setDecimals(1)
        self.normal_temp.setValue(37.5)
        self.normal_temp.setSuffix(" °C")
        self.start_normal_btn = QPushButton("启动普通控温")
        self.stop_btn = QPushButton("停止")
        normal_layout.addRow("目标温度", self.normal_temp)
        normal_layout.addRow(self.start_normal_btn)
        normal_layout.addRow(self.stop_btn)
        layout.addWidget(normal_box)

        program_box = QGroupBox("程序控温")
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
        self.set_program_btn = QPushButton("设置程序")
        self.start_program_btn = QPushButton("启动程序")
        self.stop_program_btn = QPushButton("停止程序")

        program_layout.addRow("起始温度", self.start_temp)
        program_layout.addRow("保持时间", self.hold_s)
        program_layout.addRow("升降速率", self.rate)
        program_layout.addRow("下一目标", self.next_temp)
        program_layout.addRow("等待时间", self.wait_s)
        program_layout.addRow("重复次数", self.repeat)
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
        self.mode_label.setText(MODE_TEXT.get(state.mode, state.mode.value))
        self.owner_label.setText(OWNER_TEXT.get(state.owner, state.owner.value))
        self.running_label.setText("运行" if state.running else "停止")
        self.current_label.setText(f"{state.current:.1f} °C")
        if state.aux_valid:
            self.aux_temp_label.setText(f"{state.aux_temp:.1f} °C")
        else:
            self.aux_temp_label.setText("无效")
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
            QMessageBox.warning(self, "程序参数无效", error)
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
            return f"起始温度必须在 {TEMP_MIN_C:.1f} 到 {TEMP_MAX_C:.1f} °C 之间。"
        if not TEMP_MIN_C <= next_temp <= TEMP_MAX_C:
            return f"下一目标温度必须在 {TEMP_MIN_C:.1f} 到 {TEMP_MAX_C:.1f} °C 之间。"
        if not RAMP_RATE_MIN_C_PER_MIN <= rate <= RAMP_RATE_MAX_C_PER_MIN:
            return (
                f"升降速率必须在 {RAMP_RATE_MIN_C_PER_MIN:.1f} 到 "
                f"{RAMP_RATE_MAX_C_PER_MIN:.1f} °C/min 之间。"
            )

        step = next_temp - start
        if step > 0.0 and rate > PROGRAM_HEAT_RAMP_MAX_C_PER_MIN:
            return f"升温速率不能超过 {PROGRAM_HEAT_RAMP_MAX_C_PER_MIN:.1f} °C/min。"
        if step < 0.0 and rate > PROGRAM_COOL_RAMP_MAX_C_PER_MIN:
            return f"降温速率不能超过 {PROGRAM_COOL_RAMP_MAX_C_PER_MIN:.1f} °C/min。"

        final_target = next_temp + step * repeat
        if not TEMP_MIN_C <= final_target <= TEMP_MAX_C:
            return (
                "程序最终目标超出温度限制。"
                f"最终目标将达到 {final_target:.1f} °C，允许范围是 "
                f"{TEMP_MIN_C:.1f} 到 {TEMP_MAX_C:.1f} °C。"
            )

        return None

