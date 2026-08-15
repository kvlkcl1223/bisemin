from __future__ import annotations

import sys

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QPlainTextEdit,
    QMessageBox,
    QSplitter,
    QStatusBar,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from app.logger import DataLogger
from app.models import CellState
from app.protocol import Frame, ProtocolError, bytes_to_hex, encode_frame, frame_to_text
from app.serial_worker import SerialClient
from app.widgets.cell_panel import (
    CellPanel,
    PROGRAM_COOL_RAMP_MAX_C_PER_MIN,
    PROGRAM_HEAT_RAMP_MAX_C_PER_MIN,
    RAMP_RATE_MAX_C_PER_MIN,
    RAMP_RATE_MIN_C_PER_MIN,
    TEMP_MAX_C,
    TEMP_MIN_C,
)
from app.widgets.calib_panel import CalibPanel
from app.widgets.log_panel import LogPanel


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Bisemin 上位机控制软件")
        self.resize(1280, 760)

        self.serial = SerialClient()
        self.serial.frame_received.connect(self.on_frame_received)
        self.serial.status_changed.connect(self.on_serial_status)
        self.serial.error_reported.connect(self.on_serial_error)

        self.seq = 1
        self.cells = [CellState(0), CellState(1)]
        self.logger = DataLogger()
        self._calib_read_cell: int | None = None
        self._calib_next_index = 0
        self._calib_step_count = 0

        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)

        top = QHBoxLayout()
        self.port_combo = QComboBox()
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["1000000", "921600", "115200"])
        self.baud_combo.setCurrentText("1000000")
        self.refresh_btn = QPushButton("刷新")
        self.connect_btn = QPushButton("连接")
        self.hello_btn = QPushButton("握手")
        self.state_btn = QPushButton("读取状态")
        self.link_label = QLabel("未连接")
        top.addWidget(QLabel("串口"))
        top.addWidget(self.port_combo)
        top.addWidget(QLabel("波特率"))
        top.addWidget(self.baud_combo)
        top.addWidget(self.refresh_btn)
        top.addWidget(self.connect_btn)
        top.addWidget(self.hello_btn)
        top.addWidget(self.state_btn)
        top.addStretch(1)
        top.addWidget(self.link_label)
        layout.addLayout(top)

        splitter = QSplitter()
        layout.addWidget(splitter, 1)

        left = QWidget()
        left_layout = QHBoxLayout(left)
        self.cell_panels = [CellPanel(0), CellPanel(1)]
        for panel in self.cell_panels:
            panel.start_normal.connect(self.start_normal)
            panel.stop_cell.connect(self.stop_cell)
            panel.set_program.connect(self.set_program)
            panel.start_program.connect(self.start_program)
            left_layout.addWidget(panel)
        splitter.addWidget(left)

        tabs = QTabWidget()
        self.calib_panel = CalibPanel()
        self.log_panel = LogPanel(self.logger)
        self.rx_log = QPlainTextEdit()
        self.rx_log.setReadOnly(True)
        self.tx_log = QPlainTextEdit()
        self.tx_log.setReadOnly(True)
        tabs.addTab(self.calib_panel, "校准")
        tabs.addTab(self.log_panel, "数据记录")
        tabs.addTab(self.rx_log, "接收")
        tabs.addTab(self.tx_log, "发送")
        splitter.addWidget(tabs)
        splitter.setSizes([760, 520])

        self.setStatusBar(QStatusBar())

        self.refresh_btn.clicked.connect(self.refresh_ports)
        self.connect_btn.clicked.connect(self.toggle_connection)
        self.hello_btn.clicked.connect(self.send_hello)
        self.state_btn.clicked.connect(self.get_state)
        self.calib_panel.start_calib.connect(self.start_calib)
        self.calib_panel.stop_calib.connect(self.stop_calib)
        self.calib_panel.refresh_status.connect(self.get_calib_status)
        self.calib_panel.read_result.connect(self.get_calib_result)

        self.heartbeat_timer = QTimer(self)
        self.heartbeat_timer.setInterval(1000)
        self.heartbeat_timer.timeout.connect(self.send_heartbeat)

        self.refresh_ports()

    def refresh_ports(self) -> None:
        current = self.port_combo.currentText()
        self.port_combo.clear()
        self.port_combo.addItems(self.serial.available_ports())
        if current:
            idx = self.port_combo.findText(current)
            if idx >= 0:
                self.port_combo.setCurrentIndex(idx)

    def toggle_connection(self) -> None:
        if self.serial.is_open():
            self.serial.close()
            self.heartbeat_timer.stop()
            self.connect_btn.setText("连接")
            return
        port = self.port_combo.currentText()
        if not port:
            self.statusBar().showMessage("未选择串口", 3000)
            return
        if self.serial.open(port, int(self.baud_combo.currentText())):
            self.connect_btn.setText("断开")
            self.heartbeat_timer.start()

    def next_seq(self) -> int:
        seq = self.seq
        self.seq = 1 if self.seq >= 0xFFFF else self.seq + 1
        return seq

    def send_frame(self, frame_type: str, seq: int = 0, **fields: object) -> None:
        try:
            data = encode_frame(frame_type, seq=seq, **fields)
        except (ProtocolError, UnicodeEncodeError, ValueError) as exc:
            self.statusBar().showMessage(f"协议编码错误：{exc}", 5000)
            return

        payload = ",".join(f"{key}={value}" for key, value in fields.items() if value is not None)
        self.tx_log.appendPlainText(
            f'类型={frame_type}, 序号={seq}, 载荷="{payload}"\n十六进制={bytes_to_hex(data)}'
        )
        self.serial.write(data)

    def send_cmd(self, op: str, **fields: object) -> None:
        self.send_frame("CMD", seq=self.next_seq(), op=op, **fields)

    def send_hello(self) -> None:
        self.send_frame("HELLO", seq=self.next_seq(), role="PC", proto=1, app="BiseminQt")

    def send_heartbeat(self) -> None:
        if self.serial.is_open():
            self.send_frame("HEARTBEAT", seq=self.next_seq())

    def get_state(self) -> None:
        self.send_cmd("GET_STATE")

    def start_normal(self, cell: int, temp: float) -> None:
        self.send_cmd("START_NORMAL", cell=cell, temp=f"{temp:.1f}")

    def stop_cell(self, cell: int) -> None:
        self.send_cmd("STOP", cell=cell)

    def set_program(
        self,
        cell: int,
        start: float,
        hold: int,
        rate: float,
        next_temp: float,
        wait: int,
        repeat: int,
    ) -> None:
        error = self.program_range_error(start, next_temp, rate, repeat)
        if error:
            QMessageBox.warning(self, "程序参数无效", error)
            return

        self.send_cmd(
            "SET_PROGRAM",
            cell=cell,
            start=f"{start:.1f}",
            hold=hold,
            rate=f"{rate:.1f}",
            next=f"{next_temp:.1f}",
            wait=wait,
            repeat=repeat,
        )

    def start_program(self, cell: int) -> None:
        self.send_cmd("START_PROGRAM", cell=cell)

    def start_calib(self, cell: int) -> None:
        self.send_cmd("START_CALIB", cell=cell)

    def stop_calib(self) -> None:
        self._reset_calib_read()
        self.send_cmd("STOP_CALIB")

    def get_calib_status(self) -> None:
        self.send_cmd("GET_CALIB_STATUS")

    def get_calib_result(self, cell: int) -> None:
        self._calib_read_cell = cell
        self._calib_next_index = 0
        self._calib_step_count = 0
        self.calib_panel.clear_result()
        self.send_cmd("GET_CALIB_RESULT", cell=cell)

    def _request_next_calib_step(self) -> None:
        if self._calib_read_cell is None:
            return
        if self._calib_next_index >= self._calib_step_count:
            self.calib_panel.set_progress(self._calib_step_count, self._calib_step_count)
            self.statusBar().showMessage("校准结果读取完成", 3000)
            self._reset_calib_read()
            return
        self.calib_panel.set_progress(self._calib_next_index, self._calib_step_count)
        self.send_cmd(
            "GET_CALIB_RESULT",
            cell=self._calib_read_cell,
            index=self._calib_next_index,
        )

    def _reset_calib_read(self) -> None:
        self._calib_read_cell = None
        self._calib_next_index = 0
        self._calib_step_count = 0

    def program_range_error(
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

    def on_frame_received(self, frame: Frame) -> None:
        self.rx_log.appendPlainText(f"{frame_to_text(frame)}\nHEX={bytes_to_hex(frame.raw)}")

        if frame.frame_type == "STATE":
            self.handle_state(frame.fields)
        elif frame.frame_type == "DATA":
            self.handle_state(frame.fields)
            self.handle_data(frame.fields)
        elif frame.frame_type == "ACK":
            if self.handle_ack(frame):
                return
            self.statusBar().showMessage(f"确认 seq={frame.seq}", 2000)
        elif frame.frame_type == "NACK":
            self.handle_nack(frame)
            self.statusBar().showMessage(
                f"拒绝 seq={frame.seq} 错误={frame.fields.get('err', '')}",
                5000,
            )
        elif frame.frame_type == "EVENT":
            self.statusBar().showMessage(
                f"事件 {frame.fields.get('type', '')} 单元={frame.fields.get('cell', '')}",
                3000,
            )
        elif frame.frame_type == "HELLO":
            self.statusBar().showMessage(f"MCU 握手成功，固件版本={frame.fields.get('fw', '')}", 3000)

    def handle_ack(self, frame: Frame) -> bool:
        op = frame.fields.get("op", "")
        if op == "CALIB_STATUS":
            self.calib_panel.apply_status(frame.fields)
            self.statusBar().showMessage("校准状态已更新", 2000)
            return True
        if op == "CALIB_META":
            self._calib_step_count = self.calib_panel.apply_meta(frame.fields)
            self._calib_next_index = 0
            self._request_next_calib_step()
            return True
        if op == "CALIB_STEP":
            self.calib_panel.apply_step(frame.fields)
            self._calib_next_index = int(frame.fields.get("index", "0")) + 1
            self._request_next_calib_step()
            return True
        if op == "START_CALIB":
            self.statusBar().showMessage("已发送开始校准命令", 3000)
            return True
        if op == "STOP_CALIB":
            self.statusBar().showMessage("已发送停止校准命令", 3000)
            return True
        return False

    def handle_nack(self, frame: Frame) -> None:
        msg = frame.fields.get("msg", "")
        err = frame.fields.get("err", "")
        if msg.startswith("CALIB") or err in {"1004", "1005", "1006"}:
            self._reset_calib_read()
            detail = frame.fields.get("detail", "")
            suffix = f"，detail={detail}" if detail else ""
            self.calib_panel.show_error(f"校准命令失败：{msg or err}{suffix}")

    def handle_state(self, fields: dict[str, str]) -> None:
        cell = int(fields.get("cell", "0"))
        if cell < 0 or cell >= len(self.cells):
            return
        try:
            self.cells[cell].update_from_fields(fields)
        except (ValueError, KeyError) as exc:
            self.statusBar().showMessage(f"状态帧解析失败：{exc}", 5000)
            return
        self.cell_panels[cell].apply_state(self.cells[cell])

    def handle_data(self, fields: dict[str, str]) -> None:
        row = self.logger.append_data_fields(fields)
        if row is not None:
            self.log_panel.append_row(row)

    def on_serial_status(self, status: str) -> None:
        self.link_label.setText(status)

    def on_serial_error(self, message: str) -> None:
        self.statusBar().showMessage(message, 5000)


def main() -> None:
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


