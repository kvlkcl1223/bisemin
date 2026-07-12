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
from app.widgets.cell_panel import CellPanel
from app.widgets.log_panel import LogPanel


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Bisemin PC Control")
        self.resize(1280, 760)

        self.serial = SerialClient()
        self.serial.frame_received.connect(self.on_frame_received)
        self.serial.status_changed.connect(self.on_serial_status)
        self.serial.error_reported.connect(self.on_serial_error)

        self.seq = 1
        self.cells = [CellState(0), CellState(1)]
        self.logger = DataLogger()

        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)

        top = QHBoxLayout()
        self.port_combo = QComboBox()
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["1000000", "921600", "115200"])
        self.baud_combo.setCurrentText("1000000")
        self.refresh_btn = QPushButton("Refresh")
        self.connect_btn = QPushButton("Connect")
        self.hello_btn = QPushButton("HELLO")
        self.state_btn = QPushButton("GET_STATE")
        self.link_label = QLabel("Disconnected")
        top.addWidget(QLabel("Port"))
        top.addWidget(self.port_combo)
        top.addWidget(QLabel("Baud"))
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
        self.log_panel = LogPanel(self.logger)
        self.rx_log = QPlainTextEdit()
        self.rx_log.setReadOnly(True)
        self.tx_log = QPlainTextEdit()
        self.tx_log.setReadOnly(True)
        tabs.addTab(self.log_panel, "Data Log")
        tabs.addTab(self.rx_log, "RX")
        tabs.addTab(self.tx_log, "TX")
        splitter.addWidget(tabs)
        splitter.setSizes([760, 520])

        self.setStatusBar(QStatusBar())

        self.refresh_btn.clicked.connect(self.refresh_ports)
        self.connect_btn.clicked.connect(self.toggle_connection)
        self.hello_btn.clicked.connect(self.send_hello)
        self.state_btn.clicked.connect(self.get_state)

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
            self.connect_btn.setText("Connect")
            return
        port = self.port_combo.currentText()
        if not port:
            self.statusBar().showMessage("No serial port selected", 3000)
            return
        if self.serial.open(port, int(self.baud_combo.currentText())):
            self.connect_btn.setText("Disconnect")
            self.heartbeat_timer.start()

    def next_seq(self) -> int:
        seq = self.seq
        self.seq = 1 if self.seq >= 0xFFFF else self.seq + 1
        return seq

    def send_frame(self, frame_type: str, seq: int = 0, **fields: object) -> None:
        try:
            data = encode_frame(frame_type, seq=seq, **fields)
        except (ProtocolError, UnicodeEncodeError, ValueError) as exc:
            self.statusBar().showMessage(f"Protocol encode error: {exc}", 5000)
            return

        payload = ",".join(f"{key}={value}" for key, value in fields.items() if value is not None)
        self.tx_log.appendPlainText(
            f'TYPE={frame_type}, SEQ={seq}, PAYLOAD="{payload}"\nHEX={bytes_to_hex(data)}'
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

    def on_frame_received(self, frame: Frame) -> None:
        self.rx_log.appendPlainText(f"{frame_to_text(frame)}\nHEX={bytes_to_hex(frame.raw)}")

        if frame.frame_type == "STATE":
            self.handle_state(frame.fields)
        elif frame.frame_type == "DATA":
            self.handle_state(frame.fields)
            self.handle_data(frame.fields)
        elif frame.frame_type == "ACK":
            self.statusBar().showMessage(f"ACK seq={frame.seq}", 2000)
        elif frame.frame_type == "NACK":
            self.statusBar().showMessage(
                f"NACK seq={frame.seq} err={frame.fields.get('err', '')}",
                5000,
            )
        elif frame.frame_type == "EVENT":
            self.statusBar().showMessage(
                f"EVENT {frame.fields.get('type', '')} cell={frame.fields.get('cell', '')}",
                3000,
            )
        elif frame.frame_type == "HELLO":
            self.statusBar().showMessage(f"MCU HELLO fw={frame.fields.get('fw', '')}", 3000)

    def handle_state(self, fields: dict[str, str]) -> None:
        cell = int(fields.get("cell", "0"))
        if cell < 0 or cell >= len(self.cells):
            return
        try:
            self.cells[cell].update_from_fields(fields)
        except (ValueError, KeyError) as exc:
            self.statusBar().showMessage(f"Bad STATE: {exc}", 5000)
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


