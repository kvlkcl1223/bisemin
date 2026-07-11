from __future__ import annotations

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import (
    QFileDialog,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from app.logger import DataLogger, LogRow


class LogPanel(QWidget):
    log_started = pyqtSignal()
    log_stopped = pyqtSignal()

    def __init__(self, logger: DataLogger, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.logger = logger

        root = QVBoxLayout(self)
        toolbar = QHBoxLayout()
        self.status = QLabel("Log stopped")
        self.start_btn = QPushButton("Start Log")
        self.stop_btn = QPushButton("Stop Log")
        self.export_btn = QPushButton("Export CSV")
        toolbar.addWidget(self.status)
        toolbar.addStretch(1)
        toolbar.addWidget(self.start_btn)
        toolbar.addWidget(self.stop_btn)
        toolbar.addWidget(self.export_btn)
        root.addLayout(toolbar)

        self.table = QTableWidget(0, 8)
        self.table.setHorizontalHeaderLabels([
            "PC Time",
            "MCU ms",
            "Cell",
            "Mode",
            "Target",
            "Current",
            "Duty",
            "Error",
        ])
        self.table.horizontalHeader().setStretchLastSection(True)
        root.addWidget(self.table)

        self.start_btn.clicked.connect(self.start_log)
        self.stop_btn.clicked.connect(self.stop_log)
        self.export_btn.clicked.connect(self.export_csv)

    def start_log(self) -> None:
        self.logger.start()
        self.table.setRowCount(0)
        self.status.setText("Logging")
        self.log_started.emit()

    def stop_log(self) -> None:
        self.logger.stop()
        self.status.setText(f"Log stopped, rows={len(self.logger.rows)}")
        self.log_stopped.emit()

    def append_row(self, row: LogRow) -> None:
        r = self.table.rowCount()
        self.table.insertRow(r)
        values = [
            row.pc_time,
            str(row.mcu_time_ms),
            str(row.cell),
            row.mode,
            f"{row.target:.2f}",
            f"{row.current:.2f}",
            f"{row.duty:.3f}",
            str(row.error),
        ]
        for c, value in enumerate(values):
            self.table.setItem(r, c, QTableWidgetItem(value))
        self.table.scrollToBottom()

    def export_csv(self) -> None:
        path, _ = QFileDialog.getSaveFileName(
            self,
            "Export CSV",
            "bisemin_log.csv",
            "CSV Files (*.csv)",
        )
        if not path:
            return
        self.logger.export_csv(path)
        self.status.setText(f"Exported {path}")
