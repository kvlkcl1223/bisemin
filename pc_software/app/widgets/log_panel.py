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
        self.status = QLabel("记录已停止")
        self.start_btn = QPushButton("开始记录")
        self.stop_btn = QPushButton("停止记录")
        self.export_btn = QPushButton("导出 CSV")
        toolbar.addWidget(self.status)
        toolbar.addStretch(1)
        toolbar.addWidget(self.start_btn)
        toolbar.addWidget(self.stop_btn)
        toolbar.addWidget(self.export_btn)
        root.addLayout(toolbar)

        self.table = QTableWidget(0, 9)
        self.table.setHorizontalHeaderLabels([
            "MCU 时间 ms",
            "单元",
            "模式",
            "阶段",
            "目标温度",
            "当前温度",
            "占空比",
            "错误码",
        ])
        self.table.setHorizontalHeaderLabels([
            "MCU 时间 ms",
            "单元",
            "模式",
            "阶段",
            "目标温度",
            "当前温度",
            "环境/水温",
            "占空比",
            "错误码",
        ])
        self.table.horizontalHeader().setStretchLastSection(True)
        root.addWidget(self.table)

        self.start_btn.clicked.connect(self.start_log)
        self.stop_btn.clicked.connect(self.stop_log)
        self.export_btn.clicked.connect(self.export_csv)

    def start_log(self) -> None:
        self.logger.start()
        self.table.setRowCount(0)
        self.status.setText("正在记录")
        self.log_started.emit()

    def stop_log(self) -> None:
        self.logger.stop()
        self.status.setText(f"记录已停止，行数={len(self.logger.rows)}")
        self.log_stopped.emit()

    def append_row(self, row: LogRow) -> None:
        r = self.table.rowCount()
        self.table.insertRow(r)
        values = [
            str(row.mcu_time_ms),
            row.pool,
            row.mode,
            str(row.phase),
            f"{row.target:.1f}",
            f"{row.current:.1f}",
            f"{row.aux_temp:.1f}",
            f"{row.duty:.3f}",
            str(row.error),
        ]
        for c, value in enumerate(values):
            self.table.setItem(r, c, QTableWidgetItem(value))
        self.table.scrollToBottom()

    def export_csv(self) -> None:
        path, _ = QFileDialog.getSaveFileName(
            self,
            "导出 CSV",
            "bisemin_log.csv",
            "CSV 文件 (*.csv)",
        )
        if not path:
            return
        self.logger.export_csv(path)
        self.status.setText(f"已导出 {path}")
