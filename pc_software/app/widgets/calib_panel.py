from __future__ import annotations

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import (
    QAbstractItemView,
    QComboBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)


STATE_TEXT = {
    0: "空闲",
    1: "初始化",
    2: "设置输出",
    3: "等待稳定",
    4: "完成",
    5: "故障",
}


class CalibPanel(QWidget):
    start_calib = pyqtSignal(int)
    stop_calib = pyqtSignal()
    refresh_status = pyqtSignal()
    read_result = pyqtSignal(int)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)

        root = QVBoxLayout(self)

        control_box = QGroupBox("校准控制")
        control_layout = QGridLayout(control_box)
        self.cell_combo = QComboBox()
        self.cell_combo.addItems(["温控池 1", "温控池 2"])
        self.start_btn = QPushButton("开始校准")
        self.stop_btn = QPushButton("停止校准")
        self.status_btn = QPushButton("读取状态")
        self.result_btn = QPushButton("读取结果")

        control_layout.addWidget(QLabel("对象"), 0, 0)
        control_layout.addWidget(self.cell_combo, 0, 1)
        control_layout.addWidget(self.start_btn, 0, 2)
        control_layout.addWidget(self.stop_btn, 0, 3)
        control_layout.addWidget(self.status_btn, 1, 2)
        control_layout.addWidget(self.result_btn, 1, 3)
        root.addWidget(control_box)

        status_box = QGroupBox("当前状态")
        status_layout = QGridLayout(status_box)
        self.state_label = QLabel("未知")
        self.running_label = QLabel("-")
        self.active_label = QLabel("-")
        self.cell_label = QLabel("-")
        self.index_label = QLabel("-")
        self.error_label = QLabel("-")
        self.meta_label = QLabel("未读取")
        labels = [
            ("状态", self.state_label),
            ("运行", self.running_label),
            ("激活", self.active_label),
            ("单元", self.cell_label),
            ("步骤", self.index_label),
            ("错误", self.error_label),
            ("结果", self.meta_label),
        ]
        for row, (name, widget) in enumerate(labels):
            status_layout.addWidget(QLabel(name), row, 0)
            status_layout.addWidget(widget, row, 1)
        root.addWidget(status_box)

        progress_row = QHBoxLayout()
        self.progress_label = QLabel("读取进度：-")
        progress_row.addWidget(self.progress_label)
        progress_row.addStretch(1)
        root.addLayout(progress_row)

        self.table = QTableWidget(0, 6)
        self.table.setHorizontalHeaderLabels(["序号", "占空比", "左温", "右温", "有效", "稳定"])
        self.table.verticalHeader().setVisible(False)
        self.table.setAlternatingRowColors(True)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.Stretch)
        root.addWidget(self.table, 1)

        self.start_btn.clicked.connect(lambda: self.start_calib.emit(self.cell_combo.currentIndex()))
        self.stop_btn.clicked.connect(self.stop_calib.emit)
        self.status_btn.clicked.connect(self.refresh_status.emit)
        self.result_btn.clicked.connect(lambda: self.read_result.emit(self.cell_combo.currentIndex()))

    def apply_status(self, fields: dict[str, str]) -> None:
        state = int(fields.get("state", "-1"))
        self.state_label.setText(STATE_TEXT.get(state, str(state)))
        self.running_label.setText("是" if fields.get("running") == "1" else "否")
        self.active_label.setText("是" if fields.get("active") == "1" else "否")
        self.cell_label.setText(fields.get("cell", "-"))
        index = fields.get("index", "-")
        count = fields.get("count", "-")
        self.index_label.setText(f"{index} / {count}")
        self.error_label.setText(fields.get("error", "0"))

    def apply_meta(self, fields: dict[str, str]) -> int:
        count = int(fields.get("count", "0"))
        self.table.setRowCount(count)
        self.meta_label.setText(
            "温控池 {cell}，{count} 点，{start} 到 {end}，步进 {step}，CRC {crc}".format(
                cell=int(fields.get("cell", "0")) + 1,
                count=count,
                start=fields.get("start", "-"),
                end=fields.get("end", "-"),
                step=fields.get("step", "-"),
                crc=fields.get("crc", "-"),
            )
        )
        for row in range(count):
            self._set_item(row, 0, str(row))
        return count

    def apply_step(self, fields: dict[str, str]) -> None:
        row = int(fields.get("index", "0"))
        if row >= self.table.rowCount():
            self.table.setRowCount(row + 1)
        self._set_item(row, 0, str(row))
        self._set_item(row, 1, fields.get("duty", ""))
        self._set_item(row, 2, fields.get("t0", ""))
        self._set_item(row, 3, fields.get("t1", ""))
        self._set_item(row, 4, "是" if fields.get("valid") == "1" else "否")
        self._set_item(row, 5, "是" if fields.get("settled") == "1" else "否")
        self.table.scrollToItem(self.table.item(row, 0))

    def clear_result(self) -> None:
        self.table.setRowCount(0)
        self.meta_label.setText("读取中")
        self.progress_label.setText("读取进度：-")

    def set_progress(self, done: int, total: int) -> None:
        self.progress_label.setText(f"读取进度：{done} / {total}")

    def show_error(self, message: str) -> None:
        self.progress_label.setText(message)

    def _set_item(self, row: int, col: int, text: str) -> None:
        self.table.setItem(row, col, QTableWidgetItem(text))
