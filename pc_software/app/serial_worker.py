from __future__ import annotations

from PyQt5.QtCore import QObject, pyqtSignal
from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo


class SerialClient(QObject):
    line_received = pyqtSignal(bytes)
    status_changed = pyqtSignal(str)
    error_reported = pyqtSignal(str)

    def __init__(self) -> None:
        super().__init__()
        self.port = QSerialPort(self)
        self.port.readyRead.connect(self._on_ready_read)
        self.port.errorOccurred.connect(self._on_error)
        self._buffer = bytearray()

    @staticmethod
    def available_ports() -> list[str]:
        return [port.portName() for port in QSerialPortInfo.availablePorts()]

    def is_open(self) -> bool:
        return self.port.isOpen()

    def open(self, port_name: str, baudrate: int) -> bool:
        if self.port.isOpen():
            self.port.close()
        self.port.setPortName(port_name)
        self.port.setBaudRate(baudrate)
        self.port.setDataBits(QSerialPort.Data8)
        self.port.setParity(QSerialPort.NoParity)
        self.port.setStopBits(QSerialPort.OneStop)
        self.port.setFlowControl(QSerialPort.NoFlowControl)
        ok = self.port.open(QSerialPort.ReadWrite)
        self.status_changed.emit("connected" if ok else "open failed")
        if not ok:
            self.error_reported.emit(self.port.errorString())
        return ok

    def close(self) -> None:
        if self.port.isOpen():
            self.port.close()
        self.status_changed.emit("disconnected")

    def write(self, data: bytes) -> None:
        if not self.port.isOpen():
            self.error_reported.emit("serial port is not open")
            return
        self.port.write(data)

    def _on_ready_read(self) -> None:
        self._buffer.extend(bytes(self.port.readAll()))
        while b"\n" in self._buffer:
            idx = self._buffer.index(b"\n")
            line = bytes(self._buffer[: idx + 1])
            del self._buffer[: idx + 1]
            self.line_received.emit(line)

    def _on_error(self, error: QSerialPort.SerialPortError) -> None:
        if error == QSerialPort.NoError:
            return
        self.error_reported.emit(self.port.errorString())
