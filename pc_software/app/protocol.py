from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import Iterable


SOF = b"\xA5\x5A"
EOF = b"\x0D\x0A"
PAYLOAD_MAX = 256
LEN_MIN = 3
LEN_MAX = 1 + 2 + PAYLOAD_MAX
FRAME_MIN = 2 + 2 + LEN_MIN + 2 + 2


class ProtocolError(ValueError):
    pass


class FrameType(IntEnum):
    CMD = 0x01
    ACK = 0x02
    NACK = 0x03
    STATE = 0x04
    DATA = 0x05
    EVENT = 0x06
    HELLO = 0x07
    HEARTBEAT = 0x08


TYPE_BY_NAME = {item.name: item for item in FrameType}


@dataclass(frozen=True)
class Frame:
    frame_type: str
    seq: int
    payload: str
    fields: dict[str, str]
    raw: bytes = b""
    frame_type_value: int = 0


def crc16_ccitt(data: bytes | str) -> int:
    if isinstance(data, str):
        data = data.encode("ascii")
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def _frame_type_value(frame_type: str | int | FrameType) -> int:
    if isinstance(frame_type, FrameType):
        return int(frame_type)
    if isinstance(frame_type, int):
        if frame_type < 0 or frame_type > 0xFF:
            raise ProtocolError("帧类型超出范围")
        return frame_type
    name = frame_type.upper()
    if name not in TYPE_BY_NAME:
        raise ProtocolError(f"未知帧类型：{frame_type}")
    return int(TYPE_BY_NAME[name])


def _frame_type_name(value: int) -> str:
    try:
        return FrameType(value).name
    except ValueError:
        return f"0x{value:02X}"


def encode_payload(**fields: object) -> str:
    parts: list[str] = []
    for key, value in fields.items():
        if value is None:
            continue
        text_key = str(key)
        text_value = str(value)
        if any(ch in text_key for ch in ",="):
            raise ProtocolError(f"载荷字段名无效：{text_key}")
        if any(ch in text_value for ch in ",\r\n"):
            raise ProtocolError(f"载荷字段值无效：{text_key}")
        parts.append(f"{text_key}={text_value}")
    payload = ",".join(parts)
    payload.encode("ascii")
    if len(payload.encode("ascii")) > PAYLOAD_MAX:
        raise ProtocolError("载荷过长")
    return payload


def parse_payload(payload: str) -> dict[str, str]:
    fields: dict[str, str] = {}
    if not payload:
        return fields
    for item in payload.split(","):
        if not item:
            continue
        if "=" not in item:
            fields[item] = ""
            continue
        key, value = item.split("=", 1)
        fields[key] = value
    return fields


def encode_frame(frame_type: str | int | FrameType, seq: int = 0, **fields: object) -> bytes:
    type_value = _frame_type_value(frame_type)
    if seq < 0 or seq > 0xFFFF:
        raise ProtocolError("序号超出范围")

    payload = encode_payload(**fields).encode("ascii")
    length = 1 + 2 + len(payload)
    if length > LEN_MAX:
        raise ProtocolError("帧过长")

    body = bytearray()
    body.extend(length.to_bytes(2, "little"))
    body.append(type_value)
    body.extend(seq.to_bytes(2, "little"))
    body.extend(payload)
    crc = crc16_ccitt(bytes(body))

    frame = bytearray(SOF)
    frame.extend(body)
    frame.extend(crc.to_bytes(2, "little"))
    frame.extend(EOF)
    return bytes(frame)


def decode_frame(raw: bytes | bytearray) -> Frame:
    data = bytes(raw)
    if len(data) < FRAME_MIN:
        raise ProtocolError("帧过短")
    if not data.startswith(SOF):
        raise ProtocolError("缺少帧头")
    if data[-2:] != EOF:
        raise ProtocolError("帧尾错误")

    length = int.from_bytes(data[2:4], "little")
    if length < LEN_MIN or length > LEN_MAX:
        raise ProtocolError(f"长度错误：{length}")

    expected_size = 2 + 2 + length + 2 + 2
    if len(data) != expected_size:
        raise ProtocolError(f"帧大小错误：收到 {len(data)}，应为 {expected_size}")

    crc_expected = int.from_bytes(data[4 + length : 4 + length + 2], "little")
    crc_actual = crc16_ccitt(data[2 : 4 + length])
    if crc_expected != crc_actual:
        raise ProtocolError(f"CRC 不匹配：收到={crc_expected:04X} 计算={crc_actual:04X}")

    type_value = data[4]
    seq = int.from_bytes(data[5:7], "little")
    payload_bytes = data[7 : 4 + length]
    try:
        payload = payload_bytes.decode("ascii")
    except UnicodeDecodeError as exc:
        raise ProtocolError("载荷不是 ASCII 编码") from exc

    return Frame(
        frame_type=_frame_type_name(type_value),
        frame_type_value=type_value,
        seq=seq,
        payload=payload,
        fields=parse_payload(payload),
        raw=data,
    )


def frame_to_text(frame: Frame) -> str:
    return f'类型={frame.frame_type}(0x{frame.frame_type_value:02X}), 序号={frame.seq}, 载荷="{frame.payload}"'


def bytes_to_hex(data: Iterable[int]) -> str:
    return " ".join(f"{byte:02X}" for byte in data)


class FrameParser:
    def __init__(self) -> None:
        self._buffer = bytearray()

    def clear(self) -> None:
        self._buffer.clear()

    def feed(self, data: bytes | bytearray) -> tuple[list[Frame], list[str]]:
        self._buffer.extend(data)
        frames: list[Frame] = []
        errors: list[str] = []

        while True:
            start = self._buffer.find(SOF)
            if start < 0:
                if self._buffer:
                    errors.append(f"丢弃帧头前 {len(self._buffer)} 个字节")
                    self._buffer.clear()
                break
            if start > 0:
                errors.append(f"丢弃帧头前 {start} 个字节")
                del self._buffer[:start]

            if len(self._buffer) < 4:
                break

            length = int.from_bytes(self._buffer[2:4], "little")
            if length < LEN_MIN or length > LEN_MAX:
                errors.append(f"长度错误：{length}")
                del self._buffer[0]
                continue

            frame_size = 2 + 2 + length + 2 + 2
            if len(self._buffer) < frame_size:
                break

            raw = bytes(self._buffer[:frame_size])
            del self._buffer[:frame_size]
            try:
                frames.append(decode_frame(raw))
            except ProtocolError as exc:
                errors.append(str(exc))

        if len(self._buffer) > FRAME_MIN + LEN_MAX + 32:
            errors.append("接收缓冲区溢出，已清空")
            self._buffer.clear()

        return frames, errors
