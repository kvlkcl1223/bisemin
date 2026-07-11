from __future__ import annotations

from dataclasses import dataclass


class ProtocolError(ValueError):
    pass


@dataclass(frozen=True)
class Frame:
    frame_type: str
    fields: dict[str, str]
    raw: str = ""


def crc16_ccitt(payload: str) -> int:
    crc = 0xFFFF
    for byte in payload.encode("ascii"):
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def encode_frame(frame_type: str, **fields: object) -> bytes:
    parts = [frame_type]
    for key, value in fields.items():
        if value is None:
            continue
        parts.append(f"{key}={value}")
    payload = ",".join(parts)
    crc = crc16_ccitt(payload)
    return f"${payload}*{crc:04X}\r\n".encode("ascii")


def decode_frame(line: bytes | str) -> Frame:
    if isinstance(line, bytes):
        text = line.decode("ascii", errors="strict").strip()
    else:
        text = line.strip()

    if not text.startswith("$"):
        raise ProtocolError("missing frame start")
    if "*" not in text:
        raise ProtocolError("missing crc separator")

    body, crc_text = text[1:].rsplit("*", 1)
    if len(crc_text) != 4:
        raise ProtocolError("bad crc length")

    expected = int(crc_text, 16)
    actual = crc16_ccitt(body)
    if expected != actual:
        raise ProtocolError(f"crc mismatch expected={expected:04X} actual={actual:04X}")

    parts = body.split(",")
    if not parts or not parts[0]:
        raise ProtocolError("empty frame type")

    fields: dict[str, str] = {}
    for item in parts[1:]:
        if "=" not in item:
            continue
        key, value = item.split("=", 1)
        fields[key] = value

    return Frame(parts[0], fields, text)
