"""Wired (USB-C) HID protocol for the AULA F75 Max (0c45:800a).

Protocol from mohin7/aula-f75max Docs/Hardware-Protocol-Notes.md, thibaultmiclo/aula-driver,
and VitalyArt/Aula-F75-Max-Driver.
"""

import time
from datetime import datetime

import hid

VID, PID = 0x0C45, 0x800A
CMD_INTERFACE = 3
PACKET = 64
DELAY = 0.035

CUSTOM_MODE = 0x80
TABLE_SIZE = 0x240  # 144 four-byte slots (per-key colour or remap), sent as 9 pages
BLOCK_COLORS, BLOCK_REMAP_BASE, BLOCK_REMAP_FN, BLOCK_SETTINGS = 0x23, 0x11, 0x27, 0x17
MACRO_BUFFER_MAX = 0xE00

DISPLAY_INTERFACE = 2
SCREEN_SIZE = 128
SCREEN_HEADER = 256  # [0] frame count, [1 + i] frame delay in 2 ms units
SCREEN_FRAME_BYTES = SCREEN_SIZE * SCREEN_SIZE * 2  # RGB565 little-endian
SCREEN_MAX_FRAMES = 255  # gif_maxframes in AULA's rgb-keyboard.xml; past it the firmware overwrites flash
SCREEN_CHUNK = 4096
SCREEN_SLOT = 1


def packet(*head: int, fields: dict[int, int] | None = None) -> bytes:
    buf = bytearray(PACKET)
    buf[: len(head)] = bytes(head)

    for i, v in (fields or {}).items():
        buf[i] = v

    return bytes(buf)


class KeyboardNotFound(RuntimeError):
    pass


def find_path(interface_number: int) -> bytes:
    path = next(
        (d["path"] for d in hid.enumerate(VID, PID)
         if d.get("interface_number") == interface_number),
        None,
    )

    if path is None:
        raise KeyboardNotFound("AULA F75 Max not found on USB; connect the cable and switch to wired mode.")

    return path


def screen_chunks(frame_count: int) -> int:
    return -(-(SCREEN_HEADER + frame_count * SCREEN_FRAME_BYTES) // SCREEN_CHUNK)


class Keyboard:
    def __init__(self):
        self.dev = hid.device()
        self.dev.open_path(find_path(CMD_INTERFACE))

    def close(self):
        self.dev.close()

    def send(self, data: bytes):
        time.sleep(DELAY)

        if self.dev.send_feature_report(b"\x00" + data) < 0:
            raise IOError(f"SET_REPORT failed: {self.dev.error()}")

    def read(self) -> bytes:
        time.sleep(DELAY)
        reply = bytes(self.dev.get_feature_report(0, PACKET + 1))

        # hidapi prefixes the report id on some platforms
        return reply[1:] if len(reply) == PACKET + 1 else reply

    def send_ack(self, data: bytes):
        self.send(data)
        reply = self.read()

        if len(reply) < 4 or reply[3] != 0x01:
            raise IOError(f"no ack for {data[:2].hex(' ')}: {reply[:8].hex(' ')}")

    def lighting(self, mode: int, rgb: tuple[int, int, int], brightness: int, speed: int,
                 direction: int, rainbow: bool):
        payload = bytearray(PACKET)
        payload[0] = mode

        if mode:
            payload[1:4] = bytes(rgb)
            payload[8] = int(rainbow)
            payload[9] = brightness
            payload[10] = speed
            payload[11] = direction

        payload[14:16] = b"\xaa\x55"

        self.send_ack(packet(0x04, 0x18))
        self.send_ack(packet(0x04, 0x13, fields={8: 0x01}))
        self.send(bytes(payload))
        self.send_ack(packet(0x04, 0x02))
        self.send(packet(0x04, 0xF0))

    def clock(self, now: datetime):
        data = bytearray(PACKET)
        data[0:3] = b"\x00\x01\x5a"
        data[3:9] = bytes([now.year - 2000, now.month, now.day, now.hour, now.minute, now.second])
        data[10] = (now.weekday() + 1) % 7  # firmware: 0 = Sunday
        data[62:64] = b"\xaa\x55"

        self.send_ack(packet(0x04, 0x18))
        self.send_ack(packet(0x04, 0x28, fields={8: 0x01}))
        self.send(bytes(data))
        self.read()  # echoes the data, no ack flag
        self.send_ack(packet(0x04, 0x02))

    def settings(self, report: bytes):
        """Write key locks, Fn switch, sleep timer, and key response together; see settings.Settings.encode."""
        self.send_ack(packet(0x04, 0x18))
        self.send_ack(packet(0x04, BLOCK_SETTINGS, fields={2: 0x01, 8: 0x01}))
        self.send(report)
        self.send_ack(packet(0x04, 0x02))

    def per_key(self, colors: dict[int, tuple[int, int, int]], brightness: int):
        self.lighting(CUSTOM_MODE, (0, 0, 0), brightness, 0, 0, False)

        table = bytearray(TABLE_SIZE)

        for idx, rgb in colors.items():
            table[idx * 4 : idx * 4 + 4] = bytes((idx, *rgb))

        table[-2:] = b"\xaa\x55"
        self.write_table(BLOCK_COLORS, bytes(table))

    def write_table(self, block: int, table: bytes):
        self.send_ack(packet(0x04, 0x18))
        self.send_ack(packet(0x04, block, fields={8: TABLE_SIZE // PACKET}))

        for offset in range(0, TABLE_SIZE, PACKET):
            self.send(table[offset : offset + PACKET])

        self.read()  # last page is read back before apply
        self.send_ack(packet(0x04, 0x02))
        self.send(packet(0x04, 0xF0))
        self.read()

    def keymap(self, macros: bytes, base: bytes, fn: bytes):
        """Replace all macros, then both remap layers; see keymap.Keymap.encode."""
        if len(macros) % PACKET or len(macros) > MACRO_BUFFER_MAX:
            raise ValueError(f"invalid macro buffer: {len(macros)} bytes")

        # no 04 18 begin or 04 F0 finalize for macros
        self.send_ack(packet(0x04, 0x19))
        self.send_ack(packet(0x04, 0x15, fields={8: len(macros) // PACKET}))

        for offset in range(0, len(macros), PACKET):
            self.send(macros[offset : offset + PACKET])

        self.read()
        self.send_ack(packet(0x04, 0x02))
        self.write_table(BLOCK_REMAP_BASE, base)
        self.write_table(BLOCK_REMAP_FN, fn)

    def upload_screen(self, stream: bytes, progress=None) -> int:
        """Write an encoded screen stream; returns how many chunks the keyboard acknowledged."""
        frames = stream[0] if stream else 0

        # last check before bytes reach the keyboard's flash
        if not 1 <= frames <= SCREEN_MAX_FRAMES or len(stream) != screen_chunks(frames) * SCREEN_CHUNK:
            raise ValueError(f"invalid screen stream: {frames} frames in {len(stream)} bytes")

        chunks = len(stream) // SCREEN_CHUNK
        display = hid.device()
        display.open_path(find_path(DISPLAY_INTERFACE))

        try:
            self.send_ack(packet(0x04, 0x18))
            self.send_ack(packet(0x04, 0x72, fields={2: SCREEN_SLOT, 8: chunks & 0xFF, 9: chunks >> 8}))
            display.read(512, 150)

            acked = 0

            for i in range(chunks):
                if display.write(b"\x00" + stream[i * SCREEN_CHUNK : (i + 1) * SCREEN_CHUNK]) < 0:
                    raise IOError(f"screen chunk {i + 1}/{chunks} failed: {display.error()}")

                if display.read(512, 400):
                    acked += 1

                if progress:
                    progress(i + 1, chunks)

            self.send_ack(packet(0x04, 0x02))
        finally:
            display.close()

        return acked

