"""Recording connection: a stand-in board that keeps every packet in memory.

Used by the simulator and by golden-file tests. Nothing is written to disk;
callers decide what (test-vector-only) output to save.
"""

from __future__ import annotations

import struct
from collections import deque
from dataclasses import dataclass, field

from .connection import _check_packet
from .protocol import (
    PACKET_LIST,
    READY,
    GetSerialNo,
    list_command_lookup,
    listEndOfList,
    single_command_lookup,
)


@dataclass(frozen=True)
class Command:
    opcode: int
    params: tuple[int, int, int, int, int]
    in_list: bool

    @property
    def name(self) -> str:
        table = list_command_lookup if self.in_list else single_command_lookup
        return table.get(self.opcode, f"unknown_{self.opcode:04x}")

    def __str__(self) -> str:
        p = list(self.params)
        while p and p[-1] == 0:
            p.pop()
        return " ".join([self.name, *(str(v) for v in p)])


@dataclass
class RecordingConnection:
    serial: bytes = b"RECORDER"
    status_word: int = READY  # idle and ready to take a list
    packets: list[bytes] = field(default_factory=list)
    replies: deque[bytes] = field(default_factory=deque)  # queued replies override defaults
    _open: bool = False
    _last_opcode: int | None = None

    def open(self) -> None:
        self._open = True

    def close(self) -> None:
        self._open = False

    def is_open(self) -> bool:
        return self._open

    def write(self, packet: bytes) -> None:
        _check_packet(packet)
        if not self._open:
            raise ConnectionError("not connected")
        packet = bytes(packet)
        self.packets.append(packet)
        if len(packet) != PACKET_LIST:
            self._last_opcode = struct.unpack_from("<H", packet)[0]

    def read(self) -> bytes:
        if not self._open:
            raise ConnectionError("not connected")
        if self.replies:
            return self.replies.popleft()
        if self._last_opcode == GetSerialNo:
            return self.serial[:8].ljust(8, b"\0")
        return struct.pack("<4H", 0, 0, 0, self.status_word)

    def commands(self) -> list[Command]:
        """Every command sent, in order. Trailing list padding is dropped."""
        out = []
        for packet in self.packets:
            in_list = len(packet) == PACKET_LIST
            entries = [struct.unpack_from("<6H", packet, i) for i in range(0, len(packet), 12)]
            if in_list:
                pad = (listEndOfList, 0, 0, 0, 0, 0)
                while len(entries) > 1 and entries[-1] == pad and entries[-2] == pad:
                    entries.pop()
            out += [Command(e[0], tuple(e[1:]), in_list) for e in entries]
        return out

    def log_lines(self) -> list[str]:
        """Readable command log for golden files. Collapses runs of identical lines."""
        lines: list[str] = []
        prev, run = None, 0
        for cmd in self.commands():
            s = ("L " if cmd.in_list else "S ") + str(cmd)
            if s == prev:
                run += 1
                continue
            if run:
                lines.append(f"  ... x{run} more")
            lines.append(s)
            prev, run = s, 0
        if run:
            lines.append(f"  ... x{run} more")
        return lines
