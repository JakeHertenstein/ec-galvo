"""USBConnection against a stand-in pyusb device (no board needed)."""

from collections import deque

import pytest

usb_core = pytest.importorskip("usb.core")
usb_util = pytest.importorskip("usb.util")

from ec_galvo import (  # noqa: E402 - after the importorskip
    CorrectionTable,
    Field,
    GalvoController,
    USBConnection,
)
from ec_galvo.connection import READ_ENDPOINT, WRITE_ENDPOINT  # noqa: E402
from ec_galvo.protocol import PACKET_LIST, READY, GetSerialNo  # noqa: E402


class FakeInterface:
    bInterfaceNumber = 0


class FakeDevice:
    """Answers like the board: 8 bytes per single command, nothing per list packet."""

    def __init__(self, serial=b"FAKEBRD1", kernel_driver=True):
        self.serial = serial
        self.kernel_driver = kernel_driver
        self.detached = []
        self.written = []
        self.short_by = 0  # report this many bytes fewer than were written
        self.fail_read = False
        self.replies = deque()

    def set_configuration(self):
        raise usb_core.USBError("already configured")  # common, and fine

    def get_active_configuration(self):
        return {(0, 0): FakeInterface()}

    def is_kernel_driver_active(self, n):
        return self.kernel_driver

    def detach_kernel_driver(self, n):
        self.detached.append(n)

    def write(self, endpoint, data, timeout):
        assert endpoint == WRITE_ENDPOINT
        self.written.append(bytes(data))
        if len(data) != PACKET_LIST:
            opcode = int.from_bytes(bytes(data[:2]), "little")
            reply = self.serial if opcode == GetSerialNo else bytes(6) + READY.to_bytes(2, "little")
            self.replies.append(reply)
        return len(data) - self.short_by

    def read(self, endpoint, size, timeout):
        assert endpoint == READ_ENDPOINT and size == 8
        if self.fail_read:
            raise usb_core.USBError("timeout")
        return bytearray(self.replies.popleft())


@pytest.fixture
def usb(monkeypatch):
    """Patch pyusb's lookups; returns the device list and a record of util calls."""
    state = {"devices": [FakeDevice()], "calls": [], "fail": set()}

    def find(idVendor, idProduct, find_all):
        assert (idVendor, idProduct, find_all) == (0x9588, 0x9899, True)
        return iter(state["devices"])

    def util(name):
        def call(*args):
            state["calls"].append(name)
            if name in state["fail"]:
                raise usb_core.USBError(f"{name} failed")

        return call

    monkeypatch.setattr(usb_core, "find", find)
    for name in ("claim_interface", "release_interface", "dispose_resources"):
        monkeypatch.setattr(usb_util, name, util(name))
    return state


def test_open_claims_the_board_and_detaches_the_kernel_driver(usb):
    conn = USBConnection()
    conn.open()
    assert conn.is_open()
    assert usb["calls"] == ["claim_interface"]
    assert usb["devices"][0].detached == [0]


def test_no_board_or_wrong_index_is_refused(usb):
    usb["devices"] = []
    with pytest.raises(ConnectionRefusedError, match="no USB device"):
        USBConnection().open()
    usb["devices"] = [FakeDevice()]
    with pytest.raises(ConnectionRefusedError, match="1 board"):
        USBConnection(index=1).open()


def test_write_checks_length_connection_and_bytes_sent(usb):
    conn = USBConnection()
    with pytest.raises(ConnectionError, match="not connected"):
        conn.write(bytes(12))
    conn.open()
    with pytest.raises(ValueError, match="bad packet length"):
        conn.write(bytes(13))
    device = usb["devices"][0]
    device.short_by = 4
    with pytest.raises(ConnectionError, match="sent 8 of 12 bytes"):
        conn.write(bytes(12))
    device.short_by = 0
    conn.write(bytes(PACKET_LIST))
    assert device.written[-1] == bytes(PACKET_LIST)


def test_usb_errors_become_connection_errors(usb):
    conn = USBConnection()
    conn.open()
    device = usb["devices"][0]

    def broken_write(*args):
        raise usb_core.USBError("pipe error")

    device.write = broken_write
    with pytest.raises(ConnectionError, match="USB write failed"):
        conn.write(bytes(12))
    device.fail_read = True
    with pytest.raises(ConnectionError, match="USB read failed"):
        conn.read()


def test_close_frees_the_device_even_if_releasing_fails(usb):
    logs = []
    conn = USBConnection(log=logs.append)
    conn.open()
    usb["fail"].add("release_interface")
    conn.close()
    assert usb["calls"] == ["claim_interface", "release_interface", "dispose_resources"]
    assert not conn.is_open()
    assert "USB disconnected" in logs
    conn.close()  # closing twice is harmless
    assert usb["calls"].count("dispose_resources") == 1


def test_controller_connects_through_usb(usb):
    conn = USBConnection()
    c = GalvoController(
        conn, Field(150.0), CorrectionTable.no_correction(), expected_serial=b"FAKEBRD1"
    )
    c.connect()
    assert c.serial_number == b"FAKEBRD1"
    c.program_mode()
    c.jump_mm(10, 10)
    c.rapid_mode()
    c.disconnect()
    assert not conn.is_open()
    assert usb["calls"][-2:] == ["release_interface", "dispose_resources"]
