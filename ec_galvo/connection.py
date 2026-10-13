"""USB connection to the JCZ board, over pyusb + libusb.

Trimmed from MeerK40t meerk40t/balormk/usb_connection.py (MIT). Differences:
errors raise instead of silently reconnecting mid-job, there is no translated
UI channel, and the log never sees packet contents.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Protocol

from .protocol import PACKET_LIST, PACKET_SINGLE

USB_VENDOR = 0x9588
USB_PRODUCT = 0x9899

WRITE_ENDPOINT = 0x02
READ_ENDPOINT = 0x88


class Connection(Protocol):
    def open(self) -> None: ...
    def close(self) -> None: ...
    def is_open(self) -> bool: ...
    def write(self, packet: bytes) -> None: ...
    def read(self) -> bytes: ...


def _check_packet(packet: bytes) -> None:
    if len(packet) not in (PACKET_SINGLE, PACKET_LIST):
        raise ValueError(f"bad packet length {len(packet)}")


class USBConnection:
    def __init__(
        self,
        vendor: int = USB_VENDOR,
        product: int = USB_PRODUCT,
        index: int = 0,
        timeout_ms: int = 100,
        log: Callable[[str], None] | None = None,
    ):
        self.vendor = vendor
        self.product = product
        self.index = index
        self.timeout_ms = timeout_ms
        self._log = log or (lambda msg: None)
        self._device = None
        self._interface = None

    def open(self) -> None:
        import usb.core
        import usb.util

        devices = list(usb.core.find(idVendor=self.vendor, idProduct=self.product, find_all=True))
        if not devices:
            raise ConnectionRefusedError(
                f"no USB device {self.vendor:04x}:{self.product:04x} found"
            )
        try:
            device = devices[self.index]
        except IndexError:
            raise ConnectionRefusedError(
                f"{len(devices)} board(s) found, index {self.index} requested"
            ) from None
        try:
            device.set_configuration()
        except usb.core.USBError as e:
            # Already configured is fine; anything else shows up at claim time.
            self._log(f"set_configuration: {e}")
        interface = device.get_active_configuration()[(0, 0)]
        try:
            if device.is_kernel_driver_active(interface.bInterfaceNumber):
                device.detach_kernel_driver(interface.bInterfaceNumber)
        except NotImplementedError:
            pass  # macOS and Windows backends don't support this
        usb.util.claim_interface(device, interface)
        self._device = device
        self._interface = interface
        self._log("USB connected")

    def close(self) -> None:
        import usb.core
        import usb.util

        if self._device is None:
            return
        device = self._device
        try:
            usb.util.release_interface(device, self._interface)
        except usb.core.USBError as e:
            self._log(f"close: {e}")
        finally:
            # Even if releasing failed: free the handle so the board can be opened again.
            try:
                usb.util.dispose_resources(device)
            except usb.core.USBError as e:
                self._log(f"close: {e}")
            finally:
                self._device = None
                self._interface = None
                self._log("USB disconnected")

    def is_open(self) -> bool:
        return self._device is not None

    def write(self, packet: bytes) -> None:
        import usb.core

        _check_packet(packet)
        if self._device is None:
            raise ConnectionError("not connected")
        try:
            sent = self._device.write(WRITE_ENDPOINT, packet, self.timeout_ms)
        except usb.core.USBError as e:
            raise ConnectionError(f"USB write failed: {e}") from e
        if sent != len(packet):
            raise ConnectionError(f"USB write sent {sent} of {len(packet)} bytes")

    def read(self) -> bytes:
        import usb.core

        if self._device is None:
            raise ConnectionError("not connected")
        try:
            return bytes(self._device.read(READ_ENDPOINT, 8, self.timeout_ms))
        except usb.core.USBError as e:
            raise ConnectionError(f"USB read failed: {e}") from e
