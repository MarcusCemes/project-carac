from asyncio import DatagramProtocol, get_event_loop
from typing import Callable


class SimpleDatagramProtocol(DatagramProtocol):
    def __init__(
        self, addr: tuple[str, int], callback: Callable[[bytes], None] | None = None
    ):
        self._addr = addr
        self._callback = callback
        self._transport = None

    def connection_made(self, transport) -> None:
        self._transport = transport

    def datagram_received(self, data: bytes, _):
        if self._callback:
            self._callback(data)

    # == Public == #

    def set_callback(self, callback: Callable[[bytes], None]) -> None:
        assert not self._callback, "Callback already set"
        self._callback = callback

    def clear_callback(self) -> None:
        self._callback = None

    async def recv(self) -> bytes:
        assert not self._callback

        future = get_event_loop().create_future()
        self._callback = future.set_result

        try:
            return await future

        finally:
            self._callback = None

    def send(self, data: bytes) -> None:
        assert self._transport

        self._transport.sendto(data, self._addr)
