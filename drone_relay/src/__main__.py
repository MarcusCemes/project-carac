from asyncio import (
    DatagramProtocol,
    DatagramTransport,
    Event,
    run,
    get_event_loop,
)
from struct import Struct
from typing import Any


# == Definitions == #

PORT = 2380

DRONE_IP = "192.168.194.234"
DRONE_PORT = 2390

ENCODING_RESOLUTION = 3
RANGE = (-1, 1)
MAPPED_RANGE = (0, 999)

BUFFER_SIZE = 1024
MAGIC_NUMBER = 0xDE


# == Functions == #

decoder = Struct("!5f")


class ReceiverProtocol(DatagramProtocol):

    # == Protocol == #

    def connection_made(self, transport: DatagramTransport) -> None:
        self._transport = transport

    def datagram_received(self, data: bytes, addr: tuple[str | Any, int]) -> None:
        if data[0] != MAGIC_NUMBER:
            print("Invalid message received")
            return

        values = list(decoder.unpack(data[2:]))
        values[1] = -values[1]
        msg = b"".join(map(encode_value, values))

        self._send(msg)
        self._echo_data(data, addr)

    # == Private == #

    def _send(self, data: bytes) -> None:
        self._transport.sendto(data, (DRONE_IP, DRONE_PORT))

    def _echo_data(self, data: bytes, addr: tuple[str | Any, int]) -> None:
        self._transport.sendto(data, addr)


async def main():
    loop = get_event_loop()

    await loop.create_datagram_endpoint(
        ReceiverProtocol,
        local_addr=("0.0.0.0", PORT),
    )

    print(f"Relay active: 0.0.0.0:{PORT} -> {DRONE_IP}:{DRONE_PORT}")
    await Event().wait()


def encode_value(value: float) -> bytes:
    """Encodes a floating-point value to three ASCII characters."""
    value = round(map_range(value, RANGE, MAPPED_RANGE))
    return bytes(f"{value:0{ENCODING_RESOLUTION}}", "ascii")


def map_range(value: float, in_range: tuple, out_range: tuple) -> float:
    """Maps a value from one range to another."""
    (min_in, max_in) = in_range
    (min_out, max_out) = out_range

    value = (value - min_in) / (max_in - min_in) * (max_out - min_out) + min_out

    return min(max(value, min_out), max_out)


if __name__ == "__main__":
    try:
        run(main())

    except KeyboardInterrupt:
        print("Relay stopped by user.")
