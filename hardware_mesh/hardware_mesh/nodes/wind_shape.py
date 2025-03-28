from asyncio import DatagramProtocol, get_event_loop, wait_for
from dataclasses import dataclass
from typing import Callable

from rclpy import init

from hardware_mesh_interfaces.srv import (
    WindShape as WindShapeSrv,
    WindShape_Request,
    WindShape_Response,
)

from hardware_mesh.lib.async_node import AsyncNode, spin_async

DEFAULT_ADDR = "192.168.88.40"
LOCAL_PORT = 60333
REMOTE_PORT = 60334
TIMEOUT_S = 3

BUFFER_SIZE = 8192
HEARTBEAT_INTERVAL_S = 0.04
MODULES = 56
MODULE_FANS = 18

SERVICE_NAME = "wind_shape"


@dataclass
class Message:
    code: str
    client_id: int
    payload: list[str]


class Protocol(DatagramProtocol):

    def __init__(self, addr: str, callback: Callable[[bytes], None] | None = None):
        self._addr = addr
        self._callback = callback
        self._transport = None

    def connection_made(self, transport) -> None:
        self._transport = transport

    def datagram_received(self, data: bytes, _) -> None:
        if self._callback:
            self._callback(data)

    # == Public API == #

    async def recv(self) -> bytes:
        assert self._transport

        if self._callback:
            raise RuntimeError("Can't recv() on two concurrent tasks")

        try:
            future = get_event_loop().create_future()
            self._callback = future.set_result
            return await future

        finally:
            self._callback = None

    def send(self, data: bytes) -> None:
        assert self._transport
        self._transport.sendto(data, (self._addr, REMOTE_PORT))


class WindShape(AsyncNode):

    def __init__(self, addr: str = DEFAULT_ADDR):
        super().__init__(WindShape.__name__)

        self._addr = addr
        self._client_id = 0

        self._in_control = False

        self.create_service(WindShapeSrv, SERVICE_NAME, self.service)
        self.create_timer(HEARTBEAT_INTERVAL_S, self._send_status)

    async def __aenter__(self):
        self._loop = get_event_loop()

        (self._transport, self._protocol) = await self._loop.create_datagram_endpoint(
            lambda: Protocol(self._addr),
            local_addr=("0.0.0.0", LOCAL_PORT),
        )

        self.get_logger().info("Handshaking...")
        self._send("REQUEST_CONNECTION", "no_message")

        try:
            msg = await wait_for(self._recv(), TIMEOUT_S)

        except TimeoutError:
            error = "Handshake failure. No response from WindShape"
            self.get_logger().error(error)
            raise RuntimeError(error)

        if msg.code != "ADDRESS":
            error = f"Handshake failure. Expected ADDRESS, got {msg.code}"
            self.get_logger().error(error)
            raise RuntimeError(error)

        self.get_logger().info(f"Received client ID {msg.client_id}")
        self._client_id = msg.client_id

    async def __aexit__(self, *_):
        assert self._transport
        self._transport.abort()

    # == Callbacks == #

    def on_message(self, msg: Message) -> None:
        if msg.code == "STATUS":
            [in_control, *_] = msg.payload
            in_control = in_control == "1"

            if in_control != self._in_control:
                self.get_logger().debug(f"Control change: {bool(in_control)}")
                self._in_control = in_control

    # == Service == #

    def service(
        self,
        request: WindShape_Request,
        response: WindShape_Response,
    ) -> WindShape_Response:
        self.get_logger().debug(
            f"Power: {request.enable_power} | Speed: {request.fan_speed}"
        )

        self._send_module(request.enable_power, request.fan_speed)
        return response

    # == Private API == #

    async def _recv(self) -> Message:
        assert self._protocol
        data = await self._protocol.recv()
        return parse_message(data)

    def _send_status(self) -> None:
        self._send("STATUS", "1;0")

    def _send_module(self, enable_power: bool, fan_speed: float) -> None:
        self._send("MODULE", generate_module_message(enable_power, fan_speed))

    def _send(self, code: str, payload: str) -> None:
        data = f"{code}@{self._client_id}:{payload}\0".encode()
        self._loop.call_soon_threadsafe(self._protocol.send, data)


def parse_message(data: bytes) -> Message:
    (code, rest) = data.split(b"@")
    (client_id, payload) = rest.split(b":")

    if payload[-1] == 0:
        payload = payload[:-1]

    return Message(code.decode(), int(client_id), payload.decode().split(";"))


def generate_module_message(enable_power: bool, fan_speed: float) -> str:
    fan_speed_str = str(int(max(0.0, min(100.0, fan_speed))))
    fans_str = ",".join(MODULE_FANS * [fan_speed_str])
    power_str = "1" if enable_power else "0"
    module_str = f"{fans_str};{power_str};0;0;0"
    return "|".join(map(lambda i: f"{i + 1};{module_str}", range(MODULES)))


def main(args: list[str] | None = None) -> None:
    init(args=args)
    spin_async(WindShape())


if __name__ == "__main__":
    main()
