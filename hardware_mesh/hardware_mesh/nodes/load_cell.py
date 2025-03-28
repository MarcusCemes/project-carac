from asyncio import get_event_loop
from dataclasses import dataclass
from enum import IntEnum
from struct import pack, unpack

from rclpy import init

from hardware_mesh_interfaces import msg, srv

from ..defs import DEFAULT_QOS, Services, Topics
from ..lib.async_node import AsyncNode, spin_async
from ..lib.async_utils import SimpleDatagramProtocol

DEFAULT_IP = "192.168.1.1"
DEFAULT_PORT = 49152

COMMAND_HEADER = 0x1234


class Command(IntEnum):
    StopStreaming = 0x0
    StartRealtimeStreaming = 0x2
    StartBufferedStreaming = 0x3
    SetSoftwareBias = 0x42


@dataclass
class Response:
    rdt_sequence: int
    ft_sequence: int
    status: int

    fx: int
    fy: int
    fz: int
    tx: int
    ty: int
    tz: int


class LoadCell(AsyncNode):

    def __init__(self, host: str = DEFAULT_IP, port: int = DEFAULT_PORT):
        super().__init__(LoadCell.__name__)

        self._addr = (host, port)
        self._protocol = None
        self._transport = None

        self._rdt_sequence = 0

        self.create_service(
            srv.LoadCellSetBias, Services.LoadCellSetBias, self._handle_set_bias
        )

        self.create_service(
            srv.LoadCellStream, Services.LoadCellStream, self._handle_stream
        )

        self._publisher = self.create_publisher(
            msg.LoadCellData, Topics.LoadCellData, DEFAULT_QOS
        )

    async def __aenter__(self) -> None:
        self._loop = get_event_loop()

        (self._transport, self._protocol) = await self._loop.create_datagram_endpoint(
            lambda: SimpleDatagramProtocol(self._addr, self._on_datagram),
        )

    async def __aexit__(self, *_) -> None:
        assert self._transport
        self._transport.abort()

    # == Callbacks == #

    def _handle_set_bias(
        self,
        _: srv.LoadCellSetBias_Request,
        response: srv.LoadCellSetBias_Response,
    ) -> srv.LoadCellSetBias_Response:
        if self._protocol:
            self.get_logger().info("Setting software bias")

            data = pack(">HHI", COMMAND_HEADER, Command.SetSoftwareBias, 0)

            self._loop.call_soon_threadsafe(self._protocol.send, data)
            response.success = True

        return response

    def _handle_stream(
        self,
        request: srv.LoadCellStream_Request,
        response: srv.LoadCellStream_Response,
    ) -> srv.LoadCellStream_Response:
        if self._protocol:
            self.get_logger().info(
                "Starting buffered streaming"
                if request.enable
                else "Stopping streaming"
            )

            data = pack(
                ">HHI", COMMAND_HEADER, Command.StartRealtimeStreaming, request.samples
            )

            self._loop.call_soon_threadsafe(self._protocol.send, data)
            response.success = True

        return response

    def _on_datagram(self, data: bytes) -> None:
        assert self.executor

        response = Response(*unpack(">3I6i", data))

        self._rdt_sequence += 1

        if response.rdt_sequence != self._rdt_sequence:
            self.get_logger().warn("Packet loss detected!")
            self._rdt_sequence = response.rdt_sequence

        event = msg.LoadCellData(
            fx=response.fx,
            fy=response.fy,
            fz=response.fz,
            tx=response.tx,
            ty=response.ty,
            tz=response.tz,
        )

        self._publisher.publish(event)


def main(args: list[str] | None = None):
    init(args=args)
    spin_async(LoadCell())


if __name__ == "__main__":
    main()
