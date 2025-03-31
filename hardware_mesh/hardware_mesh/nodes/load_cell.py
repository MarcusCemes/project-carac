from asyncio import get_event_loop
from dataclasses import dataclass
from enum import IntEnum
from httpx import AsyncClient
from struct import Struct, pack
import xml.etree.ElementTree as ET

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

        self._cpf = None
        self._cpt = None

        self._rdt_sequence = 0

        self._unpacker = Struct(">3I6i")

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

        await self._request_config()

        (self._transport, self._protocol) = await self._loop.create_datagram_endpoint(
            lambda: SimpleDatagramProtocol(self._addr, self._on_datagram),
            local_addr=("0.0.0.0", 0),
        )

        (_, port) = self._transport.get_extra_info("sockname")
        self.get_logger().info(f"Listening on port 0.0.0.0:{port}")

    async def __aexit__(self, *_) -> None:
        assert self._transport
        self._transport.abort()

    async def _request_config(self) -> None:
        self.get_logger().info("Requesting configuration...")

        async with AsyncClient() as client:
            response = await client.get(f"http://{self._addr[0]}/netftcalapi.xml")

        if response.status_code != 200:
            raise RuntimeError(f"Failed to get configuration: {response.status_code}")

        root = ET.fromstring(response.content)

        if (unit := LoadCell._get_field(root, "scalfu")) != "N":
            self.get_logger().warn(f"Not using Newtons! Unit is set to {unit}")

        if (torque_unit := LoadCell._get_field(root, "scaltu")) != "Nm":
            self.get_logger().warn(
                f"Not using Newton Metres! Unit is set to {torque_unit}"
            )

        cpf = LoadCell._get_field(root, "calcpf")
        cpt = LoadCell._get_field(root, "calcpt")

        if not cpf or not cpt:
            raise RuntimeError("Failed to get configuration: missing cfgcpf or cfgcpt")

        self._cpf = int(cpf)
        self._cpt = int(cpt)

    @staticmethod
    def _get_field(element: ET.Element, name: str) -> str | None:
        field = element.find(name)
        return field.text if field is not None else None

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
            message, command = (
                ("Requesting stream start", Command.StartBufferedStreaming)
                if request.enable
                else ("Requesting stream stop", Command.StopStreaming)
            )

            if request.enable:
                self._rdt_sequence = 0

            self.get_logger().info(message)
            data = pack(">HHI", COMMAND_HEADER, command, request.samples)

            self._loop.call_soon_threadsafe(self._protocol.send, data)
            response.success = True

        return response

    def _on_datagram(self, data: bytes) -> None:
        assert self._cpf and self._cpt

        if not self.executor:
            return

        response = Response(*self._unpacker.unpack(data))
        self._rdt_sequence += 1

        if response.rdt_sequence != self._rdt_sequence:
            self.get_logger().warn(
                f"Packet loss! Expected seq {self._rdt_sequence}, got {response.rdt_sequence}"
            )

            self._rdt_sequence = response.rdt_sequence

        event = msg.LoadCellData(
            fx=response.fx / self._cpf,
            fy=response.fy / self._cpf,
            fz=response.fz / self._cpf,
            tx=response.tx / self._cpt,
            ty=response.ty / self._cpt,
            tz=response.tz / self._cpt,
        )

        self._publisher.publish(event)


def main(args: list[str] | None = None):
    init(args=args)
    spin_async(LoadCell())


if __name__ == "__main__":
    main()
