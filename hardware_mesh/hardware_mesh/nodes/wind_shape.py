from dataclasses import dataclass
from socket import AF_INET, SOCK_DGRAM, socket
from threading import Thread

from rclpy import init, spin
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from hardware_mesh_interfaces.srv import (
    WindShape as WindShapeSrv,
    WindShape_Request,
    WindShape_Response,
)

DEFAULT_ADDR = "192.168.88.40"
LOCAL_PORT = 60333
REMOTE_PORT = 60334

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


class WindShape(Node):

    def __init__(self, addr: str = DEFAULT_ADDR):
        super().__init__(WindShape.__name__)

        self._addr = addr
        self._client_id = 0

        self._socket = socket(AF_INET, SOCK_DGRAM)
        self._socket.bind(("0.0.0.0", LOCAL_PORT))

        self._in_control = False

        self.create_service(WindShapeSrv, SERVICE_NAME, self.service)
        self.create_timer(HEARTBEAT_INTERVAL_S, self._send_status)

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

    # == Public API == #

    def handshake(self) -> None:
        self._send("REQUEST_CONNECTION", "no_message")
        msg = self._recv()

        if msg.code != "ADDRESS":
            raise RuntimeError(f"Handshake failure: {msg.code}")

        self._client_id = msg.client_id

    def start(self) -> None:
        Thread(target=self._worker).start()

    # == Private API == #

    def _worker(self):
        while True:
            msg = self._recv()

            if self.executor:
                self.executor.create_task(self.on_message, msg)

    def _recv(self) -> Message:
        (data, _) = self._socket.recvfrom(BUFFER_SIZE)
        return parse_message(data)

    def _send(self, code: str, payload: str) -> None:
        msg = f"{code}@{self._client_id}:{payload}\0".encode()
        self._socket.sendto(msg, (self._addr, REMOTE_PORT))

    def _send_status(self) -> None:
        self._send("STATUS", "1;0")

    def _send_module(self, enable_power: bool, fan_speed: float) -> None:
        self._send("MODULE", generate_module_message(enable_power, fan_speed))


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

    try:
        node = WindShape()

        node.get_logger().info("Handshaking...")
        node.handshake()

        node.get_logger().info("Spawning network thread...")
        node.start()

        node.get_logger().info("Entering spin")
        spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
