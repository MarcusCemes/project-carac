from asyncio import (
    DatagramProtocol,
    StreamReader,
    get_event_loop,
    open_connection,
    wait_for,
)
from dataclasses import dataclass
from struct import pack, unpack, iter_unpack
from typing import Callable

from hardware_mesh.utils.math import euler_to_quaternion
from rclpy import init
from std_msgs.msg import Bool

from hardware_mesh_interfaces.msg import RobotArmPosition
from hardware_mesh_interfaces.srv import (
    RobotArmConfig,
    RobotArmConfig_Request,
    RobotArmConfig_Response,
    RobotArmTool,
    RobotArmTool_Request,
    RobotArmTool_Response,
)

from hardware_mesh.lib.async_node import AsyncNode, spin_async

DEFAULT_ADDR = "192.168.100.254"
PORT = 20000
TIMEOUT_S = 3

BUFFER_SIZE = 8192
MAGIC_HEADER = 0x95
QOS_DEPTH = 10

CONFIG_SERVICE_NAME = "robot_arm_config"
TOOL_SERVICE_NAME = "robot_arm_tool"
MOVING_TOPIC_NAME = "robot_arm_moving"
POSE_TOPIC_NAME = "robot_arm_pose"

SixFloats = tuple[float, float, float, float, float, float]


@dataclass
class MsgReport:
    pose: SixFloats
    joints: SixFloats
    error: SixFloats


@dataclass
class MsgComplete:
    pass


@dataclass
class MsgPowerOff:
    pass


@dataclass
class MsgAck:
    pass


@dataclass
class MsgError:
    code: int
    argument: int


Msg = MsgReport | MsgComplete | MsgPowerOff | MsgAck | MsgError


@dataclass
class RobotState:
    moving: bool
    position: list[float]
    joints: list[float]
    error: list[float]


class Protocol(DatagramProtocol):
    def __init__(self, callback: Callable[[bytes], None], addr: str = DEFAULT_ADDR):
        self._addr = addr
        self._callback = callback

    def datagram_received(self, data: bytes, _) -> None:
        self._callback(data)


class RobotArm(AsyncNode):

    def __init__(self, addr: str = DEFAULT_ADDR):
        super().__init__(RobotArm.__name__)

        self._addr = addr
        self._moving = False

        self._pose_publisher = self.create_publisher(
            RobotArmPosition, POSE_TOPIC_NAME, QOS_DEPTH
        )

        self._moving_publisher = self.create_publisher(
            Bool, MOVING_TOPIC_NAME, QOS_DEPTH
        )

        self.create_service(
            RobotArmConfig,
            CONFIG_SERVICE_NAME,
            self.on_config,
        )

        self.create_service(
            RobotArmTool,
            TOOL_SERVICE_NAME,
            self.on_tool,
        )

    async def __aenter__(self):
        self._loop = get_event_loop()

        self.get_logger().info(f"Connecting to {self._addr}:{PORT}")

        try:
            (reader, self._writer) = await wait_for(
                open_connection(self._addr, PORT),
                TIMEOUT_S,
            )

        except TimeoutError:
            error = f"Connection to {self._addr}:{PORT} timed out"
            self.get_logger().error(error)
            raise RuntimeError(error)

        self._task = get_event_loop().create_task(self._receiver(reader))

    async def __aexit__(self, *_):
        assert self._writer
        self._writer.close()

    # == Callbacks == #

    def on_message_threadsafe(self, msg: Msg) -> None:
        assert self.executor
        self.executor.create_task(self.on_message, msg)

    def on_message(self, msg: Msg) -> None:
        if isinstance(msg, MsgReport):
            self._update_moving(True)

            orientation = euler_to_quaternion(msg.pose[3:])
            event = RobotArmPosition(position=msg.pose[:3], orientation=orientation)

            self._pose_publisher.publish(event)

        elif isinstance(msg, MsgComplete):
            self._update_moving(False)
            self._moving_publisher.publish(Bool(data=False))
            self.get_logger().debug("Motion complete")

        elif isinstance(msg, MsgPowerOff):
            self.get_logger().info("Shutdown signal received")

        elif isinstance(msg, MsgAck):
            self.get_logger().debug("ACK")

        elif isinstance(msg, MsgError):
            self.get_logger().error(f"Error: {msg.code} - {msg.argument}")

    def on_tool(
        self,
        request: RobotArmTool_Request,
        response: RobotArmTool_Response,
    ) -> RobotArmTool_Response:
        instruction = 0x02 if request.linear else 0x03
        payload = pack("<6f", *request.position, *request.orientation)

        self._send(instruction, payload)
        return response

    def on_config(
        self, request: RobotArmConfig_Request, response: RobotArmConfig_Response
    ) -> RobotArmConfig_Response:
        if not 0 < request.acceleration_scale <= 100:
            raise ValueError("Acceleration scale must be between 1 and 100")

        instruction = 0x05
        payload = pack(
            "<2f3B",
            request.translation_limit,
            request.rotation_limit,
            request.acceleration_scale,
            100,
            request.acceleration_scale,
        )

        self._send(instruction, payload)
        return response

    # == Background task == #

    async def _receiver(self, reader: StreamReader) -> None:
        while True:
            try:
                msg = await self._recv_message(reader)
                self.on_message_threadsafe(msg)

            except ValueError as e:
                self.get_logger().error(f"Error receiving message: {e}")
                break

    async def _recv_message(self, reader: StreamReader) -> Msg:
        [magic_header, instruction] = await reader.readexactly(2)

        if magic_header != MAGIC_HEADER:
            raise ValueError("Invalid magic header")

        match instruction:
            case 0x01:
                return MsgReport(*iter_unpack("<6f", await reader.readexactly(72)))

            case 0x02:
                return MsgComplete()

            case 0x03:
                return MsgPowerOff()

            case 0xFE:
                return MsgAck()

            case 0xFF:
                return MsgError(*unpack("<BH", await reader.readexactly(3)))

            case _:
                error = f"Invalid instruction: {instruction}"
                self.get_logger().error(error)
                raise ValueError(error)

    # == Private API == #

    def _update_moving(self, moving: bool) -> None:
        if self._moving != moving:
            self._moving = moving
            self._moving_publisher.publish(Bool(data=moving))

    def _send(self, instruction: int, payload: bytes) -> None:
        data = pack("<BB", MAGIC_HEADER, instruction) + payload
        self._loop.call_soon_threadsafe(self._writer.write, data)


def main(args: list[str] | None = None) -> None:
    init(args=args)
    spin_async(RobotArm())


if __name__ == "__main__":
    main()
