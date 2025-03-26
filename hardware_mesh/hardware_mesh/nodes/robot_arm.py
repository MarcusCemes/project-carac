from dataclasses import dataclass
from socket import AF_INET, SOCK_STREAM, socket
from struct import pack, unpack, iter_unpack
from threading import Thread

from hardware_mesh.utils.math import euler_to_quaternion
from rclpy import init, spin
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
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

DEFAULT_ADDR = "192.168.100.254"
PORT = 20000

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


class RobotArm(Node):

    def __init__(self, addr: str = DEFAULT_ADDR):
        super().__init__(RobotArm.__name__)

        self._moving = False

        self._socket = socket(AF_INET, SOCK_STREAM)
        self._socket.connect((addr, PORT))

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

    # == Callbacks == #

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

    # == Public API == #

    def start(self) -> None:
        Thread(target=self._worker).start()

    # == Private API == #

    def _update_moving(self, moving: bool) -> None:
        if self._moving != moving:
            self._moving = moving
            self._moving_publisher.publish(Bool(data=moving))

    def _worker(self) -> None:
        while True:
            msg = recv_message(self._socket)

            if self.executor:
                self.executor.create_task(self.on_message, msg)

    def _send(self, instruction: int, payload: bytes) -> None:
        msg = pack("<BB", MAGIC_HEADER, instruction) + payload
        self._socket.send(msg)


def recv_message(socket: socket) -> Msg:
    [magic_header, instruction] = socket.recv(2)

    if magic_header != MAGIC_HEADER:
        raise ValueError("Invalid magic header")

    match instruction:
        case 0x01:
            data = socket.recv(72)
            return MsgReport(*iter_unpack("<6f", data))

        case 0x02:
            return MsgComplete()

        case 0x03:
            return MsgPowerOff()

        case 0xFE:
            return MsgAck()

        case 0xFF:
            return MsgError(*unpack("<BH", socket.recv(3)))

        case _:
            raise ValueError(f"Invalid instruction: {instruction}")


def main(args: list[str] | None = None) -> None:
    init(args=args)

    try:
        node = RobotArm()

        node.get_logger().info("Spawning network thread...")
        node.start()

        node.get_logger().info("Entering spin")
        spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    print("Shutting down...")


if __name__ == "__main__":
    main()
