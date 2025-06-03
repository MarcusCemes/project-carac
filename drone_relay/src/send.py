from socket import AF_INET, SOCK_DGRAM, socket
from struct import Struct
from time import sleep

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

relay = socket(AF_INET, SOCK_DGRAM)
relay.connect(("127.0.0.1", PORT))


def main():
    send_action([-1, 0, 0, 0, 0])
    sleep(0.5)

    send_action([-1, -1, -1, 0, 0])
    sleep(0.2)

    send_action([-1, 1, 1, 0, 0])
    sleep(0.2)

    for i in range(10):
        k = -1 + i * (2 / 10)
        send_action([-1, k, k, 0, 0])
        sleep(0.1)

    send_action([-1, 0, 0, -1, -1])
    sleep(0.25)

    send_action([-1, 0, 0, 1, 1])
    sleep(0.25)

    send_action([-1, 0, 0, 0, 0])


def send_action(action: list[float]):
    print(f"Sending action: {action}")

    payload = decoder.pack(*action)
    data = MAGIC_NUMBER.to_bytes(1, "big") + payload

    relay.send(data)


if __name__ == "__main__":
    main()
