import numpy as np
import socket


def compress(con):
    return con / 2 + 0.5


def call(con, UDP_IP="192.168.1.1", UDP_PORT=5005, sock=None):
    sock.sendto(bytes(str(con), "utf-8"), (UDP_IP, UDP_PORT))


# used for Arduino communication
def encode(con):
    con = np.array(con)
    # all the different commands (-1,1) are converted into a string with positive numbers (0,1) at a certain resolution
    res = 3
    con = np.round(compress(con), res)
    str_con = str()
    for i in range(len(con)):
        con_clipped = np.clip(con[i], 10**-res, 1 - 10**-res)
        str_con += str(con_clipped)[1:].replace(".", "").ljust(res, "0")
    return str_con


def cmd_wifi_callback(action, UDP_IP, UDP_Port, UDP_socket):
    # action: Input action snet to drone, in order of throttle, left sweep, right sweep, elevator, rudder
    act_sim_to_real = np.array(
        [1.0, -1.0, 1.0, 1.0, 1.0]
    )  # Ensure correct servo signal to drone from drone input
    com = encode(action * act_sim_to_real)
    call(com, UDP_IP, UDP_Port, UDP_socket)

    return com


if __name__ == "__main__":
    UDP_IP = "192.168.194.234"  # Drone IP
    UDP_PORT = 2390  # Drone listener Port

    UDP_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Internet  # UDP

    # Example use, Call cmd_wifi_callback (should be in Timer node with regular update intervals (i.e 50 Hz))
    action = np.array([0.0, 1.0, 1.0, 0, 0])  # Example action
    com = cmd_wifi_callback(action, UDP_IP, UDP_PORT, UDP_socket)
