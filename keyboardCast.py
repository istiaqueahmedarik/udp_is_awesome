import socket
import struct
import sys
import time
import keyboard

#!/usr/bin/env python

# Multicast settings
UDP_IP = "239.0.0.16"
UDP_PORT = 5476  # Default port, can be overridden with command line arg

# Create UDP multicast socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
# Join multicast group
mreq = struct.pack("4sl", socket.inet_aton(UDP_IP), socket.INADDR_ANY)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

DEADZONE = 30

# Global toggle and flag variables
open_state = 1000
open1_state = 1000
cnt1 = 0
cnt2 = 0
flag3 = [0] * 15

# Helper to clamp values between 1000 and 2000


def clamp(val):
    return max(1000, min(2000, val))


def read_keyboard_data():
    global open_state, open1_state, cnt1, cnt2, flag3

    # Drive control: use w,s for forward/backward and a,d for turning
    forward = 0
    turn = 0
    if keyboard.is_pressed('w'):
        forward = 1
    if keyboard.is_pressed('s'):
        forward = -1
    if keyboard.is_pressed('a'):
        turn = -1
    if keyboard.is_pressed('d'):
        turn = 1

    # Differential drive mixing
    leftMotor = clamp(1500 + forward * 500 + turn * 300)
    rightMotor = clamp(1500 + forward * 500 - turn * 300)

    # For separate movement channels (B and C)
    up_down = 1500
    if keyboard.is_pressed('w'):
        up_down = 1000
    elif keyboard.is_pressed('s'):
        up_down = 2000

    left_right = 1500
    if keyboard.is_pressed('a'):
        left_right = 2000
    elif keyboard.is_pressed('d'):
        left_right = 1000

    # Other channels default to 1500
    leftX = 1500
    leftY = 1500
    base = 1500
    lifter = 1500
    lifter_mode = 1500
    speed_mode = 1500
    arm = 1500
    gripper = 1500

    # Toggle keys for gripper channels
    # Toggle using 'p' for open_state and 'o' for open1_state;
    # 'h' toggles both simultaneously.
    if keyboard.is_pressed('p'):
        cnt1 += 1
        cnt1 %= 3
        if cnt1 == 0:
            open_state = 2000 if open_state == 1000 else 1000

    if keyboard.is_pressed('o'):
        cnt2 += 1
        cnt2 %= 3
        if cnt2 == 0:
            open1_state = 2000 if open1_state == 1000 else 1000

    if keyboard.is_pressed('h'):
        if open_state == 1000:
            open_state = 2000
            open1_state = 2000
        else:
            open_state = 1000
            open1_state = 1000

    # Build command string with channels:
    # "0": leftMotor, "1": rightMotor,
    # "2": leftX, "3": leftY, "4": base, "5": lifter,
    # "6": gripper, "7": lifter_mode, "8": speed_mode,
    # "9": arm, "B": up_down, "C": left_right,
    # "D": open_state, "E": open1_state
    cmd = "["

    # Check and add motor values only if they deviate significantly
    if abs(1500 - leftMotor) > DEADZONE:
        flag3[0] = 0
        cmd += "0" + str(leftMotor) + ","
    elif flag3[0] == 0:
        flag3[0] = 1
        cmd += "0" + str(1500) + ","

    if abs(1500 - rightMotor) > DEADZONE:
        flag3[1] = 0
        cmd += "1" + str(rightMotor) + ","
    elif flag3[1] == 0:
        flag3[1] = 1
        cmd += "1" + str(1500) + ","

    # For the other channels, send only on change (or once)
    channels = [
        ("2", leftX, 5),
        ("3", leftY, 5),
        ("4", base, 5),
        ("5", lifter, 0),  # Use direct comparison
        ("6", gripper, 0),
        ("7", lifter_mode, 5),
        ("8", speed_mode, 5),
        ("9", arm, 0),
        ("B", up_down, 0),
        ("C", left_right, 0),
        ("D", open_state, 0),
        ("E", open1_state, 0)
    ]

    # Process each channel in order
    channel_index = 2
    for tag, value, thresh in channels:
        if abs(1500 - value) > thresh:
            flag3[channel_index] = 0
            cmd += tag + str(value) + ","
        elif flag3[channel_index] == 0:
            flag3[channel_index] = 1
            cmd += tag + str(1500) + ","
        channel_index += 1

    # Remove trailing comma and close bracket properly
    if cmd.endswith(","):
        cmd = cmd[:-1]
    cmd += "]"
    return cmd


def transmitter_mode(port):
    print(f"Starting keyboard transmitter on multicast group {UDP_IP}:{port}")
    try:
        while True:
            data_str = read_keyboard_data()
            sock.sendto(data_str.encode('utf-8'), (UDP_IP, port))
            print(f"Sent: {data_str}")
            time.sleep(0.1)  # 10Hz update rate
    except KeyboardInterrupt:
        print("Exiting transmitter mode...")


def receiver_mode(port):
    print(f"Starting keyboard receiver on multicast group {UDP_IP}:{port}")
    sock.bind((UDP_IP, port))
    try:
        while True:
            data, addr = sock.recvfrom(1024)
            data_str = data.decode('utf-8')
            print(f"Received from {addr}: {data_str}")
    except KeyboardInterrupt:
        print("Exiting receiver mode...")


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ["tx", "rx"]:
        print("Usage:")
        print("  Transmit mode: keyboardCast.py tx [port]")
        print("  Receive mode: keyboardCast.py rx [port]")
        sys.exit(1)

    port = UDP_PORT
    if len(sys.argv) >= 3:
        port = int(sys.argv[2])

    if sys.argv[1] == "tx":
        transmitter_mode(port)
    else:
        receiver_mode(port)


if __name__ == "__main__":
    main()
