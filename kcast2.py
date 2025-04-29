import socket
import struct
import sys
import time
import keyboard

# Unicast settings
UDP_IP = "192.168.1.177"
UDP_PORT = 5001  # Fixed port for unicast

# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

DEADZONE = 30

# Global toggle and flag variables
open_state = 1000
open1_state = 1000
cnt1 = 0
cnt2 = 0
flag3 = [0] * 15


def clamp(val):
    return max(1000, min(2000, val))


def read_keyboard_data():
    global open_state, open1_state, cnt1, cnt2, flag3
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
    leftMotor = clamp(1500 + forward * 500 + turn * 300)
    rightMotor = clamp(1500 + forward * 500 - turn * 300)
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
    leftX = 1500
    leftY = 1500
    base = 1500
    lifter = 1500
    lifter_mode = 1500
    speed_mode = 1500
    arm = 1500
    gripper = 1500
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
    cmd = "["
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
    channels = [
        ("2", leftX, 5),
        ("3", leftY, 5),
        ("4", base, 5),
        ("5", lifter, 0),
        ("6", gripper, 0),
        ("7", lifter_mode, 5),
        ("8", speed_mode, 5),
        ("9", arm, 0),
        ("B", up_down, 0),
        ("C", left_right, 0),
        ("D", open_state, 0),
        ("E", open1_state, 0)
    ]
    channel_index = 2
    for tag, value, thresh in channels:
        if abs(1500 - value) > thresh:
            flag3[channel_index] = 0
            cmd += tag + str(value) + ","
        elif flag3[channel_index] == 0:
            flag3[channel_index] = 1
            cmd += tag + str(1500) + ","
        channel_index += 1
    if cmd.endswith(","):
        cmd = cmd[:-1]
    cmd += "]"
    return "JOYSTICK:"+cmd


def transmitter_mode():
    print(f"Starting keyboard transmitter on {UDP_IP}:{UDP_PORT}")
    try:
        while True:
            data_str = read_keyboard_data()
            sock.sendto(data_str.encode('utf-8'), (UDP_IP, UDP_PORT))
            print(f"Sent: {data_str}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Exiting transmitter mode...")


def receiver_mode():
    print(f"Starting keyboard receiver on {UDP_IP}:{UDP_PORT}")
    sock.bind((UDP_IP, UDP_PORT))
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
        print("  Transmit mode: kcast2.py tx")
        print("  Receive mode: kcast2.py rx")
        sys.exit(1)
    if sys.argv[1] == "tx":
        transmitter_mode()
    else:
        receiver_mode()


if __name__ == "__main__":
    main()
