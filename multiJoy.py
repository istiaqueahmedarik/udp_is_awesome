#!/usr/bin/env python
import socket
import struct
import sys
import time
import pygame
import keyboard
import threading

# Multicast settings
UDP_IP = "239.0.0.18"
UDP_PORT = 5010  # Default port, can be overridden with command line arg

# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

# Constants
DEADZONE = 30

# Global variables for joystick state
open1 = 1000
open = 1000
cnt1 = 0
cnt2 = 0
flag1 = False
flag2 = False
flag3 = [0] * 15
l_state = 'on'


def joystick_to_motor_speed(x, y):
    # Normalize the joystick values
    x_norm = (x - 1500) / 500
    y_norm = (y - 1500) / 500

    # Calculate motor speeds
    left_speed_norm = y_norm + x_norm
    right_speed_norm = y_norm - x_norm

    # Rescale to motor speed range
    left_speed = 1500 + 500 * left_speed_norm
    right_speed = 1500 + 500 * right_speed_norm

    # Ensure the values are within the valid range
    left_speed = max(1000, min(2000, left_speed))
    right_speed = max(1000, min(2000, right_speed))

    return int(left_speed), int(right_speed)


# --- Modified get_joystick_value to handle extreme joystick ---
def get_joystick_value(joystick):
    if "extreme" in joystick.get_name().lower():
        return {
            "leftY": joystick.get_axis(1),
            "leftX": joystick.get_axis(0),
            "rightY": joystick.get_axis(3),
            "rightX": joystick.get_axis(2),
            "gripper_of": joystick.get_button(2),
            "gripper_on": joystick.get_button(3),
            "base": joystick.get_axis(4),
            "lifter_mode": joystick.get_axis(5),
            "speed_mode": joystick.get_button(1),
            "lifter": joystick.get_axis(6),
            "arm": joystick.get_button(0),
            # new added field for extreme joystick
            "extreme_new": joystick.get_axis(7)
        }
    # If joystick name contains 'extreme', process separately: return empty.
    return {
        "leftY": joystick.get_axis(3),
        "leftX": joystick.get_axis(2),
        "rightY": joystick.get_axis(1),
        "rightX": joystick.get_axis(0),
        "gripper_of": joystick.get_button(2),
        "gripper_on": joystick.get_button(3),
        "base": joystick.get_axis(4),
        "lifter_mode": joystick.get_axis(5),
        "speed_mode": joystick.get_button(1),
        "lifter": joystick.get_axis(6),
        "arm": joystick.get_button(0)
    }


def get_arm_joystick_value(joystick):
    X = joystick.get_axis(0)
    Y = joystick.get_axis(1)
    base = -1*joystick.get_axis(2)
    gripper_on = joystick.get_button(0)
    gripper_of = joystick.get_button(1)
    ghurbe = 1500
    ghurbe = 2000 if joystick.get_button(2) == 1 else ghurbe
    ghurbe = 1000 if joystick.get_button(3) == 1 else ghurbe
    uthbe = 1500
    uthbe = 2000 if joystick.get_button(4) == 1 else uthbe
    uthbe = 1000 if joystick.get_button(5) == 1 else uthbe
    # lifter = joystick.get_axis(5)

    return {
        "X": X,
        "Y": Y,
        "gripper_of": gripper_of,
        "gripper_on": gripper_on,
        "base": base,
        "ghurbe": ghurbe,
        "uthbe": uthbe
    }


def read_joystick_data():
    global open, open1, cnt1, cnt2, flag3, l_state

    pygame.event.pump()
    joystick = None
    arm_joystick = None
    try:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
    except pygame.error:
        pass

    try:
        arm_joystick = pygame.joystick.Joystick(1)
        arm_joystick.init()
    except pygame.error:
        # Get joystick values using helper function.
        pass

    if (joystick is not None and arm_joystick is not None):
        if (joystick.get_name().lower() == "extreme"):
            tmp = joystick
            joystick = arm_joystick
            arm_joystick = tmp
    elif joystick is None:
        print('must be connected normal_joystick')
        return

    joystick_values = ""
    if joystick is not None:
        joystick_values = get_joystick_value(joystick)

    arm_joystick_values = {}
    if arm_joystick is not None:
        arm_joystick_values = get_arm_joystick_value(arm_joystick)

    # Replace direct calls with values from joystick_values.
    leftY = arm_joystick_values["Y"] if arm_joystick_values else 1500 if joystick_values == "" else joystick_values["leftY"]
    leftX = arm_joystick_values["X"] if arm_joystick_values else 1500 if joystick_values == "" else joystick_values["leftX"]
    rightY = 1500 if joystick_values == "" else joystick_values["rightY"]
    rightX = 1500 if joystick_values == "" else joystick_values["rightX"]
    gripper_of = 1500 if joystick_values == "" else joystick_values["gripper_of"]
    gripper_on = 1500 if joystick_values == "" else joystick_values["gripper_on"]
    base = 1500 if joystick_values == "" else joystick_values["base"]
    lifter_mode = 1500 if joystick_values == "" else joystick_values["lifter_mode"]
    speed_mode = 1500 if joystick_values == "" else joystick_values["speed_mode"]
    lifter = 1500 if joystick_values == "" else joystick_values["lifter"]
    arm = 1500 if joystick_values == "" else joystick_values["arm"]
    ghurbe = 1500
    uthbe = 1500
    if (arm_joystick is not None):
        base = arm_joystick_values['base']
        ghurbe = arm_joystick_values['ghurbe']
        uthbe = arm_joystick_values['uthbe']

    # Map from -1 to 1 to 1000 to 2000
    leftY = int((leftY + 1) * 500 + 1000)
    leftX = int((leftX + 1) * 500 + 1000)
    rightY = int((rightY + 1) * 500 + 1000)
    rightX = int((rightX + 1) * 500 + 1000)
    arm = int((arm + 1) * 500 + 1000)

    base = int((base + 1) * 500 + 1000)
    speed_mode = int((speed_mode + 1) * 500 + 1000)
    lifter = int((lifter + 1) * 500 + 1000)
    lifter_mode = int((lifter_mode + 1) * 500 + 1000)
    gripper_of = int((1 - gripper_of) * 500 + 1000)
    gripper_on = int(gripper_on * 500 + 1500)
    gripper = 1500
    if (arm_joystick is not None):
        gripper_of = arm_joystick_values["gripper_of"]
        gripper_on = arm_joystick_values["gripper_on"]
    if (gripper_on == 1 and gripper_of == 1):
        gripper = 2000
    elif (gripper_on == 1 and gripper_of == 0):
        gripper = 1000
    # if gripper_of != 1500:
    #     gripper = gripper_of

    # Keyboard controls
    h = keyboard.is_pressed('h')
    l = keyboard.is_pressed('l')
    w = keyboard.is_pressed('w')
    s = keyboard.is_pressed('s')
    a = keyboard.is_pressed('a')
    d = keyboard.is_pressed('d')

    # Handle 'p' key for toggle
    if keyboard.is_pressed('p'):
        o = True
        cnt1 += 1
        cnt1 %= 3
        if cnt1 == 0 and open == 1000:
            open = 2000
        elif cnt1 == 0 and open == 2000:
            open = 1000
    else:
        o = False

    # Handle 'o' key for toggle
    if keyboard.is_pressed('o'):
        o1 = True
        cnt2 += 1
        cnt2 %= 3
        if cnt2 == 0 and open1 == 1000:
            open1 = 2000
        elif cnt2 == 0 and open1 == 2000:
            open1 = 1000
    else:
        o1 = False

    # Movement controls
    up_down = 1500
    left_right = 1500
    if w:
        up_down = 1000
    if s:
        up_down = 2000
    if a:
        left_right = 2000
    if d:
        left_right = 1000

    light = int((l + 1) * 500 + 1000)
    straighMove = keyboard.is_pressed('k')

    # Calculate motor speeds
    (leftMotor, rightMotor) = joystick_to_motor_speed(rightX, rightY)

    # Adjust for straight movement
    if straighMove:
        leftX = 1300
        leftY = 1700

    # Handle 'h' key for toggle
    if h:
        if open == 1000:
            open = 2000
            open1 = 2000
        else:
            open = 1000
            open1 = 1000

    # Format data as string
    s = "["

    # Add motor values with flag checking for changes
    if abs(1500 - leftMotor) > DEADZONE:
        flag3[0] = 0
        s += "0" + str(leftMotor) + ","
    elif flag3[0] == 0:
        flag3[0] = 1
        s += "0" + str(1500) + ","
    else:
        s += "0" + str(1500) + ","

    if abs(1500 - rightMotor) > DEADZONE:
        flag3[1] = 0
        s += "1" + str(rightMotor) + ","
    elif flag3[1] == 0:
        flag3[1] = 1
        s += "1" + str(1500) + ","
    else:
        s += "1" + str(1500) + ","

    # Add more controls to string
    if abs(leftX - 1500) > 5:
        flag3[2] = 0
        s += "2" + str(leftX) + ","
    elif flag3[2] == 0:
        flag3[2] = 1
        s += "2" + str(1500) + ","
    else:
        s += "2" + str(1500) + ","

    if abs(leftY - 1500) > 5:
        flag3[3] = 0
        s += "3" + str(leftY) + ","
    elif flag3[3] == 0:
        flag3[3] = 1
        s += "3" + str(1500) + ","
    else:
        s += "3" + str(1500) + ","

    if abs(base - 1500) > 5:
        flag3[4] = 0
        s += "4" + str(base) + ","
    elif flag3[4] == 0:
        flag3[4] = 1
        s += "4" + str(1500) + ","

    if lifter != 1500:
        flag3[5] = 0
        s += "5" + str(lifter) + ","
    elif flag3[5] == 0:
        flag3[5] = 1
        s += "5" + str(1500) + ","

    if gripper != 1500:
        flag3[6] = 0
        s += "6" + str(gripper) + ","
    elif flag3[6] == 0:
        flag3[6] = 1
        s += "6" + str(1500) + ","
    else:
        s += "6" + str(1500) + ","

    if lifter_mode != 1500:
        flag3[7] = 0
        s += "7" + str(lifter_mode) + ","
    elif flag3[7] == 0:
        flag3[7] = 1
        s += "7" + str(1500) + ","

    if speed_mode != 1500:
        flag3[8] = 0
        s += "8" + str(speed_mode) + ","
    elif flag3[8] == 0:
        flag3[8] = 1
        s += "8" + str(1500) + ","

    if arm != 1500:
        flag3[9] = 0
        s += "9" + str(arm) + ","
    elif flag3[9] == 0:
        flag3[9] = 1
        s += "9" + str(1500) + ","

    if up_down != 1500:
        flag3[11] = 0
        s += "B" + str(up_down) + ","
    elif flag3[11] == 0:
        flag3[11] = 1
        s += "B" + str(1500) + ","

    if left_right != 1500:
        flag3[12] = 0
        s += "C" + str(left_right) + ","
    elif flag3[12] == 0:
        flag3[12] = 1
        s += "C" + str(1500) + ","

    if open != 1500:
        flag3[13] = 0
        s += "D" + str(open) + ","
    elif flag3[13] == 0:
        flag3[13] = 1
        s += "D" + str(1500) + ","

    if open1 != 1500:
        flag3[14] = 0
        s += "E" + str(open1)+","
    elif flag3[14] == 0:
        flag3[14] = 1
        s += "E" + str(1500)+","

    s += "F"+str(ghurbe)+","
    s += "G"+str(uthbe)+","

    s += "]"

    # Clean up trailing comma if present
    if s[len(s) - 2] == ',':
        s = s[:len(s) - 2] + ']'
    if (arm == 1500):
        return "#"
    return "JOYSTICK:" + s


def transmitter_mode(port):
    """Transmit joystick data over UDP."""
    print(f"Starting joystick transmitter on {UDP_IP}:{port}")

    # Set multicast TTL to 1
    ttl = struct.pack('b', 1)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl)

    # Initialize pygame
    pygame.init()
    pygame.joystick.init()

    # Check for joystick
    if pygame.joystick.get_count() == 0:
        print("No joystick detected.")
        sys.exit(1)

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    print("Initialized joystick:", joystick.get_name())
    print("Number of axes:", joystick.get_numaxes())
    print("Number of buttons:", joystick.get_numbuttons())

    try:
        while True:
            # Read and send joystick data
            joy_data = read_joystick_data()
            sock.sendto(joy_data.encode('utf-8'), (UDP_IP, port))
            print(f"Sent: {joy_data}")
            time.sleep(0.1)  # 10Hz update rate
    except KeyboardInterrupt:
        print("Exiting transmitter mode...")
    finally:
        pygame.quit()


def receiver_mode(port):
    """Receive joystick data over UDP."""
    print(f"Starting joystick receiver on {UDP_IP}:{port}")

    # Bind to all interfaces on the given port for multicast
    sock.bind(('', port))

    # Join multicast group
    mreq = struct.pack("4sl", socket.inet_aton(UDP_IP), socket.INADDR_ANY)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

    try:
        while True:
            # Receive data
            data, addr = sock.recvfrom(1024)
            joy_data = data.decode('utf-8')
            print(f"Received from {addr}: {joy_data}")
    except KeyboardInterrupt:
        print("Exiting receiver mode...")


def main():
    """Main function to parse arguments and start in the appropriate mode."""
    if len(sys.argv) < 2 or sys.argv[1] not in ["tx", "rx"]:
        print("Usage:")
        print("  Transmit mode: joycast.py tx [port]")
        print("  Receive mode: joycast.py rx [port]")
        sys.exit(1)

    # Set the port (use default or command line argument)
    port = UDP_PORT
    if len(sys.argv) >= 3:
        port = int(sys.argv[2])

    # Start in appropriate mode
    if sys.argv[1] == "tx":
        transmitter_mode(port)
    else:  # rx mode
        receiver_mode(port)


if __name__ == "__main__":
    main()
