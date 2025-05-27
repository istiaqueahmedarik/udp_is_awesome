#!/usr/bin/env python
import socket
import struct
import sys
import time
import pygame
import threading

# Multicast settings
UDP_IP = "192.168.2.177"  # Changed to unicast address
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


mode = 0


def get_joystick_value(joystick):
    if joystick is None:
        return {
            "leftY": 0,
            "leftX": 0,
            "rightY": 0,
            "rightX":  0,
        }

    ghora = 0
    if (joystick.get_button(4) == 1):
        ghora = 1
    if (joystick.get_button(4) == 1 and joystick.get_button(2) == 1):
        ghora = -1
    upor = 0
    if (joystick.get_button(3) == 1):
        upor = 1
    if (joystick.get_button(3) == 1 and joystick.get_button(2) == 1):
        upor = -1

    return {
        "leftY": -1*joystick.get_axis(2),
        "leftX": -1*joystick.get_axis(3),
        "rightY": joystick.get_axis(1),
        "rightX":  joystick.get_axis(0),
    }


mode = 0


def get_joystick_value1(joystick):
    if joystick is None:
        return {
            "leftY": 0,
            "leftX": 0,
            "base": 0,
            "grip": 0,
            "ghora": 0,
            "upor": 0,
        }
    mode = joystick.get_button(9)

    grip = 0
    if (joystick.get_button(4) == 1):
        grip = 1
    if (joystick.get_button(5) == 1):
        grip = -1
    ghora = 0
    if (joystick.get_button(2) == 1):
        ghora = 1
    if (joystick.get_button(3) == 1):
        ghora = -1
    upor = 0
    if (joystick.get_button(0) == 1):
        upor = 1
    if (joystick.get_button(1) == 1):
        upor = -1

    return {
        "leftY":  joystick.get_axis(0),
        "leftX": joystick.get_axis(1),
        "base": joystick.get_axis(2),
        "grip": grip,
        "ghora": ghora,
        "upor": upor,
    }


def apply_deadzone(value, center=1500, radius=100):
    """Apply a deadzone around the center value."""
    if abs(value - center) <= radius:
        return center
    return value


def read_joystick_data():
    global open, open1, cnt1, cnt2, flag3, l_state

    pygame.event.pump()
    # joystick = pygame.joystick.Joystick(0)
    # arm = pygame.joystick.Joystick(1)

    joystick = None
    arm = None
    if pygame.joystick.get_count() > 0:
        tmp = pygame.joystick.Joystick(0)
        if tmp.get_name().count("Extreme") > 0 or tmp.get_name().count("extreme") > 0:
            arm = tmp
            arm.init()
        else:
            joystick = tmp
            joystick.init()
    if pygame.joystick.get_count() > 1:
        tmp = pygame.joystick.Joystick(1)
        if tmp.get_name().count("Extreme") > 0 or tmp.get_name().count("extreme") > 0:
            arm = tmp
            arm.init()
        else:
            joystick = tmp
            joystick.init()
    if joystick is None and arm is None:
        idleValue = "JOYSTICK:[01500,11500,21500,31500]"
        sock.sendto(idleValue.encode('utf-8'), (UDP_IP, UDP_PORT))
        print(f"Sent idle value: {idleValue}")

    # Get joystick values using helper function.
    joystick_values = get_joystick_value(joystick)
    joystick_values1 = get_joystick_value1(arm)

    rightY = joystick_values["rightY"]
    rightX = joystick_values["rightX"]

    rightY = int((rightY + 1) * 500 + 1000)
    rightX = int((rightX + 1) * 500 + 1000)

    leftY = joystick_values["leftY"]
    leftX = joystick_values["leftX"]
    if (arm is not None and arm.get_button(7) == 1):
        leftY = 0

    grip = joystick_values1["grip"]
    ghora = joystick_values1["ghora"]
    upor = joystick_values1["upor"]
    base = joystick_values1["base"]
    # Map from -1 to 1 to 1000 to 2000
    leftY = int((leftY + 1) * 500 + 1000)
    leftX = int((leftX + 1) * 500 + 1000)
    grip = int((grip + 1) * 500 + 1000)
    ghora = int((ghora + 1) * 500 + 1000)
    upor = int((upor + 1) * 500 + 1000)
    base = int((base + 1) * 500 + 1000)

    # patch
    if (base > 1500):
        base = 1500-(abs(1500-base))
    else:
        base = 1500+(abs(1500-base))

    # if (leftX > 1500):
    #     leftX = 1500-(abs(1500-leftX))
    # else:
    #     leftX = 1500+(abs(1500-leftX))

    if (leftY > 1500):
        leftY = 1500-(abs(1500-leftY))
    else:
        leftY = 1500+(abs(1500-leftY))

    # Apply 50-value radius deadzone to all axes
    leftY = apply_deadzone(leftY)
    leftX = apply_deadzone(leftX, radius=200)
    tmp = leftY
    leftY = leftX
    leftX = tmp
    base = apply_deadzone(base, center=1500, radius=100)
    if (base > 1500):
        base = 1500-abs(1500-base)
    else:
        base = 1500+abs(1500-base)

    mode = 1500
    if (arm is not None):
        mode = arm.get_button(9)
    if (mode == 0):
        base = 1500

    # Handle 'p' key for toggle

    # Calculate motor speeds
    (leftMotor, rightMotor) = (rightX, rightY)

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

    if (abs(1500-leftX) > DEADZONE):
        flag3[2] = 0
        s += "2" + str(leftX) + ","
    elif flag3[2] == 0:
        flag3[2] = 1
        s += "2" + str(1500) + ","
    else:
        s += "2" + str(1500) + ","
    if (abs(1500-leftY) > DEADZONE):
        flag3[3] = 0
        s += "3" + str(leftY) + ","
    elif flag3[3] == 0:
        flag3[3] = 1
        s += "3" + str(1500) + ","
    else:
        s += "3" + str(1500) + ","

    if (abs(1500-grip) > DEADZONE):
        flag3[4] = 0
        s += "4" + str(grip) + ","
    elif flag3[4] == 0:
        flag3[4] = 1
        s += "4" + str(1500) + ","
    else:
        s += "4" + str(1500) + ","
    if (abs(1500-ghora) > DEADZONE):
        flag3[5] = 0
        s += "5" + str(ghora) + ","
    elif flag3[5] == 0:
        flag3[5] = 1
        s += "5" + str(1500) + ","
    else:
        s += "5" + str(1500) + ","
    if (abs(1500-upor) > DEADZONE):
        flag3[6] = 0
        s += "6" + str(upor) + ","
    elif flag3[6] == 0:
        flag3[6] = 1
        s += "6" + str(1500) + ","
    else:
        s += "6" + str(1500) + ","

    if (abs(1500-base) > DEADZONE):
        flag3[7] = 0
        s += "7" + str(base)
    elif flag3[7] == 0:
        flag3[7] = 1
        s += "7" + str(1500)
    else:
        s += "7" + str(1500)

    s += "]"

    # Clean up trailing comma if present
    if s[len(s) - 2] == ',':
        s = s[:len(s) - 2] + ']'
    if (joystick is None):
        arm = 2000
    elif (joystick is not None and joystick.get_button(0) == 1):
        arm = 2000
    else:
        arm = 1500
    if (arm == 1500):
        return "#"
    return "JOYSTICK:"+s


def transmitter_mode(port):
    """Transmit joystick data over UDP."""
    print(f"Starting joystick transmitter on {UDP_IP}:{port}")

    # Set multicast TTL to 1
    # ttl = struct.pack('b', 1)
    # sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl)
    # No need to set TTL for unicast

    # Initialize pygame
    pygame.init()
    pygame.joystick.init()

    idleValue = "JOYSTICK:[01500,11500,21500,31500]"
    sock.sendto(idleValue.encode('utf-8'), (UDP_IP, port))
    print(f"Sent idle value: {idleValue}")

    try:
        while True:
            # Read and send joystick data
            joy_data = read_joystick_data()
            sock.sendto(joy_data.encode('utf-8'), (UDP_IP, port))
            print(f"Sent: {joy_data}")
            # run in 120hz
            # time.sleep(1 / 720.0)

    except KeyboardInterrupt:
        print("Exiting transmitter mode...")
    finally:
        pygame.quit()


def receiver_mode(port):
    """Receive joystick data over UDP."""
    print(f"Starting joystick receiver on {UDP_IP}:{port}")

    # Bind to all interfaces on the given port for unicast
    sock.bind(('', port))

    # No need to join multicast group for unicast

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
