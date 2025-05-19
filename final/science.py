#!/usr/bin/env python
import socket
import struct
import sys
import time
import pygame
import threading

# Address settings
BODY_IP = "192.168.2.177"  # IP for body controls
SCIENCE_IP = "192.168.2.179"  # IP for science arm
UDP_PORT = 5010  # Default port, can be overridden with command line arg

# Create UDP sockets
body_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
body_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
science_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
science_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
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
flag3 = [0] * 18 # Increased size for all unique controls
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
        "leftY": joystick.get_axis(3),
        "leftX": joystick.get_axis(2),
        "rightY": joystick.get_axis(1),
        "rightX":  joystick.get_axis(0),
        "axis4": joystick.get_axis(4),
    }


mode = 0


def get_joystick_value1(joystick):

    return {
        "leftY":  joystick.get_axis(0),
        "leftX": joystick.get_axis(1),
        "axis2": joystick.get_axis(2),  
        "servo1": joystick.get_button(4),
        "servo2": joystick.get_button(5),
        "servo3": joystick.get_button(2),
        "servo4": joystick.get_button(3),
        "pump1": joystick.get_button(6),
        "pump2": joystick.get_button(7),
        "pump3": joystick.get_button(8),
        "pump4": joystick.get_button(9),
        "relay1": joystick.get_button(10),
        "relay2": joystick.get_button(11),
        "relay3": joystick.get_button(0),
        "relay4": joystick.get_button(1),
    }


def apply_deadzone(value, center=1500, radius=100):
    """Apply a deadzone around the center value."""
    if abs(value - center) <= radius:
        return center
    return value


def read_joystick_data():
    global open, open1, cnt1, cnt2, flag3, l_state

    pygame.event.pump()
    joystick = pygame.joystick.Joystick(0)
    arm = pygame.joystick.Joystick(1)

    # Get joystick values using helper function.
    joystick_values = get_joystick_value(joystick)
    joystick_values1 = get_joystick_value1(arm)

    # Body control values (from joystick 0)
    rightY = joystick_values["rightY"]
    rightX = joystick_values["rightX"]

    rightY = int((rightY + 1) * 500 + 1000)
    rightX = int((rightX + 1) * 500 + 1000)

    # Science arm values (from joystick 1)
    leftY = joystick_values1["leftY"]
    leftX = joystick_values1["leftX"]
    axis2 = joystick_values1["axis2"]  # Get new axis2 value
    axis4 = joystick_values["axis4"]  # Get new axis4 value

    servo1 = joystick_values1["servo1"]
    servo2 = joystick_values1["servo2"]
    servo3 = joystick_values1["servo3"]
    servo4 = joystick_values1["servo4"]
    pump1 = joystick_values1["pump1"]
    pump2 = joystick_values1["pump2"]
    pump3 = joystick_values1["pump3"]
    pump4 = joystick_values1["pump4"]
    relay1 = joystick_values1["relay1"]
    relay2 = joystick_values1["relay2"]
    relay3 = joystick_values1["relay3"]
    relay4 = joystick_values1["relay4"]

    # Map from -1 to 1 to 1000 to 2000 for science arm controls
    leftY = int((leftY + 1) * 500 + 1000)
    leftX = int((leftX + 1) * 500 + 1000)
    axis2 = int((axis2 + 1) * 500 + 1000)  # Map axis2
    axis4 = int((axis4 + 1) * 500 + 1000)  # Map axis4
    servo1 = int((servo1 + 1) * 500 + 1000)
    servo2 = int((servo2 + 1) * 500 + 1000)
    servo3 = int((servo3 + 1) * 500 + 1000)
    servo4 = int((servo4 + 1) * 500 + 1000)
    pump1 = int((pump1 + 1) * 500 + 1000)
    pump2 = int((pump2 + 1) * 500 + 1000)
    pump3 = int((pump3 + 1) * 500 + 1000)
    pump4 = int((pump4 + 1) * 500 + 1000)
    relay1 = int((relay1 + 1) * 500 + 1000)
    relay2 = int((relay2 + 1) * 500 + 1000)
    relay3 = int((relay3 + 1) * 500 + 1000)
    relay4 = int((relay4 + 1) * 500 + 1000)

    # Apply 50-value radius deadzone to all axes
    leftY = apply_deadzone(leftY)
    leftX = apply_deadzone(leftX)
    axis2 = apply_deadzone(axis2)
    axis4 = apply_deadzone(axis4)

    base = joystick.get_axis(4);
    base = int((base + 1) * 500 + 1000)
    # Calculate motor speeds for body
    (leftMotor, rightMotor) = (rightX, rightY)

    # Format body data as string
    body_data = "["

    # Add motor values with flag checking for changes
    if abs(1500 - leftMotor) > DEADZONE:
        flag3[0] = 0
        body_data += "0" + str(leftMotor) + ","
    elif flag3[0] == 0:
        flag3[0] = 1
        body_data += "0" + str(1500) + ","
    else:
        body_data += "0" + str(1500) + ","

    if abs(1500 - rightMotor) > DEADZONE:
        flag3[1] = 0
        body_data += "1" + str(rightMotor) + ","
    elif flag3[1] == 0:
        flag3[1] = 1
        body_data += "1" + str(1500) + ","
    else:
        body_data += "1" + str(1500) + ","

    if abs(1500 - base) > DEADZONE:
        flag3[2] = 0 # Corrected flag index for base
        body_data += "7" + str(base) + ","
    elif flag3[2] == 0: # Corrected flag index for base
        flag3[2] = 1 # Corrected flag index for base
        body_data += "7" + str(1500) + ","
    else:
        body_data += "7" + str(1500) + ","

    if (abs(1500-leftY) > DEADZONE):
        flag3[3] = 0 # Corrected flag index for leftY
        body_data += "3" + str(leftY) + ","
    elif flag3[3] == 0: # Corrected flag index for leftY
        flag3[3] = 1 # Corrected flag index for leftY
        body_data += "3" + str(1500) + ","
    else:
        body_data += "3" + str(1500) + ","
        
    if (abs(1500-leftX) > DEADZONE):
        flag3[4] = 0 # Corrected flag index for leftX
        body_data += "2" + str(leftX) + "," # Added comma for consistency
    elif flag3[4] == 0: # Corrected flag index for leftX
        flag3[4] = 1 # Corrected flag index for leftX
        body_data += "2" + str(1500) + "," # Added comma for consistency
    else:
        body_data += "2" + str(1500) + "," # Added comma for consistency

    

    body_data += "]"

    # Clean up trailing comma if present in body data
    if body_data[len(body_data) - 2] == ',':
        body_data = body_data[:len(body_data) - 2] + ']'

    # Format science arm data as string
    science_data = "["

    # Add all the science arm controls
    if (abs(1500-servo1) > DEADZONE):
        flag3[5] = 0 # Corrected flag index
        science_data += "0" + str(servo1) + ","
    elif flag3[5] == 0: # Corrected flag index
        flag3[5] = 1 # Corrected flag index
        science_data += "0" + str(1500) + ","
    else:
        science_data += "0" + str(1500) + ","
        
    if (abs(1500-servo2) > DEADZONE):
        flag3[6] = 0 # Corrected flag index
        science_data += "1" + str(servo2) + ","
    elif flag3[6] == 0: # Corrected flag index
        flag3[6] = 1 # Corrected flag index
        science_data += "1" + str(1500) + ","
    else:
        science_data += "1" + str(1500) + ","

    if (abs(1500-servo3) > DEADZONE):
        flag3[7] = 0 # Corrected flag index
        science_data += "2" + str(servo3) + "," # Corrected motor ID from "1" to "2"
    elif flag3[7] == 0: # Corrected flag index
        flag3[7] = 1 # Corrected flag index
        science_data += "2" + str(1500) + "," # Corrected motor ID from "1" to "2"
    else:
        science_data += "2" + str(1500) + "," # Corrected motor ID from "1" to "2"
        
    if (abs(1500-servo4) > DEADZONE):
        flag3[8] = 0 # Corrected flag index
        science_data += "3" + str(servo4) + ","
    elif flag3[8] == 0: # Corrected flag index
        flag3[8] = 1 # Corrected flag index
        science_data += "3" + str(1500) + ","
    else:
        science_data += "3" + str(1500) + ","
        
    if (abs(1500-pump1) > DEADZONE):
        flag3[9] = 0 # Corrected flag index
        science_data += "4" + str(pump1) + ","
    elif flag3[9] == 0: # Corrected flag index
        flag3[9] = 1 # Corrected flag index
        science_data += "4" + str(1500) + ","
    else:
        science_data += "4" + str(1500) + ","

    if (abs(1500-pump2) > DEADZONE):
        flag3[10] = 0 # Corrected flag index
        science_data += "5" + str(pump2)+ ","
    elif flag3[10] == 0: # Corrected flag index
        flag3[10] = 1 # Corrected flag index
        science_data += "5" + str(1500)+ ","
    else:
        science_data += "5" + str(1500)+ ","

    if (abs(1500-pump3) > DEADZONE):
        flag3[11] = 0 # Corrected flag index
        science_data += "6" + str(pump3) + ","
    elif flag3[11] == 0: # Corrected flag index
        flag3[11] = 1 # Corrected flag index
        science_data += "6" + str(1500) + ","
    else:
        science_data += "6" + str(1500) + ","

    if (abs(1500-pump4) > DEADZONE):
        flag3[12] = 0 # Corrected flag index
        science_data += "7" + str(pump4) + ","
    elif flag3[12] == 0: # Corrected flag index
        flag3[12] = 1 # Corrected flag index
        science_data += "7" + str(1500) + ","
    else:
        science_data += "7" + str(1500) + ","

    if (abs(1500-relay1) > DEADZONE):
        flag3[13] = 0 # Corrected flag index
        science_data += "8" + str(relay1) + ","
    elif flag3[13] == 0: # Corrected flag index
        flag3[13] = 1 # Corrected flag index
        science_data += "8" + str(1500) + ","
    else:
        science_data += "8" + str(1500) + ","

    if (abs(1500-relay2) > DEADZONE):
        flag3[14] = 0 # Corrected flag index
        science_data += "9" + str(relay2) + ","
    elif flag3[14] == 0: # Corrected flag index
        flag3[14] = 1 # Corrected flag index
        science_data += "9" + str(1500) + ","
    else:
        science_data += "9" + str(1500) + ","

    if (abs(1500-relay3) > DEADZONE):
        flag3[15] = 0 # Corrected flag index
        science_data += "A" + str(relay3) + ","
    elif flag3[15] == 0: # Corrected flag index
        flag3[15] = 1 # Corrected flag index
        science_data += "A" + str(1500) + ","
    else:
        science_data += "A" + str(1500) + ","

    if (abs(1500-relay4) > DEADZONE):
        flag3[16] = 0 # Corrected flag index
        science_data += "B" + str(relay4) + ","
    elif flag3[16] == 0: # Corrected flag index
        flag3[16] = 1 # Corrected flag index
        science_data += "B" + str(1500) + ","
    else:
        science_data += "B" + str(1500) + ","

    # Add new axis values to science data string (after the existing controls)
    if (abs(1500-axis2) > DEADZONE):
        flag3[17] = 0 # Corrected flag index
        science_data += "C" + str(axis2) + ","
    elif flag3[17] == 0: # Corrected flag index
        flag3[17] = 1 # Corrected flag index
        science_data += "C" + str(1500) + ","
    else:
        science_data += "C" + str(1500) + ","

   
        
    # Add leftY and leftX again as the last two controls
    

    science_data += "]"
    
    # Clean up trailing comma if present in science data
    if science_data[len(science_data) - 2] == ',':
        science_data = science_data[:len(science_data) - 2] + ']'

    # Check if we should send data or not
    if (joystick.get_button(0) == 1):
        arm = 2000
    else:
        arm = 1500
    
    if (arm == 1500):
        return "#", "#"
    
    return "JOYSTICK:" + body_data, "JOYSTICK:" + science_data


def transmitter_mode(port):
    """Transmit joystick data over UDP to both body and science destinations."""
    print(f"Starting joystick transmitter to body({BODY_IP}) and science({SCIENCE_IP}) on port {port}")

    # Initialize pygame
    pygame.init()
    pygame.joystick.init()

    # Check for joystick
    if pygame.joystick.get_count() < 2:
        print(f"Need 2 joysticks, but only {pygame.joystick.get_count()} detected.")
        sys.exit(1)

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    arm = pygame.joystick.Joystick(1)
    arm.init()

    print("Initialized main joystick:", joystick.get_name())
    print("Initialized arm joystick:", arm.get_name())

    try:
        while True:
            # Read joystick data (now returns two separate strings)
            body_data, science_data = read_joystick_data()
            
            # Send body data to BODY_IP
            if body_data != "#":
                body_sock.sendto(body_data.encode('utf-8'), (BODY_IP, port))
                print(f"Sent to body: {body_data}")
            
            # Send science data to SCIENCE_IP
            if science_data != "#":
                science_sock.sendto(science_data.encode('utf-8'), (SCIENCE_IP, port))
                print(f"Sent to science: {science_data}")
            
            time.sleep(0.1)  # 10Hz update rate
    except KeyboardInterrupt:
        print("Exiting transmitter mode...")
    finally:
        pygame.quit()


def receiver_mode(port):
    """Receive joystick data over UDP."""
    print(f"Starting joystick receiver on port {port}")

    # Bind to all interfaces on the given port for unicast
    sock.bind(('', port))

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
