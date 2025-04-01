#!/usr/bin/env python
import socket
import struct
import sys
import time
import pygame
import keyboard
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import String
import tty
import termios

UDP_IP = "239.0.0.16"
UDP_PORT = 5476
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
mreq = struct.pack("4sl", socket.inet_aton(UDP_IP), socket.INADDR_ANY)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

DEADZONE = 30
open1 = 1000
open = 1000
cnt1 = 0
cnt2 = 0
flag3 = [0] * 15
l_state = 'on'


def joystick_to_motor_speed(x, y):
    x_norm = (x - 1500) / 500
    y_norm = (y - 1500) / 500
    left_speed_norm = y_norm + x_norm
    right_speed_norm = y_norm - x_norm
    left_speed = 1500 + 500 * left_speed_norm
    right_speed = 1500 + 500 * right_speed_norm
    left_speed = max(1000, min(2000, left_speed))
    right_speed = max(1000, min(2000, right_speed))
    return int(left_speed), int(right_speed)


def get_joystick_value(joystick):
    if "extreme" in joystick.get_name().lower():
        return ""
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


def read_joystick_data():
    global open, open1, cnt1, cnt2, flag3, l_state
    pygame.event.pump()
    joystick = pygame.joystick.Joystick(0)
    joystick_values = get_joystick_value(joystick)
    if joystick_values == "":
        return ""
    leftY = joystick_values["leftY"]
    leftX = joystick_values["leftX"]
    rightY = joystick_values["rightY"]
    rightX = joystick_values["rightX"]
    gripper_of = joystick_values["gripper_of"]
    gripper_on = joystick_values["gripper_on"]
    base = joystick_values["base"]
    lifter_mode = joystick_values["lifter_mode"]
    speed_mode = joystick_values["speed_mode"]
    lifter = joystick_values["lifter"]
    arm = joystick_values["arm"]
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
    gripper = gripper_on
    if gripper_of != 1500:
        gripper = gripper_of
    h = keyboard.is_pressed('h')
    l = keyboard.is_pressed('l')
    w = keyboard.is_pressed('w')
    s = keyboard.is_pressed('s')
    a = keyboard.is_pressed('a')
    d = keyboard.is_pressed('d')
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
    (leftMotor, rightMotor) = joystick_to_motor_speed(rightX, rightY)
    if straighMove:
        leftX = 1300
        leftY = 1700
    if h:
        if open == 1000:
            open = 2000
            open1 = 2000
        else:
            open = 1000
            open1 = 1000
    s_str = "["
    if abs(1500 - leftMotor) > DEADZONE:
        flag3[0] = 0
        s_str += "0" + str(leftMotor) + ","
    elif flag3[0] == 0:
        flag3[0] = 1
        s_str += "0" + str(1500) + ","
    if abs(1500 - rightMotor) > DEADZONE:
        flag3[1] = 0
        s_str += "1" + str(rightMotor) + ","
    elif flag3[1] == 0:
        flag3[1] = 1
        s_str += "1" + str(1500) + ","
    if abs(leftX - 1500) > 5:
        flag3[2] = 0
        s_str += "2" + str(leftX) + ","
    elif flag3[2] == 0:
        flag3[2] = 1
        s_str += "2" + str(1500) + ","
    if abs(leftY - 1500) > 5:
        flag3[3] = 0
        s_str += "3" + str(leftY) + ","
    elif flag3[3] == 0:
        flag3[3] = 1
        s_str += "3" + str(1500) + ","
    if abs(base - 1500) > 5:
        flag3[4] = 0
        s_str += "4" + str(base) + ","
    elif flag3[4] == 0:
        flag3[4] = 1
        s_str += "4" + str(1500) + ","
    if lifter != 1500:
        flag3[5] = 0
        s_str += "5" + str(lifter) + ","
    elif flag3[5] == 0:
        flag3[5] = 1
        s_str += "5" + str(1500) + ","
    if gripper != 1500:
        flag3[6] = 0
        s_str += "6" + str(gripper) + ","
    elif flag3[6] == 0:
        flag3[6] = 1
        s_str += "6" + str(1500) + ","
    if lifter_mode != 1500:
        flag3[7] = 0
        s_str += "7" + str(lifter_mode) + ","
    elif flag3[7] == 0:
        flag3[7] = 1
        s_str += "7" + str(1500) + ","
    if speed_mode != 1500:
        flag3[8] = 0
        s_str += "8" + str(speed_mode) + ","
    elif flag3[8] == 0:
        flag3[8] = 1
        s_str += "8" + str(1500) + ","
    if arm != 1500:
        flag3[9] = 0
        s_str += "9" + str(arm) + ","
    elif flag3[9] == 0:
        flag3[9] = 1
        s_str += "9" + str(1500) + ","
    if up_down != 1500:
        flag3[11] = 0
        s_str += "B" + str(up_down) + ","
    elif flag3[11] == 0:
        flag3[11] = 1
        s_str += "B" + str(1500) + ","
    if left_right != 1500:
        flag3[12] = 0
        s_str += "C" + str(left_right) + ","
    elif flag3[12] == 0:
        flag3[12] = 1
        s_str += "C" + str(1500) + ","
    if open != 1500:
        flag3[13] = 0
        s_str += "D" + str(open) + ","
    elif flag3[13] == 0:
        flag3[13] = 1
        s_str += "D" + str(1500) + ","
    if open1 != 1500:
        flag3[14] = 0
        s_str += "E" + str(open1)
    elif flag3[14] == 0:
        flag3[14] = 1
        s_str += "E" + str(1500)
    s_str += "]"
    if s_str[len(s_str) - 2] == ',':
        s_str = s_str[:len(s_str) - 2] + ']'
    return s_str


def transmitter_mode(port):
    print(f"Starting joystick transmitter on multicast group {UDP_IP}:{port}")
    pygame.init()
    pygame.joystick.init()
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
            joy_data = read_joystick_data()
            sock.sendto(joy_data.encode('utf-8'), (UDP_IP, port))
            print(f"Sent: {joy_data}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Exiting transmitter mode...")
    finally:
        pygame.quit()


def ros2_spin_thread(node):
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)


def receiver_mode(port, stop_event):
    print(f"Starting joystick receiver on multicast group {UDP_IP}:{port}")
    node = rclpy.create_node('joystick_receiver')
    publisher = node.create_publisher(String, '/joystick', 10)
    spin_thread = threading.Thread(target=ros2_spin_thread, args=(node,))
    spin_thread.daemon = True
    spin_thread.start()
    sock.bind((UDP_IP, port))
    sock.settimeout(0.5)
    try:
        while not stop_event.is_set() and rclpy.ok():
            try:
                data, addr = sock.recvfrom(1024)
            except socket.timeout:
                continue
            joy_data = data.decode('utf-8')
            print(f"Received from {addr}: {joy_data}")
            msg = String()
            msg.data = joy_data
            publisher.publish(msg)
    except KeyboardInterrupt:
        print("Exiting receiver mode...")
    finally:
        node.destroy_node()


def pos_reciever(stop_event):
    UDP_POS_PORT = 1234
    pos_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    pos_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    mreq_pos = struct.pack("4sl", socket.inet_aton(UDP_IP), socket.INADDR_ANY)
    pos_sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq_pos)
    pos_sock.bind((UDP_IP, UDP_POS_PORT))
    pos_sock.settimeout(0.5)

    class PosReceiver(Node):
        def __init__(self):
            super().__init__('pos_receiver')
            qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST,
                                     depth=10,
                                     reliability=QoSReliabilityPolicy.BEST_EFFORT)
            self.publisher_ = self.create_publisher(
                String, 'position', qos_profile)

        def publish_pos(self, data):
            msg = String()
            msg.data = data
            self.publisher_.publish(msg)
    pos_node = PosReceiver()
    try:
        while not stop_event.is_set() and rclpy.ok():
            try:
                data, addr = pos_sock.recvfrom(1024)
            except socket.timeout:
                continue
            pos_data = data.decode('utf-8')
            pos_node.publish_pos(pos_data)
            print(f"Received position: {pos_data}")
    except KeyboardInterrupt:
        print("Exiting pos_reciever...")
    finally:
        pos_node.destroy_node()


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ["tx", "rx"]:
        print("Usage:")
        print("  Transmit mode: joycast.py tx [port]")
        print("  Receive mode:  joycast.py rx [port]")
        sys.exit(1)
    if sys.argv[1] in ["tx", "rx"]:
        port = UDP_PORT
        if len(sys.argv) >= 3:
            port = int(sys.argv[2])
    if sys.argv[1] == "tx":
        transmitter_mode(port)
    elif sys.argv[1] == "rx":
        rclpy.init(args=None)
        stop_event = threading.Event()
        rx_thread = threading.Thread(
            target=receiver_mode, args=(port, stop_event))
        pos_thread = threading.Thread(target=pos_reciever, args=(stop_event,))
        rx_thread.start()
        pos_thread.start()
        try:
            while rx_thread.is_alive() and pos_thread.is_alive():
                time.sleep(0.5)
        except KeyboardInterrupt:
            stop_event.set()
        finally:
            stop_event.set()
            rx_thread.join()
            pos_thread.join()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
