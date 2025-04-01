#!/usr/bin/env python
import socket
import struct
import sys
import tty
import termios
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import String


UDP_IP = "239.0.0.16"
UDP_PORT = 1234

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
# Join multicast group
mreq = struct.pack("4sl", socket.inet_aton(UDP_IP), socket.INADDR_ANY)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

if len(sys.argv) < 2 or sys.argv[1] not in ["tx", "rx"]:
    print("Usage: keycast.py [tx|rx]")
    sys.exit(1)


if sys.argv[1] == "tx":
    print("Press keys to send them over multicast. Press 'q' to quit.")
    while True:
        ch = input('Enter motor number (1-4) or "quit": ')
        if ch == 'quit':
            break
        val = input('Enter value (0-360): ')

        val = float(val)*(7.0/90.0)

        combined = f"{ch} {val}"
        sock.sendto(combined.encode('utf-8'), (UDP_IP, UDP_PORT))
else:  # rx mode
    sock.bind((UDP_IP, UDP_PORT))
    print("Receiving keystrokes:")
    rclpy.init()

    class KeyReceiver(Node):
        def __init__(self):
            super().__init__('key_receiver')
            qos_profile = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=QoSReliabilityPolicy.BEST_EFFORT
            )
            self.publisher_ = self.create_publisher(
                String, 'position', qos_profile)

        def publish_key(self, data):
            msg = String()
            msg.data = data
            self.publisher_.publish(msg)
    receiver = KeyReceiver()

    try:
        while rclpy.ok():
            data, addr = sock.recvfrom(1024)
            if data:
                key = data.decode('utf-8')
                receiver.publish_key(key)
                print(f"{data.decode('utf-8')}", end='', flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        receiver.destroy_node()
        rclpy.shutdown()
