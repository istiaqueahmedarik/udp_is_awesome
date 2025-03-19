#!/usr/bin/env python
import socket
import struct
import sys
import tty
import termios

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


def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


if sys.argv[1] == "tx":
    print("Press keys to send them over multicast. Press 'q' to quit.")
    while True:
        ch = getch()
        if ch == 'q':
            break
        sock.sendto(ch.encode('utf-8'), (UDP_IP, UDP_PORT))
else:  # rx mode
    sock.bind((UDP_IP, UDP_PORT))
    print("Receiving keystrokes:")
    while True:
        data, addr = sock.recvfrom(1024)
        # Print received key without newline formatting
        print(f"{data.decode('utf-8')}", end='', flush=True)
