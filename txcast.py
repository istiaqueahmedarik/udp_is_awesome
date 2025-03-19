#!/usr/bin/env python
import socket
import struct
import sys

UDP_IP = "239.0.0.16"
UDP_PORT = 1234

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
# Join multicast group
mreq = struct.pack("4sl", socket.inet_aton(UDP_IP), socket.INADDR_ANY)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

if len(sys.argv) < 2 or sys.argv[1] not in ["tx", "rx"]:
    print("Usage: txcast.py [tx|rx]")
    sys.exit(1)

if sys.argv[1] == "tx":
    while True:
        try:
            line = input("Enter text to send: ")
        except EOFError:
            break
        if not line:
            continue
        sock.sendto(line.encode('utf-8'), (UDP_IP, UDP_PORT))
else:  # rx mode
    sock.bind((UDP_IP, UDP_PORT))
    print("Receiving text stream:")
    while True:
        data, addr = sock.recvfrom(1024)
        print(f"Received: {data.decode('utf-8')}")
