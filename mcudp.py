# The same exampel as udptest.py but transmitting
# and receiving from a multicast group

import socket
import struct
import sys
import time

UDP_IP = "239.0.0.16"
UDP_PORT = 1234

if len(sys.argv) < 2:
    print("Missing mode")
    sys.exit(1)

sock = socket.socket(
    socket.AF_INET,
    socket.SOCK_DGRAM
)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

mreq = struct.pack("4sl", socket.inet_aton(UDP_IP), socket.INADDR_ANY)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

if sys.argv[1] == "tx":
    msg = f"{time.time()}"
    if len(sys.argv) >= 3:
        msg = sys.argv[2]
    msg = bytes(msg, encoding="utf-8")
    sock.sendto(msg, (UDP_IP, UDP_PORT))
elif sys.argv[1] == "rx":
    sock.bind((UDP_IP, UDP_PORT))
    while True:
        data, addr = sock.recvfrom(1024)
        data = data.decode('utf-8')
        print(f"Received from {addr}: {data}")
else:
    print(f"Unknown mode {sys.argv[1]}")
