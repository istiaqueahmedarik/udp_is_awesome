#!/usr/bin/env python
from __future__ import division
import cv2
import numpy as np
import socket
import struct
import math
import sys
import time

global camIdx

# Multicast settings from mcudp.py
UDP_IP = "239.0.0.16"
global UDP_PORT

# Create UDP multicast socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
# Join multicast group (optional for sending; helps when testing local rx mode)
mreq = struct.pack("4sl", socket.inet_aton(UDP_IP), socket.INADDR_ANY)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)


class FrameSegment(object):
    # Use same maximum datagram size minus a header value for image
    MAX_DGRAM = 2**16
    MAX_IMAGE_DGRAM = MAX_DGRAM - 64  # reserve extra bytes for header

    def __init__(self, sock, port, addr=UDP_IP):
        self.s = sock
        self.port = port
        self.addr = addr

    def udp_frame(self, img):
        # compress image similar to sender.py
        compress_img = cv2.imencode('.jpg', img)[1]
        # optional - reduce quality and size
        params = [int(cv2.IMWRITE_JPEG_QUALITY), 50]
        compress_img = cv2.imencode('.jpg', img, params)[1]

        dat = compress_img.tostring()
        size = len(dat)
        count = math.ceil(size / (self.MAX_IMAGE_DGRAM))
        array_pos_start = 0
        while count:
            array_pos_end = min(size, array_pos_start + self.MAX_IMAGE_DGRAM)
            # send segment with first byte as segment counter
            segment = struct.pack("B", count) + \
                dat[array_pos_start:array_pos_end]
            self.s.sendto(segment, (self.addr, self.port))
            array_pos_start = array_pos_end
            count -= 1

# New helper function for receiving mode:


def dump_buffer(s):
    MAX_DGRAM = 2**16
    while True:
        seg, addr = s.recvfrom(MAX_DGRAM)
        print(seg[0])
        if struct.unpack("B", seg[0:1])[0] == 1:
            print("finish emptying buffer")
            break


def main():
    if len(sys.argv) < 4 or sys.argv[1] not in ["tx", "rx"]:
        print("Usage: imcast.py [tx|rx] [camera] [port]")
        sys.exit(1)
    global camIdx
    camIdx = int(sys.argv[2])
    global UDP_PORT
    UDP_PORT = int(sys.argv[3])

    if sys.argv[1] == "tx":
        fs = FrameSegment(sock, UDP_PORT)
        cap = cv2.VideoCapture(camIdx)
        if not cap.isOpened():
            print("Cannot open camera")
            sys.exit(1)
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break
            fs.udp_frame(frame)
        cap.release()
        sock.close()
    elif sys.argv[1] == "rx":
        sock.bind((UDP_IP, UDP_PORT))
        dump_buffer(sock)
        dat = b''
        MAX_DGRAM = 2**16
        while True:
            seg, addr = sock.recvfrom(MAX_DGRAM)
            if struct.unpack("B", seg[0:1])[0] > 1:
                dat += seg[1:]
            else:
                dat += seg[1:]
                img = cv2.imdecode(np.fromstring(dat, dtype=np.uint8), 1)
                cv2.imshow('frame', img)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                dat = b''
        cv2.destroyAllWindows()
        sock.close()


if __name__ == '__main__':
    main()
