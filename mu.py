import cv2
import numpy as np
import socket
import struct
import math
import sys
import time
import threading
import glob
import pyudev
import argparse

#!/usr/bin/env python3

BASE_UDP_PORT = 1234
MAX_CAMERAS = 10
camera_streams = {}
frames = {}
last_update = {}
dynamic_ports = list(range(BASE_UDP_PORT, BASE_UDP_PORT + MAX_CAMERAS))


class FrameSegment:
    MAX_DGRAM = 2**16
    MAX_IMAGE_DGRAM = MAX_DGRAM - 64

    def __init__(self, sock, port, addr):
        self.s = sock
        self.port = port
        self.addr = addr

    def udp_frame(self, img):
        quality = 100
        params = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
        dat = cv2.imencode('.jpg', img, params)[1].tobytes()
        size = len(dat)
        count = math.ceil(size / self.MAX_IMAGE_DGRAM)
        pos = 0
        while count:
            end = min(size, pos + self.MAX_IMAGE_DGRAM)
            seg = struct.pack("B", count) + dat[pos:end]
            self.s.sendto(seg, (self.addr, self.port))
            pos = end
            count -= 1


def stream_camera(device_node, udp_port, recv_ip):
    cap = cv2.VideoCapture(device_node)
    if not cap.isOpened():
        return
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    fs = FrameSegment(sock, udp_port, recv_ip)
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        fs.udp_frame(frame)
    cap.release()


def detect_existing_cameras():
    return glob.glob("/dev/video*")


def add_camera(device_node, recv_ip):
    if device_node in camera_streams:
        return
    udp_port = BASE_UDP_PORT + len(camera_streams)
    t = threading.Thread(
        target=stream_camera,
        args=(device_node, udp_port, recv_ip),
        daemon=True
    )
    camera_streams[device_node] = {'port': udp_port, 'thread': t}
    t.start()


def remove_camera(device_node):
    if device_node in camera_streams:
        del camera_streams[device_node]


def monitor_new_cameras(recv_ip):
    ctx = pyudev.Context()
    mon = pyudev.Monitor.from_netlink(ctx)
    mon.filter_by(subsystem='video4linux')
    for dev in iter(mon.poll, None):
        node = dev.device_node
        if dev.action == 'add':
            add_camera(node, recv_ip)
        elif dev.action == 'remove':
            remove_camera(node)


def tx_mode(recv_ip):
    existing = detect_existing_cameras()
    for dev in existing:
        add_camera(dev, recv_ip)
    t = threading.Thread(target=monitor_new_cameras,
                         args=(recv_ip,), daemon=True)
    t.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        sys.exit(0)


def dump_buffer(s):
    max_dgram = 2**16
    while True:
        seg, _ = s.recvfrom(max_dgram)
        if struct.unpack("B", seg[:1])[0] == 1:
            break


def receiver_thread(port):
    global frames, last_update
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(('0.0.0.0', port))
    dump_buffer(s)
    max_dgram = 2**16
    dat = b''
    while True:
        seg, _ = s.recvfrom(max_dgram)
        header = struct.unpack("B", seg[:1])[0]
        dat += seg[1:]
        if header == 1:
            img = cv2.imdecode(np.frombuffer(dat, dtype=np.uint8), 1)
            if img is not None:
                frames[port] = img
                last_update[port] = time.time()
            dat = b''
    s.close()


def dashboard_mouse_callback(event, x, y, flags, param):
    global selected_port, ports_list
    if event == cv2.EVENT_LBUTTONDOWN:
        h = param['height']
        if not ports_list:
            return
        idx = y // (h // len(ports_list))
        if idx < len(ports_list):
            selected_port = ports_list[idx]


def rx_mode_dynamic():
    global selected_port, ports_list
    for port in dynamic_ports:
        t = threading.Thread(target=receiver_thread, args=(port,), daemon=True)
        t.start()
    selected_port = None
    w, h = 1200, 800
    cv2.namedWindow('Dashboard')
    cv2.setMouseCallback('Dashboard', dashboard_mouse_callback, {'height': h})
    timeout = 3
    while True:
        now = time.time()
        ports_list = [
            p for p in dynamic_ports
            if p in last_update and now - last_update[p] < timeout
        ]
        if ports_list and selected_port not in ports_list:
            selected_port = ports_list[0]
        if selected_port in frames:
            focus = cv2.resize(frames[selected_port], (w, h))
        else:
            focus = np.zeros((h, w, 3), np.uint8)
        thumbs = []
        th = h // (len(ports_list) or 1)
        for p in ports_list:
            if p in frames:
                t = cv2.resize(frames[p], (300, th))
            else:
                t = np.zeros((th, 300, 3), np.uint8)
            cv2.putText(t, f"Port {p}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            thumbs.append(t)
        if thumbs:
            rp = np.vstack(thumbs)
            right = cv2.resize(rp, (300, h))
        else:
            right = np.zeros((h, 300, 3), np.uint8)
        dash = np.hstack((focus, right))
        cv2.imshow('Dashboard', dash)
        if cv2.waitKey(30) & 0xFF == 27:
            break
    cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('mode', choices=['tx', 'rx'])
    parser.add_argument('-r', '--receiver-ip')
    args = parser.parse_args()
    if args.mode == 'tx':
        if not args.receiver_ip:
            parser.error('Receiver IP required in tx mode')
        tx_mode(args.receiver_ip)
    else:
        rx_mode_dynamic()


if __name__ == '__main__':
    main()
