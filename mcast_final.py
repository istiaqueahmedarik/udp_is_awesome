#!/usr/bin/env python
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

UDP_IP = "239.0.0.16"
BASE_UDP_PORT = 1234
MAX_CAMERAS = 10

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
mreq = struct.pack("4sl", socket.inet_aton(UDP_IP), socket.INADDR_ANY)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

camera_streams = {}  # { device_node: {'port': udp_port, 'thread': thread} }

# For RX mode: shared frames and timestamps per UDP port
frames = {}        # { port: image (numpy array) }
last_update = {}   # { port: timestamp }
selected_port = None
# RX dynamic ports are assumed to be in this range
dynamic_ports = list(range(BASE_UDP_PORT, BASE_UDP_PORT + MAX_CAMERAS))

###################################
# UDP FrameSegment Class
###################################


class FrameSegment(object):
    MAX_DGRAM = 2**16
    MAX_IMAGE_DGRAM = MAX_DGRAM - 64

    def __init__(self, sock, port, addr=UDP_IP):
        self.s = sock
        self.port = port
        self.addr = addr

    def udp_frame(self, img):
        quality = 100
        params = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
        compress_img = cv2.imencode('.jpg', img, params)[1]
        dat = compress_img.tobytes()
        size = len(dat)
        count = math.ceil(size / self.MAX_IMAGE_DGRAM)
        array_pos_start = 0
        while count:
            array_pos_end = min(size, array_pos_start + self.MAX_IMAGE_DGRAM)
            segment = struct.pack("B", count) + \
                dat[array_pos_start:array_pos_end]
            self.s.sendto(segment, (self.addr, self.port))
            array_pos_start = array_pos_end
            count -= 1

###################################
# TX Mode Functions
###################################


def stream_camera(device_node, udp_port):
    print(f"Starting stream for {device_node} on UDP port {udp_port}")
    cap = cv2.VideoCapture(device_node)
    if not cap.isOpened():
        print(f"Cannot open camera {device_node}")
        return
    fs = FrameSegment(sock, udp_port)
    while True:
        ret, frame = cap.read()
        if not ret:
            print(f"Lost stream for {device_node}")
            break
        fs.udp_frame(frame)
        # time.sleep(0.03)
    cap.release()
    print(f"Stopped stream for {device_node}")


def detect_existing_cameras():
    devices = glob.glob("/dev/video*")
    return devices


def add_camera(device_node):
    if device_node in camera_streams:
        return
    udp_port = BASE_UDP_PORT + len(camera_streams)
    t = threading.Thread(target=stream_camera, args=(
        device_node, udp_port), daemon=True)
    camera_streams[device_node] = {'port': udp_port, 'thread': t}
    print(f"Adding camera {device_node} on port {udp_port}")
    t.start()


def remove_camera(device_node):
    if device_node in camera_streams:
        print(f"Removing camera {device_node}")
        del camera_streams[device_node]


def monitor_new_cameras():
    context = pyudev.Context()
    monitor = pyudev.Monitor.from_netlink(context)
    monitor.filter_by(subsystem='video4linux')
    print("Monitoring for new camera add/remove events...")
    for device in iter(monitor.poll, None):
        if device.action == 'add':
            dev_node = device.device_node
            print(f"New camera detected: {dev_node}")
            add_camera(dev_node)
        elif device.action == 'remove':
            dev_node = device.device_node
            print(f"Camera removed: {dev_node}")
            remove_camera(dev_node)


def tx_mode():
    existing = detect_existing_cameras()
    print("Existing cameras:", existing)
    for dev in existing:
        add_camera(dev)
    monitor_thread = threading.Thread(target=monitor_new_cameras, daemon=True)
    monitor_thread.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopping TX mode.")
        sock.close()

###################################
# RX Mode Functions (Dynamic)
###################################


def dump_buffer(s):
    MAX_DGRAM = 2**16
    while True:
        seg, addr = s.recvfrom(MAX_DGRAM)
        if struct.unpack("B", seg[0:1])[0] == 1:
            break


def receiver_thread(port):
    global frames, last_update
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    mreq = struct.pack("4sl", socket.inet_aton(UDP_IP), socket.INADDR_ANY)
    s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    try:
        s.bind((UDP_IP, port))
    except Exception as e:
        print(f"Could not bind to port {port}: {e}")
        return
    dump_buffer(s)
    MAX_DGRAM = 2**16
    dat = b''
    while True:
        try:
            seg, addr = s.recvfrom(MAX_DGRAM)
        except Exception:
            break
        header = struct.unpack("B", seg[0:1])[0]
        if header > 1:
            dat += seg[1:]
        else:
            dat += seg[1:]
            img = cv2.imdecode(np.frombuffer(dat, dtype=np.uint8), 1)
            if img is not None:
                frames[port] = img
                last_update[port] = time.time()
            dat = b''
    s.close()


def dashboard_mouse_callback(event, x, y, flags, param):
    global selected_port, ports_list
    dashboard_height = param['height']
    if event == cv2.EVENT_LBUTTONDOWN and x >= 900:
        if not ports_list:  # Prevent division by zero
            return
        thumb_height = dashboard_height // len(ports_list)
        index = y // thumb_height
        if index < len(ports_list):
            selected_port = ports_list[index]


def rx_mode_dynamic():
    global selected_port, ports_list, frames, last_update
    for port in dynamic_ports:
        t = threading.Thread(target=receiver_thread, args=(port,), daemon=True)
        t.start()
    selected_port = None
    dash_width, dash_height = 1200, 800
    cv2.namedWindow('Dashboard')
    cv2.setMouseCallback('Dashboard', dashboard_mouse_callback, param={
                         'height': dash_height})
    TIMEOUT = 3

    while True:
        now = time.time()
        ports_list = [port for port in dynamic_ports if port in last_update and (
            now - last_update[port]) < TIMEOUT]
        if selected_port not in ports_list and ports_list:
            selected_port = ports_list[0]
        if selected_port in frames:
            focus = cv2.resize(frames[selected_port],
                               (dash_width, dash_height))
        else:
            focus = np.zeros((dash_height, dash_width, 3), dtype=np.uint8)
        thumbnails = []
        thumb_h = dash_height // (len(ports_list) if ports_list else 1)
        for port in ports_list:
            if port in frames:
                thumb = cv2.resize(frames[port], (300, thumb_h))
            else:
                thumb = np.zeros((thumb_h, 300, 3), dtype=np.uint8)
            cv2.putText(thumb, f"Port {port}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            thumbnails.append(thumb)
        if thumbnails:
            right_panel = np.vstack(thumbnails)
            right_panel = cv2.resize(right_panel, (300, dash_height))
        else:
            right_panel = np.zeros((dash_height, 300, 3), dtype=np.uint8)
        dashboard = np.hstack((focus, right_panel))
        cv2.imshow('Dashboard', dashboard)
        if cv2.waitKey(30) & 0xFF == 27:
            break
    cv2.destroyAllWindows()

###################################
# Main entry point
###################################


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ["tx", "rx"]:
        print("Usage:")
        print("  TX mode: script.py tx")
        print("  RX mode (dynamic): script.py rx")
        sys.exit(1)
    if sys.argv[1] == "tx":
        tx_mode()
    elif sys.argv[1] == "rx":
        rx_mode_dynamic()


if __name__ == '__main__':
    main()
