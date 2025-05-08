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

DEFAULT_UDP_IP = "192.168.1.8"
UDP_IP = DEFAULT_UDP_IP  # Will be overridden if parameter provided
BASE_UDP_PORT = 1234
MAX_CAMERAS = 10

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
# Removed multicast configuration

camera_streams = {}

frames = {}
last_update = {}
selected_port = None

# Port management
available_ports = [BASE_UDP_PORT + i for i in range(MAX_CAMERAS)]
used_ports = {}  # device_node -> port


class FrameSegment(object):
    MAX_DGRAM = 2**16
    MAX_IMAGE_DGRAM = MAX_DGRAM - 64

    def __init__(self, sock, port, addr=None):
        self.s = sock
        self.port = port
        self.addr = UDP_IP if addr is None else addr

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


def get_next_available_port():
    if available_ports:
        return available_ports.pop(0)
    return None


def release_port(port):
    if port is not None and port not in available_ports:
        available_ports.append(port)
        available_ports.sort()


def stream_camera(device_node, udp_port):
    print(f"Starting stream for {device_node} on UDP port {udp_port}")
    cap = cv2.VideoCapture(device_node, cv2.CAP_V4L2)

    # Configure for low CPU usage
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(
        'M', 'J', 'P', 'G'))  # Use MJPEG
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # Low resolution width
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # Low resolution height
    cap.set(cv2.CAP_PROP_FPS, 8)  # Set to 15fps
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


def try_add_camera(index):
    device_node = f"/dev/video{index}"
    if device_node in camera_streams:
        return
    udp_port = get_next_available_port()
    if udp_port is None:
        print("No available UDP ports for new camera.")
        return
    try:
        cap = cv2.VideoCapture(device_node, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        cap.set(cv2.CAP_PROP_FPS, 10)
        if not cap.isOpened():
            release_port(udp_port)
            return  # Device does not exist or cannot be opened
        cap.release()
    except Exception:
        release_port(udp_port)
        return  # Device cannot be opened
    t = threading.Thread(target=stream_camera, args=(
        device_node, udp_port), daemon=True)
    camera_streams[device_node] = {
        'port': udp_port, 'thread': t, 'index': index}
    used_ports[device_node] = udp_port
    print(f"Adding camera {device_node} on port {udp_port}")
    t.start()


def remove_camera_by_index(index):
    device_node = f"/dev/video{index}"
    if device_node in camera_streams:
        print(f"Removing camera {device_node}")
        port = camera_streams[device_node]['port']
        release_port(port)
        used_ports.pop(device_node, None)
        del camera_streams[device_node]


def monitor_new_cameras():
    context = pyudev.Context()
    monitor = pyudev.Monitor.from_netlink(context)
    monitor.filter_by(subsystem='video4linux')
    print("Monitoring for new camera add/remove events...")
    for device in iter(monitor.poll, None):
        if device.action == 'add':
            # Extract index from device_node
            dev_node = device.device_node
            if dev_node.startswith("/dev/video"):
                try:
                    idx = int(dev_node.replace("/dev/video", ""))
                    try_add_camera(idx)
                except Exception:
                    pass
        elif device.action == 'remove':
            dev_node = device.device_node
            if dev_node.startswith("/dev/video"):
                try:
                    idx = int(dev_node.replace("/dev/video", ""))
                    remove_camera_by_index(idx)
                except Exception:
                    pass


def tx_mode():
    # Try all possible indices
    for idx in range(MAX_CAMERAS * 2):
        try_add_camera(idx)
    monitor_thread = threading.Thread(target=monitor_new_cameras, daemon=True)
    monitor_thread.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopping TX mode.")
        sock.close()


def dump_buffer(s):
    MAX_DGRAM = 2**16
    while True:
        seg, addr = s.recvfrom(MAX_DGRAM)
        if struct.unpack("B", seg[0:1])[0] == 1:
            break


def receiver_thread(port, idx):
    global frames, last_update
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # Removed multicast configuration
    try:
        s.bind(('', port))  # Bind to all interfaces for receiving
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
                frames[idx] = img
                last_update[idx] = time.time()
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
    # Discover active ports by listening for a short period
    port_threads = []
    port_idx_map = {}
    for i in range(MAX_CAMERAS):
        port = BASE_UDP_PORT + i
        t = threading.Thread(target=receiver_thread,
                             args=(port, i), daemon=True)
        t.start()
        port_threads.append(t)
        port_idx_map[port] = i
    selected_port = None
    dash_width, dash_height = 1200, 800
    cv2.namedWindow('Dashboard')
    cv2.setMouseCallback('Dashboard', dashboard_mouse_callback, param={
                         'height': dash_height})
    TIMEOUT = 3

    while True:
        now = time.time()
        ports_list = [i for i in range(MAX_CAMERAS) if i in last_update and (
            now - last_update[i]) < TIMEOUT]
        if selected_port not in ports_list and ports_list:
            selected_port = ports_list[0]
        if selected_port in frames:
            focus = cv2.resize(frames[selected_port],
                               (dash_width, dash_height))
        else:
            focus = np.zeros((dash_height, dash_width, 3), dtype=np.uint8)
        thumbnails = []
        thumb_h = dash_height // (len(ports_list) if ports_list else 1)
        for idx in ports_list:
            if idx in frames:
                thumb = cv2.resize(frames[idx], (300, thumb_h))
            else:
                thumb = np.zeros((thumb_h, 300, 3), dtype=np.uint8)
            cv2.putText(thumb, f"Index {idx}", (10, 30),
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


def main():
    global UDP_IP

    # Parse command line args
    if len(sys.argv) < 2 or sys.argv[1] not in ["tx", "rx"]:
        print("Usage:")
        print("  TX mode: script.py tx [ip_address]")
        print("  RX mode (dynamic): script.py rx [ip_address]")
        sys.exit(1)

    # Check for IP address parameter
    if len(sys.argv) > 2:
        UDP_IP = sys.argv[2]
    else:
        UDP_IP = DEFAULT_UDP_IP

    print(f"Using IP address: {UDP_IP}")

    if sys.argv[1] == "tx":
        tx_mode()
    elif sys.argv[1] == "rx":
        rx_mode_dynamic()


if __name__ == '__main__':
    main()
