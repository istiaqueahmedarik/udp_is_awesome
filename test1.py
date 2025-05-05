#!/usr/bin/env python
import cv2
import threading
import time
import sys
import pyudev

# Multicast settings
UDP_GROUP = "239.0.0.16"
BASE_UDP_PORT = 1234
MAX_CAMERAS = 10

# GStreamer pipeline templates
GST_TX_PIPE = (
    'appsrc ! videoconvert ! video/x-raw,format=I420 ! '
    'x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! '
    'rtph264pay config-interval=1 pt=96 ! '
    'udpsink host={group} port={port} auto-multicast=true'
)
GST_RX_PIPE = (
    'udpsrc multicast-group={group} port={port} caps="application/x-rtp,media=video,'
    'encoding-name=H264,payload=96" ! '
    'rtpjitterbuffer latency=100 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink'
)

# Shared state
frames = {}
tx_lock = threading.Lock()
assigned_ports = {}  # device_node -> port
used_ports = set()


def get_next_port():
    for i in range(MAX_CAMERAS):
        port = BASE_UDP_PORT + i
        if port not in used_ports:
            return port
    return None


def start_tx_stream(device_node):
    port = get_next_port()
    if port is None:
        print(f"[TX] No available port for {device_node}")
        return
    with tx_lock:
        used_ports.add(port)
        assigned_ports[device_node] = port

    print(f"[TX] {device_node} → udp://{UDP_GROUP}:{port}")
    cap = cv2.VideoCapture(device_node, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"[TX] Cannot open {device_node}, releasing port {port}")
        with tx_lock:
            used_ports.remove(port)
            del assigned_ports[device_node]
        return
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    pipeline = GST_TX_PIPE.format(group=UDP_GROUP, port=port)
    writer = cv2.VideoWriter(pipeline, cv2.CAP_GSTREAMER, 0, fps, (width, height))
    if not writer.isOpened():
        print(f"[TX] GStreamer failed for {device_node}, releasing port {port}")
        cap.release()
        with tx_lock:
            used_ports.remove(port)
            del assigned_ports[device_node]
        return
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            writer.write(frame)
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        writer.release()
        print(f"[TX] Stopped {device_node}, freeing port {port}")
        with tx_lock:
            used_ports.remove(port)
            del assigned_ports[device_node]


def monitor_cameras():
    context = pyudev.Context()
    monitor = pyudev.Monitor.from_netlink(context)
    monitor.filter_by('video4linux')
    for device in monitor:
        if device.action == 'add' and device.device_node:
            start_tx_stream(device.device_node)
        elif device.action == 'remove' and device.device_node:
            with tx_lock:
                port = assigned_ports.pop(device.device_node, None)
                if port:
                    used_ports.discard(port)
                    print(f"[TX] Device {device.device_node} removed, freed port {port}")


def tx_mode():
    # Initial scan
    for idx in range(MAX_CAMERAS * 2):
        if len(assigned_ports) >= MAX_CAMERAS:
            break
        device_node = f"/dev/video{idx}"
        start_tx_stream(device_node)
        time.sleep(0.05)
    # Start monitor thread
    t = threading.Thread(target=monitor_cameras, daemon=True)
    t.start()
    print(f"[TX] Active streams: {assigned_ports}. Ctrl+C to exit.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("[TX] Exiting.")


def rx_stream(port):
    pipeline = GST_RX_PIPE.format(group=UDP_GROUP, port=port)
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        return
    print(f"[RX] Listening udp://{UDP_GROUP}:{port}")
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            with tx_lock:
                frames[port] = frame
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()


def rx_mode():
    threads = []
    for i in range(MAX_CAMERAS):
        port = BASE_UDP_PORT + i
        t = threading.Thread(target=rx_stream, args=(port,), daemon=True)
        threads.append(t)
        t.start()
        time.sleep(0.05)
    print(f"[RX] Ready to receive on ports {BASE_UDP_PORT}-{BASE_UDP_PORT+MAX_CAMERAS-1}. ESC to exit.")
    win = 'Multicam RX'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    try:
        while True:
            with tx_lock:
                data = list(frames.items())
            for port, img in data:
                cv2.imshow(f"Port {port}", img)
            if cv2.waitKey(30) == 27:
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()


def main():
    if len(sys.argv)!=2 or sys.argv[1] not in ('tx','rx'):
        print(f"Usage: {sys.argv[0]} tx|rx")
        sys.exit(1)
    if sys.argv[1]=='tx':
        tx_mode()
    else:
        rx_mode()

if __name__=='__main__':
    main()

