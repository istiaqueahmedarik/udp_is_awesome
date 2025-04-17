import cv2
import numpy as np
import socket
import struct
import time
import threading
import argparse

#!/usr/bin/env python3

BASE_UDP_PORT = 1234
MAX_CAMERAS = 10
frames = {}
last_update = {}
dynamic_ports = list(range(BASE_UDP_PORT, BASE_UDP_PORT + MAX_CAMERAS))


def dump_buffer(sock):
    """Flush out any partial frames."""
    max_dgram = 2 ** 16
    while True:
        seg, _ = sock.recvfrom(max_dgram)
        if struct.unpack("B", seg[:1])[0] == 1:
            break


def receiver_thread(port):
    """Listen on a UDP port and reconstruct JPEG frames."""
    global frames, last_update
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", port))
    dump_buffer(sock)
    max_dgram = 2 ** 16
    buffer = b""
    while True:
        seg, _ = sock.recvfrom(max_dgram)
        header = struct.unpack("B", seg[:1])[0]
        buffer += seg[1:]
        if header == 1:
            img = cv2.imdecode(np.frombuffer(buffer, dtype=np.uint8), 1)
            if img is not None:
                frames[port] = img
                last_update[port] = time.time()
            buffer = b""
    # sock.close()  # never reached


def dashboard_mouse_callback(event, x, y, flags, param):
    """Select a camera feed by clicking its thumbnail."""
    global selected_port, ports_list
    if event == cv2.EVENT_LBUTTONDOWN and ports_list:
        h = param["height"]
        slot = y // (h // len(ports_list))
        if 0 <= slot < len(ports_list):
            selected_port = ports_list[slot]


def rx_mode_dynamic(width=1200, height=800, timeout=3.0):
    """Launch receiver threads and display a dashboard window."""
    global selected_port, ports_list
    for port in dynamic_ports:
        t = threading.Thread(target=receiver_thread, args=(port,), daemon=True)
        t.start()

    selected_port = None
    cv2.namedWindow("Dashboard", cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(
        "Dashboard", dashboard_mouse_callback, {"height": height})

    while True:
        now = time.time()
        ports_list = [
            p for p in dynamic_ports
            if p in last_update and (now - last_update[p]) < timeout
        ]
        if ports_list and selected_port not in ports_list:
            selected_port = ports_list[0]

        # Focus view
        if selected_port in frames:
            focus = cv2.resize(frames[selected_port], (width, height))
        else:
            focus = np.zeros((height, width, 3), np.uint8)

        # Thumbnails
        thumbs = []
        slot_h = height // max(1, len(ports_list))
        for p in ports_list:
            if p in frames:
                thumb = cv2.resize(frames[p], (300, slot_h))
            else:
                thumb = np.zeros((slot_h, 300, 3), np.uint8)
            cv2.putText(thumb, f"Port {p}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            thumbs.append(thumb)

        if thumbs:
            panel = np.vstack(thumbs)
            side = cv2.resize(panel, (300, height))
        else:
            side = np.zeros((height, 300, 3), np.uint8)

        dashboard = np.hstack((focus, side))
        cv2.imshow("Dashboard", dashboard)
        if cv2.waitKey(30) & 0xFF == 27:
            break

    cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="UDP camera stream receiver")
    parser.add_argument("--base-port", type=int, default=BASE_UDP_PORT,
                        help="starting UDP port")
    parser.add_argument("--max-cams", type=int, default=MAX_CAMERAS,
                        help="number of ports to listen on")
    parser.add_argument("--width", type=int, default=1200,
                        help="dashboard focus width")
    parser.add_argument("--height", type=int, default=800,
                        help="dashboard focus height")
    args = parser.parse_args()

    global dynamic_ports
    dynamic_ports = list(range(args.base_port, args.base_port + args.max_cams))

    rx_mode_dynamic(width=args.width, height=args.height)


if __name__ == "__main__":
    main()
