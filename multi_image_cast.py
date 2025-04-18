import cv2
import numpy as np
import socket
import struct
import math
import sys
import threading

#!/usr/bin/env python

frames = {}
selected_port = None
ports_list = []


class FrameSegment:
    MAX_DGRAM = 2**16
    MAX_IMAGE_DGRAM = MAX_DGRAM - 64

    def __init__(self, sock, dest_ip, port):
        self.sock = sock
        self.target = (dest_ip, port)

    def udp_frame(self, img):
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]
        jpg = cv2.imencode('.jpg', img, encode_param)[1].tobytes()
        size = len(jpg)
        parts = math.ceil(size/self.MAX_IMAGE_DGRAM)
        idx = 0
        while parts:
            chunk = jpg[idx:idx+self.MAX_IMAGE_DGRAM]
            packet = struct.pack('B', parts) + chunk
            self.sock.sendto(packet, self.target)
            idx += self.MAX_IMAGE_DGRAM
            parts -= 1


def dump_buffer(s):
    # flush old packets until we get a part==1
    while True:
        seg, _ = s.recvfrom(2**16)
        if seg and seg[0] == 1:
            break


def receiver_thread(port):
    global frames
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('0.0.0.0', port))
    dump_buffer(s)
    buf = b''
    while True:
        seg, _ = s.recvfrom(2**16)
        if not seg:
            break
        cnt = seg[0]
        buf += seg[1:]
        if cnt == 1:
            img = cv2.imdecode(np.frombuffer(buf, np.uint8), cv2.IMREAD_COLOR)
            if img is not None:
                frames[port] = img
            buf = b''
    s.close()


def dashboard_mouse_callback(event, x, y, _flags, param):
    global selected_port, ports_list
    if event == cv2.EVENT_LBUTTONDOWN and x >= 900:
        thumb_h = param['height'] // len(ports_list)
        idx = y // thumb_h
        if idx < len(ports_list):
            selected_port = ports_list[idx]


def main():
    if len(sys.argv) < 3 or sys.argv[1] not in ('tx', 'rx'):
        print("Usage:\n"
              "  tx mode: multi_image_cast.py tx [camera_idx] [dest_ip] [port]\n"
              "  rx mode: multi_image_cast.py rx [port1,port2,...]\n")
        sys.exit(1)

    mode = sys.argv[1]
    if mode == 'tx':
        cam_idx = int(sys.argv[2])
        dst_ip = sys.argv[3]
        dst_port = int(sys.argv[4])
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        fs = FrameSegment(sock, dst_ip, dst_port)
        cap = cv2.VideoCapture(cam_idx)
        if not cap.isOpened():
            print("Cannot open camera")
            sys.exit(1)
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            fs.udp_frame(frame)
        cap.release()
        sock.close()

    else:  # rx
        global ports_list, selected_port
        ports_list = list(map(int, sys.argv[2].split(',')))
        for p in ports_list:
            t = threading.Thread(target=receiver_thread,
                                 args=(p,), daemon=True)
            t.start()
        selected_port = ports_list[0] if ports_list else None

        _W, H = 1200, 800
        cv2.namedWindow('Dashboard')
        cv2.setMouseCallback('Dashboard', dashboard_mouse_callback,
                             param={'height': H})

        while True:
            if selected_port in frames:
                focus = cv2.resize(frames[selected_port], (900, H))
            else:
                focus = np.zeros((H, 900, 3), np.uint8)

            thumbs = []
            th = H // len(ports_list) if ports_list else H
            for p in ports_list:
                img = frames.get(p, np.zeros((th, 300, 3), np.uint8))
                timg = cv2.resize(img, (300, th))
                cv2.putText(timg, f"Port {p}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                thumbs.append(timg)

            right = np.vstack(thumbs) if thumbs else np.zeros(
                (H, 300, 3), np.uint8)
            # ensure right pane matches height H
            h_right = right.shape[0]
            if h_right < H:
                pad = np.zeros((H - h_right, right.shape[1], 3), np.uint8)
                right = np.vstack((right, pad))
            elif h_right > H:
                right = right[:H]
            dash = np.hstack((focus, right))
            cv2.imshow('Dashboard', dash)
            if cv2.waitKey(30) == 27:
                break

        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
