#!/usr/bin/env python
from kivy.graphics.texture import Texture
from kivy.core.window import Window
from kivy.uix.gridlayout import GridLayout
from kivy.uix.scrollview import ScrollView
from kivy.uix.label import Label
from kivy.uix.image import Image as KivyImage
from kivy.uix.boxlayout import BoxLayout
from kivy.clock import Clock
from kivy.app import App
from kivy.graphics import RoundedRectangle, Color, Rectangle, StencilPush, StencilUse, StencilUnUse, StencilPop
from kivy.uix.widget import Widget
import cv2
import numpy as np
import socket
import struct
import math
import sys
import time
import threading
import pyudev

# UDP and stream settings (should match TX mode)
UDP_IP = "239.0.0.16"
BASE_UDP_PORT = 12345   # starting UDP port for streams
MAX_CAMERAS = 10         # expected max streams
TIMEOUT = 3             # seconds without update to consider stream dead
dynamic_ports = list(range(BASE_UDP_PORT, BASE_UDP_PORT + MAX_CAMERAS))

# Global dictionaries for received frames and their last update time
frames = {}       # { port: latest image (numpy array) }
last_update = {}  # { port: timestamp }
# Selected port for focus display
selected_port = None

#############################
# UDP Receiver Functions
#############################


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


# Start receiver threads for all dynamic ports
for port in dynamic_ports:
    t = threading.Thread(target=receiver_thread, args=(port,), daemon=True)
    t.start()

#############################
# Helper: Letterbox Scaling
#############################


def letterbox_image(img, target_width, target_height):
    """Resize 'img' to fit into (target_width, target_height) while preserving aspect ratio.
       The result is padded with black pixels."""
    h, w = img.shape[:2]
    scale = min(target_width / w, target_height / h)
    new_w = int(w * scale)
    new_h = int(h * scale)
    resized = cv2.resize(img, (new_w, new_h))
    # Compute padding
    top = (target_height - new_h) // 2
    bottom = target_height - new_h - top
    left = (target_width - new_w) // 2
    right = target_width - new_w - left
    padded = cv2.copyMakeBorder(
        resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=[0, 0, 0])
    return padded


#############################
# Custom Widget: RoundedImage
#############################


class RoundedImage(Widget):
    def __init__(self, radius=20, **kwargs):
        super(RoundedImage, self).__init__(**kwargs)
        self.radius = radius
        self.texture = None
        with self.canvas:
            # Use stencil instructions to clip the image to a rounded rectangle
            StencilPush()
            self.stencil_rect = RoundedRectangle(
                pos=self.pos, size=self.size, radius=[self.radius])
            StencilUse()
            self.rect = Rectangle(pos=self.pos, size=self.size)
            StencilUnUse()
            StencilPop()
        self.bind(pos=self.update_rect, size=self.update_rect)

    def update_rect(self, *args):
        self.stencil_rect.pos = self.pos
        self.stencil_rect.size = self.size
        self.rect.pos = self.pos
        self.rect.size = self.size
        if self.texture:
            self.rect.texture = self.texture

    def set_texture(self, texture):
        self.texture = texture
        self.rect.texture = texture


#############################
# Kivy Dashboard UI Classes
#############################


class Dashboard(BoxLayout):
    def __init__(self, **kwargs):
        super(Dashboard, self).__init__(**kwargs)
        self.orientation = 'vertical'
        # Top label with title
        self.title_label = Label(text="MIST MONGOL BAROTA", size_hint=(1, 0.1),
                                 font_size='24sp', color=(1, 1, 1, 1), bold=True)
        self.add_widget(self.title_label)
        # Main area: horizontal BoxLayout
        self.main_layout = BoxLayout(
            orientation='horizontal', size_hint=(1, 0.9))
        self.add_widget(self.main_layout)
        # Left: Focus view (use a KivyImage)
        self.focus_view = KivyImage(size_hint=(0.75, 1))
        self.main_layout.add_widget(self.focus_view)
        # Right: Scrollable thumbnails in a grid layout
        self.thumb_layout = GridLayout(cols=1, spacing=5, size_hint_y=None)
        self.thumb_layout.bind(
            minimum_height=self.thumb_layout.setter('height'))
        self.scrollview = ScrollView(size_hint=(0.25, 1))
        self.scrollview.add_widget(self.thumb_layout)
        self.main_layout.add_widget(self.scrollview)
        # Dynamic list of active ports and selected port
        self.active_ports = []
        self.selected_port = None
        # Schedule periodic update (30 ms)
        Clock.schedule_interval(self.update_dashboard, 0.03)

    def update_dashboard(self, dt):
        now = time.time()
        # Build active ports list based on last update timestamp
        self.active_ports = [port for port in dynamic_ports if port in last_update and (
            now - last_update[port]) < TIMEOUT]
        if self.selected_port not in self.active_ports and self.active_ports:
            self.selected_port = self.active_ports[0]
        # Prepare focus image (900xWindow.height)
        target_focus_width = int(Window.width)
        target_focus_height = int(Window.height)
        if self.selected_port in frames:
            img = frames[self.selected_port]
            # Use letterbox scaling to preserve aspect ratio
            focus_img = letterbox_image(cv2.cvtColor(
                img, cv2.COLOR_BGR2RGB), target_focus_width, target_focus_height)
        else:
            # Create blank image and add text "Run receiving script"
            focus_img = np.zeros(
                (target_focus_height, target_focus_width, 3), dtype=np.uint8)
            cv2.putText(focus_img, "Run receiving script", (50, target_focus_height//2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 3)
        # Create texture for focus view
        buf = focus_img.tobytes()
        texture = Texture.create(
            size=(focus_img.shape[1], focus_img.shape[0]), colorfmt='rgb')
        texture.blit_buffer(buf, colorfmt='rgb', bufferfmt='ubyte')
        texture.flip_vertical()
        self.focus_view.texture = texture

        # Update thumbnails
        self.thumb_layout.clear_widgets()
        num = len(self.active_ports) if self.active_ports else 1
        thumb_target_height = int(Window.height / num)
        for port in self.active_ports:
            if port in frames:
                thumb_img = letterbox_image(cv2.cvtColor(
                    frames[port], cv2.COLOR_BGR2RGB), 300, thumb_target_height)
            else:
                thumb_img = np.zeros(
                    (thumb_target_height, 300, 3), dtype=np.uint8)
            # Create texture for thumbnail
            buf_thumb = thumb_img.tobytes()
            tex_thumb = Texture.create(
                size=(thumb_img.shape[1], thumb_img.shape[0]), colorfmt='rgb')
            tex_thumb.blit_buffer(buf_thumb, colorfmt='rgb', bufferfmt='ubyte')
            tex_thumb.flip_vertical()
            # Create a custom RoundedImage widget for the thumbnail
            thumb_widget = RoundedImage(
                radius=20, size_hint_y=None, height=thumb_target_height)
            thumb_widget.set_texture(tex_thumb)
            # Bind touch event to change focus stream
            thumb_widget.bind(on_touch_down=lambda inst, touch,
                              p=port: self.on_thumb_touch(inst, touch, p))
            self.thumb_layout.add_widget(thumb_widget)

    def on_thumb_touch(self, instance, touch, port):
        if instance.collide_point(*touch.pos):
            self.selected_port = port
            print(f"Selected port: {port}")


class DashboardApp(App):
    def build(self):
        Window.size = (1200, 800)
        return Dashboard()


if __name__ == '__main__':
    DashboardApp().run()
