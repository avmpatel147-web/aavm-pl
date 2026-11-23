# AVM Plotter Android Edition (Kivy + usbserial4a)
# Z-axis: Z1 = Pen UP, Z0 = Pen DOWN (1 mm travel)
# Coordinates: X0 Y0 origin, positive +X / +Y

import os
import re
import math
import time
import bisect
import threading
import json

from kivy.app import App
from kivy.clock import Clock
from kivy.graphics import Color, Line, Rectangle, Ellipse
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.gridlayout import GridLayout
from kivy.uix.scrollview import ScrollView
from kivy.uix.label import Label
from kivy.uix.button import Button
from kivy.uix.textinput import TextInput
from kivy.uix.spinner import Spinner
from kivy.uix.progressbar import ProgressBar
from kivy.uix.widget import Widget
from kivy.uix.popup import Popup
from kivy.core.window import Window

IS_ANDROID = False
try:
    from usb4a import usb
    from usbserial4a import serial4a
    IS_ANDROID = True
except ImportError:
    usb = None
    serial4a = None

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

BAUD_RATE = 115200
_NUM = r"(-?(?:\d+(?:\.\d*)?|\.\d+))"
AXIS_XY_RE = re.compile(r"([XY])\s*" + _NUM)
Z_WORD_RE = re.compile(r"(?:^|\s)Z\s*" + _NUM)


class PlotterPreviewCanvas(Widget):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.machine_w = 210.0
        self.machine_h = 297.0
        self.segments = []
        self.points = []
        self.progress_idx = 0
        self.live_pos = {"X": None, "Y": None, "Z": None}
        self.zoom = 1.0
        self.bind(pos=self.redraw, size=self.redraw)

    def set_data(self, segments, points, machine_w=None, machine_h=None):
        self.segments = segments
        self.points = points
        if machine_w:
            self.machine_w = float(machine_w)
        if machine_h:
            self.machine_h = float(machine_h)
        self.progress_idx = 0
        self.redraw()

    def set_progress(self, idx):
        self.progress_idx = idx
        self.redraw()

    def set_live(self, pos_dict):
        self.live_pos = pos_dict
        self.redraw()

    def redraw(self, *args):
        self.canvas.clear()
        w, h = max(10, self.width), max(10, self.height)
        pad = 20.0
        xmin, ymin, xmax, ymax = 0.0, 0.0, self.machine_w, self.machine_h

        if not self.machine_w and self.points:
            xs = [p[0] for p in self.points]
            ys = [p[1] for p in self.points]
            xmax = max(10.0, max(xs))
            ymax = max(10.0, max(ys))

        sx = (w - 2 * pad) / max(1e-6, xmax - xmin)
        sy = (h - 2 * pad) / max(1e-6, ymax - ymin)
        scale = min(sx, sy) * self.zoom
        bed_w = (xmax - xmin) * scale
        bed_h = (ymax - ymin) * scale
        left = self.x + (w - bed_w) / 2.0
        bottom = self.y + (h - bed_h) / 2.0

        def to_px(px, py):
            return left + (px - xmin) * scale, bottom + (py - ymin) * scale

        with self.canvas:
            Color(1.0, 0.99, 0.97, 1)
            Rectangle(pos=self.pos, size=self.size)

            Color(0.45, 0.45, 0.45, 1)
            Line(rectangle=(left, bottom, bed_w, bed_h), width=1.4)

            if xmax >= 210 and ymax >= 297:
                ax2, ay2 = to_px(210, 297)
                Color(0.35, 0.65, 0.95, 0.7)
                Line(rectangle=(left, bottom, ax2 - left, ay2 - bottom), width=1.0)

            done_n = min(self.progress_idx, len(self.segments))
            for idx, (x1, y1, x2, y2, kind, _ln) in enumerate(self.segments):
                p1x, p1y = to_px(x1, y1)
                p2x, p2y = to_px(x2, y2)
                done = idx < done_n
                if kind == "travel":
                    Color(0.3, 0.6, 0.46, 0.6) if done else Color(0.7, 0.7, 0.7, 0.5)
                    Line(points=[p1x, p1y, p2x, p2y], width=1.0)
                else:
                    Color(0.08, 0.52, 0.28, 1.0) if done else Color(0.91, 0.46, 0.07, 1.0)
                    Line(points=[p1x, p1y, p2x, p2y], width=1.4)

            if self.points:
                sx0, sy0 = to_px(*self.points[0])
                Color(0.09, 0.41, 0.67, 1)
                Ellipse(pos=(sx0 - 5, sy0 - 5), size=(10, 10))

            if self.live_pos.get("X") is not None and self.live_pos.get("Y") is not None:
                mx, my = to_px(self.live_pos["X"], self.live_pos["Y"])
                Color(0.84, 0.15, 0.15, 1)
                Ellipse(pos=(mx - 6, my - 6), size=(12, 12))


class AVMPlotterAndroidApp(App):
    def build(self):
        Window.clearcolor = (1.0, 0.957, 0.902, 1)
        self.title = "AVM Plotter Android"

        self.ser = None
        self.serial_lock = threading.RLock()
        self.stop_sender = threading.Event()
        self.graceful_stop = threading.Event()
        self.pause_sender = threading.Event()
        self.sender_busy = False
        self.gcode_lines = []
        self.gcode_path = ""
        self.machine_width = 210.0
        self.machine_height = 297.0
        self.machine_position = {"X": None, "Y": None, "Z": None}
        self.preview_seg_lines = []
        self.feed_override = 100
        self.pending_runtime_z = None
        self.runtime_lock = threading.Lock()
        self.job_abs_mode = True
        self.job_metric = True
        self.job_last_motion = "G1"
        self.job_last_feed = None
        self._last_completed = 0
        self._job_start_idx = 0
        self.checkpoint_path = os.path.join(self.user_data_dir, "avm_grbl_checkpoint.json")

        root = BoxLayout(orientation="vertical", padding=6, spacing=5)

        header = BoxLayout(size_hint_y=None, height=44, spacing=4)
        self.status_lbl = Label(text="DISCONNECTED", bold=True, color=(0.74, 0.19, 0.19, 1), size_hint_x=0.45)
        btn_unlock = Button(text="UNLOCK ($X)", size_hint_x=0.25, on_release=lambda *_: self.unlock_alarm())
        btn_estop = Button(text="E-STOP", bold=True, background_color=(0.75, 0, 0.12, 1),
                           size_hint_x=0.30, on_release=lambda *_: self.emergency_stop())
        header.add_widget(self.status_lbl)
        header.add_widget(btn_unlock)
        header.add_widget(btn_estop)
        root.add_widget(header)

        usb_bar = BoxLayout(size_hint_y=None, height=42, spacing=4)
        self.port_spinner = Spinner(text="Select USB", values=[], size_hint_x=0.44)
        btn_ref = Button(text="Refresh", size_hint_x=0.18, on_release=lambda *_: self.refresh_ports())
        btn_conn = Button(text="Connect", size_hint_x=0.19, background_color=(0.09, 0.52, 0.28, 1),
                          on_release=lambda *_: self.connect_usb())
        btn_disc = Button(text="Disc.", size_hint_x=0.19, on_release=lambda *_: self.disconnect_usb())
        usb_bar.add_widget(self.port_spinner)
        usb_bar.add_widget(btn_ref)
        usb_bar.add_widget(btn_conn)
        usb_bar.add_widget(btn_disc)
        root.add_widget(usb_bar)

        self.pos_lbl = Label(text="Pos: X —  Y —  Z —  |  Bed: 210×297 mm",
                             size_hint_y=None, height=26, color=(0.16, 0.13, 0.11, 1), font_size="13sp")
        root.add_widget(self.pos_lbl)

        self.preview = PlotterPreviewCanvas(size_hint_y=0.34)
        root.add_widget(self.preview)

        prog_box = BoxLayout(size_hint_y=None, height=26, spacing=6)
        self.prog_bar = ProgressBar(max=100, value=0, size_hint_x=0.65)
        self.prog_lbl = Label(text="0% (0/0)", size_hint_x=0.35, color=(0.16, 0.13, 0.11, 1), font_size="12sp")
        prog_box.add_widget(self.prog_bar)
        prog_box.add_widget(self.prog_lbl)
        root.add_widget(prog_box)

        scroll = ScrollView(size_hint_y=0.42)
        ctrl = BoxLayout(orientation="vertical", size_hint_y=None, spacing=6, padding=[0, 4])
        ctrl.bind(minimum_height=ctrl.setter("height"))

        jog_row1 = GridLayout(cols=4, size_hint_y=None, height=42, spacing=4)
        self.jog_step = Spinner(text="1", values=["0.1", "0.5", "1", "5", "10"])
        jog_row1.add_widget(self.jog_step)
        jog_row1.add_widget(Button(text="Home ($H)", on_release=lambda *_: self.send_cmd("$H")))
        jog_row1.add_widget(Button(text="Go X0 Y0", on_release=lambda *_: self.go_zero()))
        jog_row1.add_widget(Button(text="Set XYZ=0", on_release=lambda *_: self.set_xyz_zero()))
        ctrl.add_widget(jog_row1)

        jog_row2 = GridLayout(cols=6, size_hint_y=None, height=42, spacing=4)
        jog_row2.add_widget(Button(text="X-", on_release=lambda *_: self.jog("X", -1)))
        jog_row2.add_widget(Button(text="X+", on_release=lambda *_: self.jog("X", 1)))
        jog_row2.add_widget(Button(text="Y-", on_release=lambda *_: self.jog("Y", -1)))
        jog_row2.add_widget(Button(text="Y+", on_release=lambda *_: self.jog("Y", 1)))
        jog_row2.add_widget(Button(text="Z- (Down)", on_release=lambda *_: self.jog("Z", -1)))
        jog_row2.add_widget(Button(text="Z+ (Up)", on_release=lambda *_: self.jog("Z", 1)))
        ctrl.add_widget(jog_row2)

        pen_row = GridLayout(cols=4, size_hint_y=None, height=42, spacing=4)
        pen_row.add_widget(Button(text="Pen UP (Z1)", on_release=lambda *_: self.pen_control(True)))
        pen_row.add_widget(Button(text="Pen DN (Z0)", on_release=lambda *_: self.pen_control(False)))
        pen_row.add_widget(Button(text="Test 20mm Sq", on_release=lambda *_: self.load_test_square(20)))
        pen_row.add_widget(Button(text="Load G-code", on_release=lambda *_: self.open_file_dialog()))
        ctrl.add_widget(pen_row)

        job_row = GridLayout(cols=5, size_hint_y=None, height=44, spacing=4)
        job_row.add_widget(Button(text="PLAY", background_color=(0.09, 0.52, 0.28, 1),
                                  on_release=lambda *_: self.start_job(0)))
        job_row.add_widget(Button(text="PAUSE", on_release=lambda *_: self.pause_job()))
        job_row.add_widget(Button(text="RESUME", on_release=lambda *_: self.resume_job()))
        job_row.add_widget(Button(text="STOP", background_color=(0.74, 0.19, 0.19, 1),
                                  on_release=lambda *_: self.stop_job()))
        job_row.add_widget(Button(text="Chkpt Res.", on_release=lambda *_: self.resume_checkpoint()))
        ctrl.add_widget(job_row)

        ov_row = BoxLayout(size_hint_y=None, height=40, spacing=4)
        ov_row.add_widget(Button(text="Feed -10%", size_hint_x=0.22, on_release=lambda *_: self.feed_ov(-10)))
        ov_row.add_widget(Button(text="Feed +10%", size_hint_x=0.22, on_release=lambda *_: self.feed_ov(10)))
        self.cmd_input = TextInput(hint_text="GRBL cmd (e.g. $$, G0 X10)", multiline=False, size_hint_x=0.38)
        ov_row.add_widget(self.cmd_input)
        ov_row.add_widget(Button(text="Send", size_hint_x=0.18, on_release=lambda *_: self.send_manual()))
        ctrl.add_widget(ov_row)

        self.log_input = TextInput(
            text="AVM Plotter Android Ready.\n", readonly=True, size_hint_y=None, height=120,
            font_size="12sp", background_color=(0.12, 0.11, 0.10, 1), foreground_color=(0.95, 0.95, 0.95, 1)
        )
        ctrl.add_widget(self.log_input)

        scroll.add_widget(ctrl)
        root.add_widget(scroll)

        self.refresh_ports()
        Clock.schedule_interval(self._poll_status, 0.6)
        return root

    def log(self, msg):
        def _append(_dt):
            stamp = time.strftime("%H:%M:%S")
            lines = self.log_input.text.splitlines()[-80:]
            lines.append(f"[{stamp}] {msg}")
            self.log_input.text = "\n".join(lines) + "\n"
            self.log_input.cursor = (0, len(self.log_input.text))
        Clock.schedule_once(_append, 0)

    def show_popup(self, title, message):
        def _open(_dt):
            box = BoxLayout(orientation="vertical", padding=10, spacing=8)
            box.add_widget(Label(text=message, halign="center"))
            btn = Button(text="OK", size_hint_y=None, height=40)
            pop = Popup(title=title, content=box, size_hint=(0.85, 0.4))
            btn.bind(on_release=pop.dismiss)
            box.add_widget(btn)
            pop.open()
        Clock.schedule_once(_open, 0)

    def refresh_ports(self):
        found = []
        if IS_ANDROID and usb is not None:
            for dev in usb.get_usb_device_list():
                found.append(dev.getDeviceName())
        elif list_ports is not None:
            found = [p.device for p in list_ports.comports()]

        self.port_spinner.values = found
        if found:
            self.port_spinner.text = found[0]
            self.log("USB devices: " + ", ".join(found))
        else:
            self.port_spinner.text = "No USB Found"
            self.log("Koi USB device nahi mila. OTG cable check karein.")

    def connect_usb(self):
        dev_name = self.port_spinner.text
        if not dev_name or "No USB" in dev_name or "Select" in dev_name:
            self.refresh_ports()
            return
        self.disconnect_usb(silent=True)

        def _worker():
            try:
                if IS_ANDROID and usb is not None and serial4a is not None:
                    device = usb.get_usb_device(dev_name)
                    if not device:
                        raise RuntimeError("USB device nahi mila.")
                    if not usb.has_usb_permission(device):
                        usb.request_usb_permission(device)
                        time.sleep(1.5)
                    s = serial4a.get_serial_port(dev_name, BAUD_RATE, 8, "N", 1, timeout=0.25)
                elif serial is not None:
                    s = serial.Serial(dev_name, BAUD_RATE, timeout=0.25, write_timeout=1)
                else:
                    raise RuntimeError("Serial driver missing.")
                time.sleep(2.0)
                s.reset_input_buffer()
                self.ser = s
                Clock.schedule_once(lambda *_: setattr(self.status_lbl, "text", "CONNECTED"), 0)
                Clock.schedule_once(lambda *_: setattr(self.status_lbl, "color", (0.09, 0.52, 0.28, 1)), 0)
                self.log(f"Connected to {dev_name} @ {BAUD_RATE}")
                self._read_settings()
            except Exception as exc:
                self.ser = None
                self.log(f"Connect error: {exc}")

        threading.Thread(target=_worker, daemon=True).start()

    def disconnect_usb(self, silent=False):
        with self.serial_lock:
            if self.ser:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
        self.status_lbl.text = "DISCONNECTED"
        self.status_lbl.color = (0.74, 0.19, 0.19, 1)
        if not silent:
            self.log("USB Disconnected.")

    def _read_settings(self):
        try:
            with self.serial_lock:
                if not self.ser or not self.ser.is_open:
                    return
                self.ser.write(b"$$\n")
                self.ser.flush()
                deadline = time.time() + 2.5
                while time.time() < deadline:
                    raw = self.ser.readline()
                    if not raw:
                        continue
                    line = raw.decode("ascii", errors="ignore").strip()
                    mx = re.match(r"\$130=([0-9.]+)", line)
                    my = re.match(r"\$131=([0-9.]+)", line)
                    if mx:
                        self.machine_width = float(mx.group(1))
                    if my:
                        self.machine_height = float(my.group(1))
                    if line.lower() == "ok":
                        break
            Clock.schedule_once(lambda *_: self._update_pos_ui(), 0)
        except Exception as exc:
            self.log(f"Settings read error: {exc}")

    def send_raw(self, data):
        with self.serial_lock:
            if not self.ser or not self.ser.is_open:
                raise RuntimeError("Arduino connected nahi hai.")
            self.ser.write(data)
            self.ser.flush()

    def send_cmd(self, cmd):
        cmd = cmd.strip()
        if not cmd:
            return
        try:
            if cmd in ("!", "~", "?"):
                self.send_raw(cmd.encode("ascii"))
            else:
                self.send_raw((cmd + "\n").encode("ascii", errors="ignore"))
            self.log(f">> {cmd}")
        except Exception as exc:
            self.log(f"Send error: {exc}")

    def send_manual(self):
        c = self.cmd_input.text.strip()
        self.cmd_input.text = ""
        self.send_cmd(c)

    def jog(self, axis, direction):
        if self.sender_busy:
            self.log("Job running; jog blocked.")
            return
        dist = float(self.jog_step.text) * direction
        feed = 300 if axis == "Z" else 700
        self.send_cmd(f"G21 G91 G1 {axis}{dist:g} F{feed}")
        self.send_cmd("G90")

    def pen_control(self, up):
        if self.sender_busy:
            with self.runtime_lock:
                self.pending_runtime_z = (1.0 if up else 0.0, 200)
            self.log(f"Live Z {'UP (Z1)' if up else 'DOWN (Z0)'} queued.")
        else:
            self.send_cmd("G21 G90 G1 Z1 F200" if up else "G21 G90 G1 Z0 F200")

    def go_zero(self):
        if not self.sender_busy:
            self.send_cmd("G21 G90 G0 Z1")
            self.send_cmd("G90 G0 X0 Y0")

    def set_xyz_zero(self):
        if not self.sender_busy:
            self.send_cmd("G10 L20 P1 X0 Y0 Z0")
            self.log("Work Zero X0 Y0 Z0 set.")

    def unlock_alarm(self):
        self.send_cmd("$X")

    def emergency_stop(self):
        self.stop_sender.set()
        self.graceful_stop.set()
        self.pause_sender.clear()
        try:
            self.send_raw(b"!")
            time.sleep(0.1)
            self.send_raw(bytes([0x18]))
            self.log("EMERGENCY STOP: Feed Hold + Soft Reset sent!")
        except Exception as exc:
            self.log(f"E-Stop error: {exc}")

    def feed_ov(self, delta):
        try:
            self.send_raw(b"\x91" if delta > 0 else b"\x92")
            self.feed_override = max(10, min(200, self.feed_override + delta))
            self.log(f"Feed override: {self.feed_override}%")
        except Exception as exc:
            self.log(f"Override error: {exc}")

    @staticmethod
    def parse_gcode(raw_lines):
        out = []
        for raw in raw_lines:
            line = raw.strip()
            if not line or line.startswith((";", "(", "%")):
                continue
            line = line.split(";")[0]
            line = re.sub(r"\([^)]*\)", "", line).strip().upper()
            if line:
                line = re.sub(r"\bG0([0-3])\b", r"G\1", line)
                out.append(line)
        return out

    def load_gcode_lines(self, lines, label="G-code"):
        self.gcode_lines = lines
        x = y = 0.0
        abs_mode = True
        scale = 1.0
        motion = "G0"
        pen_down = False
        pts = [(0.0, 0.0)]
        segs = []
        for idx, line in enumerate(lines, 1):
            if re.search(r"\bG90\b", line): abs_mode = True
            if re.search(r"\bG91\b", line): abs_mode = False
            if re.search(r"\bG20\b", line): scale = 25.4
            if re.search(r"\bG21\b", line): scale = 1.0
            if re.search(r"\bG0\b", line): motion = "G0"
            if re.search(r"\bG1\b", line): motion = "G1"
            zm = Z_WORD_RE.search(line)
            if zm:
                pen_down = float(zm.group(1)) <= 0.001
            words = {m.group(1): float(m.group(2)) * scale for m in AXIS_XY_RE.finditer(line)}
            nx = words["X"] if ("X" in words and abs_mode) else (x + words.get("X", 0.0))
            ny = words["Y"] if ("Y" in words and abs_mode) else (y + words.get("Y", 0.0))
            if nx != x or ny != y:
                kind = "draw" if (motion == "G1" and pen_down) else "travel"
                segs.append((x, y, nx, ny, kind, idx))
                x, y = nx, ny
                pts.append((x, y))
        self.preview_seg_lines = [s[5] for s in segs]
        self.preview.set_data(segs, pts, self.machine_width, self.machine_height)
        self.prog_bar.value = 0
        self.prog_lbl.text = f"0% (0/{len(lines)})"
        self.log(f"{label} loaded: {len(lines)} lines.")

    def load_test_square(self, size=20):
        lines = [
            "G21", "G90", "G0 Z1",
            "G0 X0 Y0", "G1 Z0 F200",
            f"G1 X{size} Y0 F300",
            f"G1 X{size} Y{size}",
            f"G1 X0 Y{size}",
            "G1 X0 Y0",
            "G0 Z1", "G0 X0 Y0"
        ]
        self.gcode_path = "TEST_SQUARE"
        self.load_gcode_lines(lines, f"Test Square {size}mm")

    def open_file_dialog(self):
        search_dirs = ["/sdcard/AVM_GCODE", "/sdcard/Download", os.path.expanduser("~")]
        files = []
        for d in search_dirs:
            if os.path.isdir(d):
                for fn in os.listdir(d):
                    if fn.lower().endswith((".gcode", ".nc", ".tap", ".txt")):
                        files.append(os.path.join(d, fn))
        if not files:
            self.show_popup("No G-code Found", "Phone ke Download ya AVM_GCODE folder mein .gcode file rakhein.")
            return

        box = BoxLayout(orientation="vertical", padding=8, spacing=6)
        sp = Spinner(text=os.path.basename(files[0]), values=[os.path.basename(f) for f in files], size_hint_y=None, height=44)
        box.add_widget(sp)
        btn_row = BoxLayout(size_hint_y=None, height=42, spacing=6)
        pop = Popup(title="Select G-code File", content=box, size_hint=(0.9, 0.4))

        def _load(*_):
            sel_name = sp.text
            for full in files:
                if os.path.basename(full) == sel_name:
                    with open(full, "r", encoding="utf-8", errors="ignore") as f:
                        parsed = self.parse_gcode(f)
                    self.gcode_path = full
                    self.load_gcode_lines(parsed, sel_name)
                    break
            pop.dismiss()

        btn_load = Button(text="Load", on_release=_load)
        btn_cancel = Button(text="Cancel", on_release=pop.dismiss)
        btn_row.add_widget(btn_load)
        btn_row.add_widget(btn_cancel)
        box.add_widget(btn_row)
        pop.open()

    def _poll_status(self, _dt):
        if self.ser and self.ser.is_open and not self.sender_busy:
            threading.Thread(target=self._poll_worker, daemon=True).start()

    def _poll_worker(self):
        try:
            with self.serial_lock:
                if not self.ser or not self.ser.is_open:
                    return
                self.ser.write(b"?")
                self.ser.flush()
                raw = self.ser.readline()
            if raw:
                line = raw.decode("ascii", errors="ignore").strip()
                m = re.search(r"(?:MPos|WPos):(-?[0-9.]+),(-?[0-9.]+),(-?[0-9.]+)", line)
                if m:
                    self.machine_position = dict(zip(("X", "Y", "Z"), map(float, m.groups())))
                    Clock.schedule_once(lambda *_: self._update_pos_ui(), 0)
        except Exception:
            pass

    def _update_pos_ui(self):
        p = self.machine_position
        xs = "—" if p["X"] is None else f"{p['X']:.1f}"
        ys = "—" if p["Y"] is None else f"{p['Y']:.1f}"
        zs = "—" if p["Z"] is None else f"{p['Z']:.1f}"
        self.pos_lbl.text = f"Pos: X {xs} Y {ys} Z {zs} | Bed: {self.machine_width:.0f}×{self.machine_height:.0f}"
        self.preview.set_live(p)

    def _read_ack(self, timeout=60):
        deadline = time.time() + timeout
        while not self.stop_sender.is_set():
            if self.pause_sender.is_set():
                deadline = time.time() + timeout
            if time.time() > deadline:
                raise TimeoutError("GRBL reply timeout.")
            with self.serial_lock:
                if not self.ser or not self.ser.is_open:
                    raise RuntimeError("USB disconnected.")
                raw = self.ser.readline()
            if raw:
                txt = raw.decode("utf-8", errors="replace").strip()
                low = txt.lower()
                if low == "ok" or low.startswith(("error:", "alarm:")):
                    return txt
            else:
                time.sleep(0.02)
        return "STOPPED"

    def start_job(self, start_idx=0):
        if not self.ser or not self.ser.is_open:
            self.show_popup("USB Error", "Pehle Arduino OTG se connect karein.")
            return
        if not self.gcode_lines:
            self.show_popup("No G-code", "Pehle G-code file load karein ya Test Square chunen.")
            return
        if self.sender_busy:
            return
        self.stop_sender.clear()
        self.graceful_stop.clear()
        self.pause_sender.clear()
        self._job_start_idx = start_idx
        self._last_completed = start_idx
        self.sender_busy = True
        threading.Thread(target=self._sender_worker, daemon=True).start()

    def _sender_worker(self):
        total = len(self.gcode_lines)
        try:
            for idx, line in enumerate(self.gcode_lines[self._job_start_idx:], start=self._job_start_idx + 1):
                if self.stop_sender.is_set() or self.graceful_stop.is_set():
                    break
                while self.pause_sender.is_set() and not self.stop_sender.is_set() and not self.graceful_stop.is_set():
                    time.sleep(0.1)
                if self.stop_sender.is_set() or self.graceful_stop.is_set():
                    break

                with self.serial_lock:
                    self.ser.write((line + "\n").encode("ascii", errors="ignore"))
                    self.ser.flush()

                reply = self._read_ack(timeout=60)
                if reply == "STOPPED":
                    break
                if reply.lower().startswith(("alarm:", "error:")):
                    raise RuntimeError(f"Line {idx}: {reply}")

                if re.search(r"\bG90\b", line): self.job_abs_mode = True
                if re.search(r"\bG91\b", line): self.job_abs_mode = False
                if re.search(r"\bG20\b", line): self.job_metric = False
                if re.search(r"\bG21\b", line): self.job_metric = True
                if re.search(r"\bG0\b", line): self.job_last_motion = "G0"
                if re.search(r"\bG1\b", line): self.job_last_motion = "G1"
                fm = re.search(r"\bF\s*(\d+(?:\.\d*)?|\.\d+)", line)
                if fm:
                    self.job_last_feed = float(fm.group(1))

                with self.runtime_lock:
                    z_req = self.pending_runtime_z
                    self.pending_runtime_z = None
                if z_req is not None:
                    zt, zf = z_req
                    zcmd = f"G21 G90 G1 Z{zt:g} F{zf}"
                    with self.serial_lock:
                        self.ser.write((zcmd + "\n").encode("ascii"))
                        self.ser.flush()
                    self._read_ack(timeout=60)
                    restore = []
                    if not self.job_metric: restore.append("G20")
                    if not self.job_abs_mode: restore.append("G91")
                    restore.append(self.job_last_motion)
                    if self.job_last_feed: restore.append(f"F{self.job_last_feed:g}")
                    with self.serial_lock:
                        self.ser.write((" ".join(restore) + "\n").encode("ascii"))
                        self.ser.flush()
                    self._read_ack(timeout=60)

                self._last_completed = idx
                self._save_checkpoint(idx, line)
                pct = (idx / max(1, total)) * 100.0
                seg_idx = bisect.bisect_right(self.preview_seg_lines, idx)
                Clock.schedule_once(lambda _dt, p=pct, i=idx, s=seg_idx: self._update_job_ui(p, i, total, s), 0)

            if self.stop_sender.is_set() or self.graceful_stop.is_set():
                self.log(f"Stopped safely at line {self._last_completed}/{total}.")
            else:
                self.log(f"Drawing Complete! ({total} lines)")
                if os.path.exists(self.checkpoint_path):
                    os.remove(self.checkpoint_path)
        except Exception as exc:
            self.log(f"Job error: {exc}")
        finally:
            self.sender_busy = False
            self.pause_sender.clear()

    def _update_job_ui(self, pct, idx, total, seg_idx):
        self.prog_bar.value = pct
        self.prog_lbl.text = f"{pct:.1f}% ({idx}/{total})"
        self.preview.set_progress(seg_idx)

    def pause_job(self):
        if self.sender_busy:
            self.pause_sender.set()
            self.send_cmd("!")
            self.log("Paused.")

    def resume_job(self):
        if self.sender_busy:
            self.pause_sender.clear()
            self.send_cmd("~")
            self.log("Resumed.")

    def stop_job(self):
        if self.sender_busy:
            was_paused = self.pause_sender.is_set()
            self.graceful_stop.set()
            self.pause_sender.clear()
            if was_paused:
                self.send_cmd("~")
            self.log("Stopping after current line ACK...")

    def _save_checkpoint(self, line_num, last_cmd):
        try:
            with open(self.checkpoint_path, "w", encoding="utf-8") as f:
                json.dump({
                    "line": int(line_num),
                    "total": len(self.gcode_lines),
                    "last_cmd": last_cmd,
                    "lines": self.gcode_lines
                }, f)
        except Exception:
            pass

    def resume_checkpoint(self):
        if self.sender_busy or not os.path.exists(self.checkpoint_path):
            self.log("No checkpoint available.")
            return
        try:
            with open(self.checkpoint_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            completed = int(data.get("line", 0))
            lines = data.get("lines", [])
            if 0 < completed < len(lines):
                self.load_gcode_lines(lines, "Checkpoint Job")
                self.start_job(start_idx=completed)
                self.log(f"Resuming from line {completed + 1}...")
        except Exception as exc:
            self.log(f"Checkpoint resume error: {exc}")


if __name__ == "__main__":
    AVMPlotterAndroidApp().run()
