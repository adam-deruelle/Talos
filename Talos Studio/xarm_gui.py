#!/usr/bin/env python3
"""
xArm — Console: native (Tkinter) app to control the Hiwonder xArm.

Native "phosphor" console: amber = target, green = read position, readouts
in tabular mono font, arm visualization (side view + top view),
gesture learning, per-axis torque, gripper, sync, mirror, feed + pause,
dark/light theme. Compact, scrollable interface with rounded elements.

Dependencies: hidapi (system); pip install hid.
Launch      : python xarm_gui.py     (or the ./xArm binary)
"""

import asyncio
import json
import math
import os
import queue
import socket
import threading
import time
import tkinter as tk
import tkinter.font as tkfont
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

try:
    import hid
except ImportError:
    hid = None

DEVICE_ERRORS = (OSError,) if hid is None else (OSError, getattr(hid, "HIDException", OSError))

# ----------------------------------------------------------------------
# Protocol (unchanged — proven)
# ----------------------------------------------------------------------
VENDOR_ID = 0x0483
PRODUCT_ID = 0x5750
REPORT_SIZE = 64
CMD_SERVO_MOVE = 0x03
CMD_MULT_SERVO_POS_READ = 0x15
CMD_SERVO_UNLOAD = 0x14
CMD_GET_BATTERY_VOLTAGE = 0x0F

PHONE_UDP_PORT = 8899                 # phone IMU streamer / camera_tracker.py send here

SERVOS = list(range(1, 7))
FULL_MIN, FULL_MAX = 0, 1000
CENTER = 500
HOME = {s: CENTER for s in SERVOS}
DEADBAND = 20
DELTA_DEFAULT = 350
DELTA_LIMIT = 500

STOW = {1: 500, 2: 515, 3: 150, 4: 265, 5: 740, 6: 505}
STOW_ORDER = [3, 4, 5, 6, 2, 1]

NOMS = {1: ("gripper", "servo 1"), 2: ("wrist", "rotation"), 3: ("wrist", "flex"),
        4: ("elbow", "servo 4"), 5: ("shoulder", "servo 5"), 6: ("base", "servo 6")}

INVERSES = {1, 2, 4, 6}              # hardware direction opposite to display (gripper included)
MIROIR = FULL_MIN + FULL_MAX
MAX_WIDTH = 1480                     # max usable width (symmetric gutters beyond)

DEG_PER_UNIT = 0.24                  # 0–1000 ↔ 0–240° (0.24°/unit); 500 = 120°

# Segment lengths (mm) — provided by the operator (to-scale preview)
SEG_MM = {"riser": 32 + 33, "upper": 101, "fore": 90, "wrist": 52, "grip": 85}
# Rotation direction of each joint in the preview (±1) — adjust if an axis
# turns the wrong way on screen relative to the real arm.
JOINT_SIGN = {5: -1, 4: +1, 3: -1}

# Pose presets (0–1000 display space; clamped to limits when going there)
PRESETS = {
    "Neutral (center)":   {1: 500, 2: 500, 3: 500, 4: 500, 5: 500, 6: 500},
    "Stow":             {1: 500, 2: 515, 3: 150, 4: 265, 5: 740, 6: 505},
    "Arm raised":         {1: 500, 2: 500, 3: 500, 4: 500, 5: 760, 6: 500},
    "Arm forward":  {1: 500, 2: 500, 3: 500, 4: 650, 5: 360, 6: 500},
    "Toward floor":       {1: 820, 2: 500, 3: 420, 4: 730, 5: 320, 6: 500},
    "Salute":             {1: 500, 2: 500, 3: 600, 4: 340, 5: 720, 6: 500},
    "Gripper open":     {1: 820, 2: 500, 3: 500, 4: 500, 5: 500, 6: 500},
    "Gripper closed":      {1: 180, 2: 500, 3: 500, 4: 500, 5: 500, 6: 500},
}

# Speed profiles = cubic Bézier curves (control points p1x,p1y,p2x,p2y;
# fixed endpoints (0,0)→(1,1)), like CSS timing functions.
EASING_PRESETS = {
    "Linear":     (0.00, 0.00, 1.00, 1.00),
    "Smooth":     (0.42, 0.00, 0.58, 1.00),   # ease-in-out
    "Accelerate": (0.42, 0.00, 1.00, 1.00),   # ease-in
    "Decelerate": (0.00, 0.00, 0.58, 1.00),   # ease-out
    "Snappy":     (0.25, 0.10, 0.25, 1.00),   # CSS "ease"
    "Firm":       (0.70, 0.00, 0.30, 1.00),   # pronounced ease-in-out
}


def _bez1(t, a, b):
    """One 1D cubic Bézier component with endpoints 0 and 1, controls a,b."""
    u = 1 - t
    return 3 * u * u * t * a + 3 * u * t * t * b + t * t * t


def bezier_ease(x, p1x, p1y, p2x, p2y):
    """Progress y ∈ [0,1] for time fraction x, cubic-bezier(p1x,p1y,p2x,p2y) curve."""
    if (p1x, p1y, p2x, p2y) == (0.0, 0.0, 1.0, 1.0):
        return x
    lo, hi = 0.0, 1.0
    for _ in range(24):                      # bisection: find t such that X(t)=x
        t = (lo + hi) / 2
        if _bez1(t, p1x, p2x) < x:
            lo = t
        else:
            hi = t
    return max(0.0, min(1.0, _bez1((lo + hi) / 2, p1y, p2y)))


def flip(servo_id, pos):
    return MIROIR - pos if servo_id in INVERSES else pos


# ----------------------------------------------------------------------
# Forward kinematics — operator's DH model (Kinematics.ods, "FK" sheet).
# Standard Denavit–Hartenberg chain. Joints J1..J5 map to servos below; the
# per-joint angle fed to the model is the servo's Target-column angle
# (value * DEG_PER_UNIT), minus a per-joint zero offset. Tip = tool tip.
# Verified to reproduce the spreadsheet's XYZ to the decimal.
FK_JOINT_SERVO = (6, 5, 4, 3, 2)                      # J1 base, J2 shoulder, J3 elbow, J4 wrist-flex, J5 wrist-rot
FK_THETA_OFFSET = (120.0, 30.0, 119.0, 121.0, 119.0)  # deg subtracted from each joint angle
FK_ALPHA = (90.0, 0.0, 0.0, -90.0, 0.0)               # link twist alpha (deg)
FK_D = (32.0, 0.0, 0.0, 0.0, 0.0)                     # link offset d (mm)
FK_A = (0.0, 33.0, 101.0, 90.0, 52.0)                 # link length a (mm)
FK_TOOL = 85.0                                        # tool (gripper) length along tool x (mm)


def _dh_mat(theta, alpha, d, a):
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return ((ct, -st * ca,  st * sa, a * ct),
            (st,  ct * ca, -ct * sa, a * st),
            (0.0,      sa,      ca,     d),
            (0.0,     0.0,     0.0,   1.0))


def _mat4_mul(A, B):
    return tuple(tuple(sum(A[i][k] * B[k][j] for k in range(4)) for j in range(4)) for i in range(4))


def fk_pose(vals):
    """Tool-tip position (x, y, z) in mm for servo values `vals` (0-1000 display space),
    in the base frame (Z up), per the operator's DH model."""
    T = ((1.0, 0.0, 0.0, 0.0), (0.0, 1.0, 0.0, 0.0), (0.0, 0.0, 1.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    for i, s in enumerate(FK_JOINT_SERVO):
        theta = math.radians(vals.get(s, CENTER) * DEG_PER_UNIT - FK_THETA_OFFSET[i])
        T = _mat4_mul(T, _dh_mat(theta, math.radians(FK_ALPHA[i]), FK_D[i], FK_A[i]))
    T = _mat4_mul(T, _dh_mat(0.0, 0.0, 0.0, FK_TOOL))
    return T[0][3], T[1][3], T[2][3]


# Inverse kinematics — numerical damped-least-squares on top of fk_pose (used by phone Tip/Camera modes).
IK_JOINTS = (6, 5, 4, 3)


def _solve3(A, b):
    """Solve the 3x3 system A y = b (returns [0,0,0] if singular)."""
    d = (A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
         - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
         + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]))
    if abs(d) < 1e-12:
        return [0.0, 0.0, 0.0]
    i00 = (A[1][1] * A[2][2] - A[1][2] * A[2][1]) / d
    i01 = (A[0][2] * A[2][1] - A[0][1] * A[2][2]) / d
    i02 = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) / d
    i10 = (A[1][2] * A[2][0] - A[1][0] * A[2][2]) / d
    i11 = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) / d
    i12 = (A[0][2] * A[1][0] - A[0][0] * A[1][2]) / d
    i20 = (A[1][0] * A[2][1] - A[1][1] * A[2][0]) / d
    i21 = (A[0][1] * A[2][0] - A[0][0] * A[2][1]) / d
    i22 = (A[0][0] * A[1][1] - A[0][1] * A[1][0]) / d
    return [i00 * b[0] + i01 * b[1] + i02 * b[2],
            i10 * b[0] + i11 * b[1] + i12 * b[2],
            i20 * b[0] + i21 * b[1] + i22 * b[2]]


def ik_solve(target, seed, limits_fn, iters=60, lam=0.1, step=60.0):
    """Servo values (0-1000) for IK_JOINTS reaching tool-tip `target` (x,y,z mm).
    `limits_fn(s) -> (lo, hi)` clamps each joint. Returns ({servo: int}, residual_mm)."""
    def cl(s, val):
        lo, hi = limits_fn(s)
        return lo if val < lo else hi if val > hi else val
    v = {s: float(seed.get(s, CENTER)) for s in IK_JOINTS}
    base = dict(seed)
    err = 1e9
    for _ in range(iters):
        base.update({s: v[s] for s in IK_JOINTS})
        px, py, pz = fk_pose(base)
        e = [target[0] - px, target[1] - py, target[2] - pz]
        err = math.sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2])
        if err < 0.4:
            break
        J = [[0.0] * 4 for _ in range(3)]
        for jc, s in enumerate(IK_JOINTS):
            keep = base[s]
            base[s] = cl(s, v[s] + 1.0)
            qx, qy, qz = fk_pose(base)
            base[s] = keep
            J[0][jc] = qx - px; J[1][jc] = qy - py; J[2][jc] = qz - pz
        jjt = [[sum(J[r][k] * J[c][k] for k in range(4)) for c in range(3)] for r in range(3)]
        for k in range(3):
            jjt[k][k] += lam * lam
        y = _solve3(jjt, e)
        for jc, s in enumerate(IK_JOINTS):
            dq = J[0][jc] * y[0] + J[1][jc] * y[1] + J[2][jc] * y[2]
            v[s] = cl(s, v[s] + max(-step, min(step, dq)))
    return {s: int(round(v[s])) for s in IK_JOINTS}, err


# Scenario = an ordered list of typed "blocks" (Scratch-style). Each is a dict with "type":
#   pose    {pose, dur}                go to a saved pose over dur ms
#   wait    {ms}                       pause
#   rotor   {servo, value, dur}        move one servo to value over dur ms
#   gripper {mode: open|half|close}    gripper preset
#   action  {action: home|stow|wave|release}
BLOCK_TYPES = ("pose", "wait", "rotor", "gripper", "action")


def _norm_blocks(raw):
    """Normalize a saved scenario to typed blocks; migrate the legacy {pose,dur,pause} steps."""
    out = []
    for st in raw if isinstance(raw, list) else []:
        if not isinstance(st, dict):
            continue
        if st.get("type") in BLOCK_TYPES:
            out.append(dict(st))
        elif "pose" in st:                                  # legacy step → pose block (+ wait if it had a pause)
            try:
                dur = int(st.get("dur", 800) or 800)
            except (ValueError, TypeError):
                dur = 800
            out.append({"type": "pose", "pose": st["pose"], "dur": dur})
            try:
                pause = int(st.get("pause", 0) or 0)
            except (ValueError, TypeError):
                pause = 0
            if pause > 0:
                out.append({"type": "wait", "ms": pause})
    return out


POSES_FILE = Path.home() / ".xarm_poses.json"
REC_FILE = Path.home() / ".xarm_recordings.json"
SEQ_FILE = Path.home() / ".xarm_sequence.json"
SETTINGS_FILE = Path.home() / ".xarm_settings.json"

# ----------------------------------------------------------------------
# Palettes
# ----------------------------------------------------------------------
DARK = {
    "bg": "#080a0c", "surface": "#0e1319", "surface2": "#141b23", "border": "#232b35",
    "text": "#dbe3ea", "muted": "#8b95a3", "faint": "#3d4653",
    "accent": "#ffb020", "accent_fill": "#ffb020", "accent2": "#c98a12", "on_accent": "#0b0f14",
    "live": "#2ecb8b", "warn": "#ff8c42", "danger": "#ff5245", "danger2": "#d8433b",
    "track": "#39424f", "trough": "#05070a", "grid": "#101820", "led_off": "#1c232b",
}
LIGHT = {
    "bg": "#e9ecef", "surface": "#f8f9fb", "surface2": "#e6eaee", "border": "#c2cad2",
    "text": "#171e26", "muted": "#4f5a67", "faint": "#aab2bc",
    "accent": "#7a4a00", "accent_fill": "#e08600", "accent2": "#a85f00", "on_accent": "#1a1205",
    "live": "#0a7a48", "warn": "#a8480f", "danger": "#cf3a30", "danger2": "#a52d25",
    "track": "#cfd6dd", "trough": "#dde2e7", "grid": "#dbe1e8", "led_off": "#c7cfd7",
}

FONT_PREFS = ["Inter", "Ubuntu", "Cantarell", "Noto Sans", "DejaVu Sans", "Segoe UI"]
MONO_PREFS = ["JetBrains Mono", "Cascadia Code", "DejaVu Sans Mono", "Ubuntu Mono"]


def _pick(root, prefs, fallback):
    fams = set(tkfont.families(root))
    for p in prefs:
        if p in fams:
            return p
    return fallback


def round_rect(cv, x1, y1, x2, y2, r, **kw):
    r = max(0, min(r, (x2 - x1) / 2, (y2 - y1) / 2))
    pts = [x1 + r, y1, x2 - r, y1, x2, y1, x2, y1 + r, x2, y2 - r, x2, y2,
           x2 - r, y2, x1 + r, y2, x1, y2, x1, y2 - r, x1, y1 + r, x1, y1]
    return cv.create_polygon(pts, smooth=True, **kw)


# ----------------------------------------------------------------------
# Rounded widgets (Canvas)
# ----------------------------------------------------------------------
class RButton(tk.Canvas):
    def __init__(self, parent, app, text, command=None, kind="normal", small=False, min_w=0, bg=None):
        self.app, self.kind, self.text, self.command, self.small = app, kind, text, command, small
        self.enabled, self._press, self._hover = True, False, False
        self.font = app.f_small if small else app.f_btn
        px, py = ((10, 6) if small else (14, 9))
        self._px, self._min_w = px, min_w
        w = max(min_w, self.font.measure(text) + px * 2)
        h = self.font.metrics("linespace") + py * 2
        self._bg = bg or app.pal["surface"]
        super().__init__(parent, width=w, height=h, highlightthickness=0, bd=0, bg=self._bg)
        self.bind("<Enter>", lambda _e: self._set(hover=True))
        self.bind("<Leave>", lambda _e: self._set(hover=False, press=False))
        self.bind("<ButtonPress-1>", lambda _e: self._set(press=True))
        self.bind("<ButtonRelease-1>", self._on_rel)
        self._draw()

    def _set(self, hover=None, press=None):
        if hover is not None:
            self._hover = hover
        if press is not None:
            self._press = press
        self._draw()

    def _on_rel(self, e):
        was = self._press
        self._press = False
        self._draw()
        if was and self.enabled and 0 <= e.x <= int(self["width"]) and 0 <= e.y <= int(self["height"]):
            if self.command:
                self.command()

    def set_text(self, text):
        self.text = text
        self.config(width=max(self._min_w, self.font.measure(text) + self._px * 2))
        self._draw()

    def _colors(self):
        p, k = self.app.pal, self.kind
        if not self.enabled:
            return p["surface2"], p["faint"]
        hot = self._press or self._hover
        if k in ("accent", "on"):
            return (p["accent_fill"] if hot else p["accent2"]), p["on_accent"]
        if k == "danger_fill":
            return (p["danger2"] if hot else p["danger"]), "#ffffff"
        if k == "danger":
            return (p["border"] if hot else p["surface2"]), p["danger"]
        if k == "ghost":
            return (p["surface2"] if hot else self._bg), p["muted"]
        return (p["border"] if hot else p["surface2"]), p["text"]

    def _draw(self):
        self.delete("all")
        w, h = int(self["width"]), int(self["height"])
        fill, fg = self._colors()
        round_rect(self, 1, 1, w - 1, h - 1, h / 2, fill=fill, outline="")
        self.create_text(w / 2, h / 2, text=self.text, fill=fg, font=self.font)


class RToggle(tk.Canvas):
    def __init__(self, parent, app, var, text, command=None, bg=None):
        self.app, self.var, self.command, self.text = app, var, command, text
        f = app.f_small
        w = 46 + f.measure(text) + 6
        h = max(22, f.metrics("linespace") + 6)
        self._bg = bg or app.pal["surface"]
        super().__init__(parent, width=w, height=h, highlightthickness=0, bd=0, bg=self._bg)
        self.bind("<Button-1>", self._toggle)
        self._draw()

    def _toggle(self, _e):
        self.var.set(not self.var.get())
        self._draw()
        if self.command:
            self.command()

    def _draw(self):
        self.delete("all")
        p, on = self.app.pal, bool(self.var.get())
        h = int(self["height"]); ty = h / 2
        round_rect(self, 2, ty - 9, 38, ty + 9, 9,
                   fill=(p["accent_fill"] if on else p["surface2"]), outline=p["border"])
        kx = 29 if on else 11
        self.create_oval(kx - 7, ty - 7, kx + 7, ty + 7,
                         fill=(p["on_accent"] if on else p["faint"]), outline="")
        self.create_text(46, ty, anchor="w", text=self.text, fill=p["text"], font=self.app.f_small)


class RPill(tk.Canvas):
    def __init__(self, parent, app, bg):
        self.app = app
        w = app.f_h2.measure("● Connecting…") + 28
        h = app.f_h2.metrics("linespace") + 14
        super().__init__(parent, width=w, height=h, highlightthickness=0, bd=0, bg=bg)
        self.set("wait", "● Connecting…")

    def set(self, state, text):
        self.delete("all")
        p = self.app.pal
        col = {"ok": p["live"], "ko": p["danger"], "wait": p["warn"]}[state]
        w, h = int(self["width"]), int(self["height"])
        round_rect(self, 1, 1, w - 1, h - 1, h / 2, fill=col, outline="")
        self.create_text(w / 2, h / 2, text=text, fill=p["on_accent"], font=self.app.f_h2)


# ----------------------------------------------------------------------
# Hardware layer
# ----------------------------------------------------------------------
class HidTransport:
    """USB HID link: frames are wrapped in a 0x00 + 64-byte report; a read returns a report."""
    kind = "usb"

    def __init__(self):
        self.dev = None

    def open(self):
        if hid is None:
            raise RuntimeError("module 'hid' not found — pip install hid")
        self.dev = hid.Device(VENDOR_ID, PRODUCT_ID)
        return f"{self.dev.manufacturer} {self.dev.product}".strip() or "USB"

    def send(self, frame):
        self.dev.write(b"\x00" + frame.ljust(REPORT_SIZE, b"\x00"))

    def recv(self, timeout=500):
        return self.dev.read(REPORT_SIZE, timeout=timeout)

    def close(self):
        if self.dev is not None:
            try:
                self.dev.close()
            except Exception:
                pass
        self.dev = None


class BleTransport:
    """Bluetooth-LE transparent-serial link (Hiwonder module: service FFE0 / char FFE1).
    Same LSC frames as USB, sent raw (no report padding). A private asyncio loop runs in a
    thread; the worker calls send/recv synchronously. BLE errors surface as OSError so the
    worker's existing device-error handling marks the arm offline."""
    kind = "ble"
    SVC = "0000ffe0-0000-1000-8000-00805f9b34fb"
    CH = "0000ffe1-0000-1000-8000-00805f9b34fb"

    def __init__(self, target="xArm"):
        self.target = target                          # BLE name or "AA:BB:..." address
        self.client = None
        self.ch = None
        self.addr = ""
        self._buf = bytearray()
        self._lock = threading.Lock()
        self._loop = asyncio.new_event_loop()
        self._thr = threading.Thread(target=self._loop.run_forever, daemon=True)
        self._thr.start()

    def _run(self, coro, timeout=25):
        return asyncio.run_coroutine_threadsafe(coro, self._loop).result(timeout)

    def _on_notify(self, _c, data):
        with self._lock:
            self._buf.extend(data)

    async def _open(self):
        from bleak import BleakScanner, BleakClient
        want_addr = len(self.target) == 17 and self.target.count(":") == 5
        if want_addr:
            found = await BleakScanner.find_device_by_address(self.target, timeout=15.0)
        else:
            found = await BleakScanner.find_device_by_name(self.target, timeout=15.0)
        if found is None:
            raise RuntimeError(f"Bluetooth device '{self.target}' not found (arm on? phone app disconnected?)")
        self.client = BleakClient(found)
        await self.client.connect()
        svc = self.client.services.get_service(self.SVC)
        if svc is None:
            raise RuntimeError("Bluetooth serial service (FFE0) not found on device")
        self.ch = svc.get_characteristic(self.CH)
        await self.client.start_notify(self.ch, self._on_notify)
        self.addr = found.address

    def open(self):
        self._run(self._open(), timeout=35)
        return f"xArm BLE {self.addr}"

    async def _write(self, frame):
        for i in range(0, len(frame), 20):                # respect the small BLE MTU
            await self.client.write_gatt_char(self.ch, frame[i:i + 20], response=False)

    def send(self, frame):
        with self._lock:
            self._buf.clear()                             # drop stale bytes before a fresh command
        try:
            self._run(self._write(frame), timeout=10)
        except Exception as e:
            raise OSError(f"BLE write failed: {e}")

    def recv(self, timeout=500):
        end = time.monotonic() + max(0, timeout) / 1000.0
        while time.monotonic() < end:
            fr = self._pop()
            if fr is not None:
                return fr
            time.sleep(0.01)
        return None

    def _pop(self):
        with self._lock:
            b = self._buf
            i = 0
            while i + 3 <= len(b):
                if b[i] == 0x55 and b[i + 1] == 0x55:
                    end = i + 2 + b[i + 2]
                    if end <= len(b):
                        fr = bytes(b[i:end])
                        del b[:end]
                        return fr
                    break
                i += 1
            if i:
                del b[:i]
            return None

    def close(self):
        try:
            if self.client is not None:
                self._run(self.client.disconnect(), timeout=10)
        except Exception:
            pass
        self.client = None
        try:
            self._loop.call_soon_threadsafe(self._loop.stop)
        except Exception:
            pass


def make_transport(mode, ble_target="xArm"):
    return BleTransport(ble_target) if mode == "ble" else HidTransport()


class XArm:
    def __init__(self):
        self.transport = None
        self.info = ""

    def connect(self, transport=None):
        self.close()
        self.transport = transport if transport is not None else HidTransport()
        self.info = self.transport.open()

    def close(self):
        if self.transport is not None:
            self.transport.close()
            self.transport = None

    def _send(self, cmd, data=()):
        frame = bytes([0x55, 0x55, len(data) + 2, cmd]) + bytes(data)
        self.transport.send(frame)

    def move(self, cibles, duration):
        duration = max(0, min(30000, int(duration)))
        data = [len(cibles), duration & 0xFF, (duration >> 8) & 0xFF]
        for sid, pos in cibles.items():
            pos = flip(sid, int(pos))
            data += [sid, pos & 0xFF, (pos >> 8) & 0xFF]
        self._send(CMD_SERVO_MOVE, data)

    def unload(self, ids=SERVOS):
        ids = list(ids)
        self._send(CMD_SERVO_UNLOAD, [len(ids)] + ids)

    def read_positions(self, ids=SERVOS):
        ids = list(ids)
        self._send(CMD_MULT_SERVO_POS_READ, [len(ids)] + ids)
        data = self.transport.recv(timeout=500)
        if not data or len(data) < 5 or data[3] != CMD_MULT_SERVO_POS_READ:
            return None
        lues = {}
        for i in range(data[4]):
            off = 5 + i * 3
            if off + 2 >= len(data):
                break
            sid = data[off]
            lues[sid] = flip(sid, data[off + 1] | (data[off + 2] << 8))
        return lues or None

    def read_battery(self):
        """Pack voltage in millivolts (works over USB or BLE), or None."""
        self._send(CMD_GET_BATTERY_VOLTAGE, [])
        data = self.transport.recv(timeout=500)
        if not data or len(data) < 6 or data[3] != CMD_GET_BATTERY_VOLTAGE:
            return None
        return data[4] | (data[5] << 8)


# ----------------------------------------------------------------------
# Application
# ----------------------------------------------------------------------
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("xArm — Console")
        self.geometry("1180x840")           # fallback size if not maximized
        self.minsize(800, 540)

        self.theme = "dark"
        self.pal = dict(DARK)

        base = _pick(self, FONT_PREFS, "TkDefaultFont")
        mono = _pick(self, MONO_PREFS, "TkFixedFont")
        self.f_h1 = tkfont.Font(family=base, size=16, weight="bold")
        self.f_h2 = tkfont.Font(family=base, size=11, weight="bold")
        self.f_body = tkfont.Font(family=base, size=10)
        self.f_btn = self.f_body
        self.f_small = tkfont.Font(family=base, size=9)
        self.f_live = tkfont.Font(family=mono, size=18, weight="bold")
        self.f_num = tkfont.Font(family=mono, size=10)
        self.f_delta = self.f_num
        self.f_mono = self.f_num
        self.style = ttk.Style(self)
        try:
            self.style.theme_use("clam")
        except tk.TclError:
            pass

        self.arm = XArm()
        self.connected = False
        self.conn_mode = "usb"                # "usb" (HID) or "ble" (Bluetooth LE)
        self.ble_target = "xArm"             # BLE advertised name (or a MAC address)
        self._batt_mv = None                  # last pack voltage (mV), for the header readout
        self._phone_orient = None             # latest (azimuth, pitch, roll) from a phone IMU streamer
        self._phone_ts = 0.0
        self._phone_lock = threading.Lock()
        self._phone_center_o = None
        self._phone_center_sp = None
        self._phone_pos = None                # latest phone 3D position (mm) from camera_tracker.py
        self._phone_pos_center = None
        self._tip_center = None               # tool-tip XYZ captured at "set center"
        self._phone_out = {}
        self._phone_reach_err = 0.0
        self._phone_sent = {}
        self._phone_last_send = 0.0
        self._oe_x = {}                       # 1-Euro filter state (per servo): last filtered value
        self._oe_dx = {}                      # 1-Euro filter state: last filtered derivative
        self._ph_last_t = 0.0                 # last phone-tick time (for dt)
        self.lock = threading.Lock()
        self.pending = {}
        self.pending_dur = 500
        self.jobs = queue.Queue()
        self.events = queue.Queue()
        self.stop_flag = threading.Event()
        self.run_gate = threading.Event(); self.run_gate.set()
        self.read_pending = False
        self._capture_pending = False   # editor Capture waiting on a fresh live read

        self.setpoint = {s: CENTER for s in SERVOS}
        self.reelles = {}
        self.torque = {s: True for s in SERVOS}
        self.delta = {s: DELTA_DEFAULT for s in SERVOS}
        self.lastcmd = {s: 0.0 for s in SERVOS}
        self.feed = 100
        self.profile = "Linear"
        self.ease_pts = EASING_PRESETS["Linear"]
        self.paused = False
        self.recording = False
        self.rec_buf = []
        self.rec_t0 = 0.0
        self.last_recording = []
        self.armed = 6
        self.grip_open = True
        self._suppress = False
        self.log_lines = []
        self._load_settings()
        self.poses = self._load(POSES_FILE)
        self.recordings = self._load(REC_FILE)
        _seq = self._load(SEQ_FILE) or {}
        _steps = _seq.get("steps", []) if isinstance(_seq, dict) else []
        self.sequence = _norm_blocks(_steps)   # typed blocks (legacy steps migrated), tolerant of a corrupt file
        try:
            self.seq_loops = int(_seq.get("loops", 1)) if isinstance(_seq, dict) else 1
        except (ValueError, TypeError):
            self.seq_loops = 1

        self._configure_styles()
        self._build_ui()
        self.configure(background=self.pal["bg"])
        self._bind_mousewheel()

        self.worker = threading.Thread(target=self._worker, daemon=True)
        self.worker.start()
        self.after(40, self._pump)
        self.after(120, self._align_seeks)
        self.after(2500, self._battery_tick)
        threading.Thread(target=self._phone_listener, daemon=True).start()
        self.after(300, self._phone_tick)
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.bind("<Key>", self._on_key)
        self.log("Inverted axes: " + ", ".join(str(s) for s in sorted(INVERSES))
                 + " (hardware direction opposite to display).")
        self.jobs.put(("connect", None))
        self._maximize()                    # start maximized (windowed full screen)
        if os.environ.get("XARM_OPEN_SEQ") == "1":   # diagnostic aid: open the editor at startup
            self.after(1400, self.open_sequencer)

    def _maximize(self):
        # before the first "map": most WMs honor -zoomed
        try:
            self.attributes("-zoomed", True)
        except tk.TclError:
            try:
                self.state("zoomed")
            except tk.TclError:
                pass
        self.after(300, self._maximize_fallback)

    def _maximize_fallback(self):
        # if the WM ignored -zoomed, force via screen geometry (minus the taskbar)
        try:
            if self.winfo_width() < self.winfo_screenwidth() - 80:
                sw, sh = self.winfo_screenwidth(), self.winfo_screenheight()
                self.geometry(f"{sw}x{sh - 44}+0+0")
        except tk.TclError:
            pass

    # ================= theme =================
    def _configure_styles(self):
        p, st = self.pal, self.style
        st.configure(".", background=p["surface"], foreground=p["text"], fieldbackground=p["surface2"],
                     bordercolor=p["border"], lightcolor=p["surface"], darkcolor=p["surface"],
                     troughcolor=p["surface2"], focuscolor=p["surface"], font=self.f_body)
        st.configure("TFrame", background=p["surface"])
        st.configure("Bg.TFrame", background=p["bg"])
        st.configure("TLabel", background=p["surface"], foreground=p["text"])
        st.configure("Bg.TLabel", background=p["bg"], foreground=p["text"])
        st.configure("Muted.TLabel", background=p["surface"], foreground=p["muted"])
        st.configure("BgMuted.TLabel", background=p["bg"], foreground=p["muted"])
        st.configure("Name.TLabel", background=p["surface"], foreground=p["text"], font=self.f_h2)
        st.configure("Sub.TLabel", background=p["surface"], foreground=p["muted"], font=self.f_small)
        st.configure("Col.TLabel", background=p["surface"], foreground=p["muted"], font=self.f_small)
        st.configure("H1.TLabel", background=p["bg"], foreground=p["text"], font=self.f_h1)
        st.configure("Eyebrow.TLabel", background=p["bg"], foreground=p["muted"], font=self.f_h2)
        st.configure("Card.TLabelframe", background=p["surface"], bordercolor=p["border"],
                     relief="solid", borderwidth=1)
        st.configure("Card.TLabelframe.Label", background=p["surface"], foreground=p["muted"], font=self.f_h2)
        st.configure("TSpinbox", fieldbackground=p["surface2"], background=p["surface2"], foreground=p["text"],
                     arrowcolor=p["muted"], bordercolor=p["border"], padding=3)
        st.configure("TCombobox", fieldbackground=p["surface2"], background=p["surface2"], foreground=p["text"],
                     arrowcolor=p["muted"], bordercolor=p["border"], padding=3)
        st.map("TCombobox", fieldbackground=[("readonly", p["surface2"])], foreground=[("readonly", p["text"])],
               selectbackground=[("readonly", p["surface2"])], selectforeground=[("readonly", p["text"])])
        self.option_add("*TCombobox*Listbox.background", p["surface2"])
        self.option_add("*TCombobox*Listbox.foreground", p["text"])
        self.option_add("*TCombobox*Listbox.selectBackground", p["accent_fill"])
        self.option_add("*TCombobox*Listbox.selectForeground", p["on_accent"])
        st.configure("TEntry", fieldbackground=p["surface2"], foreground=p["text"], bordercolor=p["border"],
                     padding=5, insertcolor=p["text"])
        st.configure("TSeparator", background=p["border"])
        st.configure("Vertical.TScrollbar", background=p["surface2"], troughcolor=p["bg"],
                     bordercolor=p["bg"], arrowcolor=p["muted"])
        st.configure("TNotebook", background=p["bg"], borderwidth=0, tabmargins=(8, 6, 8, 0))
        st.configure("TNotebook.Tab", background=p["surface2"], foreground=p["muted"],
                     padding=(18, 8), font=self.f_h2, borderwidth=0)
        st.map("TNotebook.Tab", background=[("selected", p["surface"])],
               foreground=[("selected", p["text"])])

    def toggle_theme(self):
        self.theme = "light" if self.theme == "dark" else "dark"
        self.pal = dict(LIGHT if self.theme == "light" else DARK)
        self._save_settings()
        self._configure_styles()
        for w in list(self.winfo_children()):
            w.destroy()
        self.configure(background=self.pal["bg"])
        self._build_ui()
        self._render_all()
        self.after(120, self._align_seeks)

    # ================= UI construction =================
    def _build_ui(self):
        self.rowconfigure(2, weight=1)
        self.columnconfigure(0, weight=1)
        self._build_header()
        ttk.Separator(self, orient="horizontal").grid(row=1, column=0, sticky="ew")
        self.nb = ttk.Notebook(self)
        self.nb.grid(row=2, column=0, sticky="nsew")

        # ===== Console tab (live control) =====
        console = ttk.Frame(self.nb, style="Bg.TFrame")
        self.nb.add(console, text="Console")
        console.rowconfigure(0, weight=1); console.columnconfigure(0, weight=1)
        wrap = ttk.Frame(console, style="Bg.TFrame")
        wrap.grid(row=0, column=0, sticky="nsew")
        wrap.rowconfigure(0, weight=1); wrap.columnconfigure(0, weight=1)
        self.scv = tk.Canvas(wrap, bg=self.pal["bg"], highlightthickness=0)
        self.scv.grid(row=0, column=0, sticky="nsew")
        vsb = ttk.Scrollbar(wrap, orient="vertical", command=self.scv.yview)
        vsb.grid(row=0, column=1, sticky="ns")
        self.scv.configure(yscrollcommand=vsb.set)
        inner = ttk.Frame(self.scv, style="Bg.TFrame", padding=(18, 12, 18, 18))
        self._innerwin = self.scv.create_window((0, 0), window=inner, anchor="nw")
        inner.bind("<Configure>", lambda _e: self.scv.configure(scrollregion=self.scv.bbox("all")))
        self.scv.bind("<Configure>", self._on_scv_config)

        inner.columnconfigure(0, weight=3)
        inner.columnconfigure(1, weight=1, minsize=340)
        inner.rowconfigure(0, weight=1)
        left = ttk.Frame(inner, style="Bg.TFrame")
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 12))
        left.columnconfigure(0, weight=1)
        self._build_viz(left)
        self._build_axes(left)
        self._build_commands(left)
        right = ttk.Frame(inner, style="Bg.TFrame")
        right.grid(row=0, column=1, sticky="nsew")
        right.columnconfigure(0, weight=1)
        right.rowconfigure(0, weight=1)
        self._build_rec(right)   # poses & sequences live in their own tab
        self._build_log(inner)

        # ===== Poses & sequences tab (full management) =====
        self.seqtab = ttk.Frame(self.nb, style="Bg.TFrame")
        self.nb.add(self.seqtab, text="Poses & sequences")
        self._build_sequencer(self.seqtab)

    def _on_scv_config(self, e):
        self.scv.itemconfig(self._innerwin, width=e.width)   # fill the full width
        self.scv.coords(self._innerwin, 0, 0)

    def _build_header(self):
        p = self.pal
        bar = ttk.Frame(self, style="Bg.TFrame", padding=(18, 10, 18, 10))
        bar.grid(row=0, column=0, sticky="ew")
        bar.columnconfigure(1, weight=1)
        self.hexlbl = tk.Label(bar, text="⬢", fg=p["accent_fill"], bg=p["bg"],
                               font=tkfont.Font(family=self.f_h1.cget("family"), size=20))
        self.hexlbl.grid(row=0, column=0, rowspan=2, padx=(0, 10))
        ttk.Label(bar, text="xArm", style="H1.TLabel").grid(row=0, column=1, sticky="w")
        ttk.Label(bar, text="Console · Hiwonder · 6 servos", style="BgMuted.TLabel").grid(row=1, column=1, sticky="w")
        r = ttk.Frame(bar, style="Bg.TFrame"); r.grid(row=0, column=2, rowspan=2, sticky="e")
        # connection cluster: status + link picker + reconnect read as one unit
        self.status_pill = RPill(r, self, p["bg"]); self.status_pill.pack(side="left", padx=(0, 6))
        self.conn_cb = ttk.Combobox(r, state="readonly", width=10, values=["USB", "Bluetooth"])
        self.conn_cb.set("Bluetooth" if self.conn_mode == "ble" else "USB")
        self.conn_cb.pack(side="left", padx=(0, 4))
        self.conn_cb.bind("<<ComboboxSelected>>", lambda _e: self._on_conn_mode())
        RButton(r, self, "⟳", command=lambda: self.jobs.put(("connect", {"mode": self.conn_mode})), kind="ghost",
                bg=p["bg"], min_w=34).pack(side="left", padx=(0, 12))
        self.batt_lbl = tk.Label(r, text="Batt —", fg=p["muted"], bg=p["bg"], font=self.f_num)
        self.batt_lbl.pack(side="left", padx=(0, 16))
        if self._batt_mv:
            self._set_battery(self._batt_mv)
        # global governor: speed
        ttk.Label(r, text="Speed", style="BgMuted.TLabel", font=self.f_small).pack(side="left")
        self.feed_var = tk.IntVar(value=self.feed)
        tk.Scale(r, from_=25, to=200, resolution=5, orient="horizontal", showvalue=False,
                 variable=self.feed_var, length=110, bg=p["bg"], fg=p["muted"], troughcolor=p["surface2"],
                 activebackground=p["accent_fill"], highlightthickness=0, bd=0, sliderrelief="flat", width=9,
                 command=lambda _v: self._on_feed()).pack(side="left", padx=6)
        self.feed_lbl = tk.Label(r, text=f"{self.feed}%", fg=p["accent_fill"], bg=p["bg"], font=self.f_num, width=5)
        self.feed_lbl.pack(side="left")
        RButton(r, self, "◐", command=self.toggle_theme, kind="ghost", bg=p["bg"], min_w=34).pack(side="left", padx=(10, 0))
        # arm-actions zone: home + release + e-stop, isolated behind a separator
        ttk.Separator(r, orient="vertical").pack(side="left", fill="y", padx=(14, 12), pady=3)
        RButton(r, self, "⌂ Home", command=self.go_home, kind="accent", bg=p["bg"]).pack(side="left", padx=(0, 8))
        RButton(r, self, "Release", command=self.release, kind="danger", bg=p["bg"]).pack(side="left", padx=(0, 8))
        RButton(r, self, "Stop", command=self.stop, kind="danger_fill", bg=p["bg"]).pack(side="left")

    def _build_viz(self, parent):
        p = self.pal
        card = ttk.LabelFrame(parent, text="  Visualization  ", style="Card.TLabelframe", padding=(18, 14, 18, 16))
        card.grid(row=0, column=0, sticky="ew")
        card.columnconfigure(0, weight=1)
        top = ttk.Frame(card, style="TFrame"); top.grid(row=0, column=0, sticky="ew")
        ttk.Label(top, text="Drag:", style="Col.TLabel").pack(side="left", padx=(0, 4))
        self.drag_mode_cb = ttk.Combobox(top, state="readonly", width=11,
                                         values=["Tip (free)", "One joint"])
        self.drag_mode_cb.set("Tip (free)")                # free 2D tip-follow by default
        self.drag_mode_cb.pack(side="left", padx=(0, 6))
        ttk.Label(top, text="drag the tip in the side view · base/wrist on the dial",
                  style="Col.TLabel").pack(side="left")
        gr = ttk.Frame(top, style="TFrame"); gr.pack(side="right")
        for k, t in (("open", "Open"), ("half", "Half"), ("close", "Close")):
            RButton(gr, self, t, command=lambda k=k: self.gripper(k), kind="ghost", small=True).pack(side="left", padx=6)
        body = ttk.Frame(card, style="TFrame"); body.grid(row=1, column=0, sticky="nsew", pady=(10, 0))
        body.rowconfigure(0, weight=1)
        body.columnconfigure(0, weight=1, uniform="viz")
        body.columnconfigure(1, weight=1, uniform="viz")
        self.armcv = tk.Canvas(body, width=520, height=320, bg=p["surface"], highlightthickness=0)
        self.armcv.grid(row=0, column=0, sticky="nsew")
        self.armcv.bind("<Configure>", lambda _e: self._draw_arm())
        self.armcv.bind("<ButtonPress-1>", self._arm_press)
        self.armcv.bind("<B1-Motion>", self._arm_drag)
        self.armcv.bind("<ButtonRelease-1>", self._arm_release)
        self.topcv = tk.Canvas(body, width=520, height=320, bg=p["surface"], highlightthickness=0)
        self.topcv.grid(row=0, column=1, sticky="nsew", padx=(12, 0))
        self.topcv.bind("<Configure>", lambda _e: self._draw_top())
        self.topcv.bind("<ButtonPress-1>", self._top_press)
        self.topcv.bind("<B1-Motion>", self._top_drag)
        self.topcv.bind("<ButtonRelease-1>", self._top_release)
        # end-effector position (forward kinematics, mm, base frame · Z up)
        xyz = ttk.Frame(card, style="TFrame"); xyz.grid(row=2, column=0, sticky="ew", pady=(12, 0))
        ttk.Label(xyz, text="Tool tip (mm)", style="Col.TLabel").pack(side="left", padx=(0, 10))
        self.tcp_lbl = tk.Label(xyz, text="X —   Y —   Z —", fg=p["accent_fill"], bg=p["surface"], font=self.f_num)
        self.tcp_lbl.pack(side="left")
        self.tcp_live_lbl = tk.Label(xyz, text="", fg=p["live"], bg=p["surface"], font=self.f_num)
        self.tcp_live_lbl.pack(side="left", padx=(16, 0))
        self._update_tcp()

    def _update_tcp(self):
        lbl = getattr(self, "tcp_lbl", None)
        if lbl is None or not lbl.winfo_exists():
            return
        x, y, z = fk_pose(self.setpoint)
        lbl.config(text=f"X {x:+6.0f}   Y {y:+6.0f}   Z {z:+6.0f}")
        if self.reelles:
            reads = {s: self.reelles.get(s, self.setpoint[s]) for s in SERVOS}
            lx, ly, lz = fk_pose(reads)
            self.tcp_live_lbl.config(text=f"·  live  X {lx:+.0f}  Y {ly:+.0f}  Z {lz:+.0f}")
        else:
            self.tcp_live_lbl.config(text="")

    # ---- drag-to-pose on the side view ----
    # Handles are downstream points; the joint that most directly moves them is the pivot upstream:
    #   grab elbow J3 → rotate shoulder(5); grab wrist J4 → elbow(4); grab tip TCP → wrist-flex(3).
    _ARM_HANDLES = ((5, 0, 1), (4, 1, 2), (3, 2, 4))   # (servo, pivot_idx, handle_idx) in _fk points [O,J3,J4,J5,TCP]

    def _drag_mode_name(self):
        cb = getattr(self, "drag_mode_cb", None)
        return cb.get() if cb is not None else "Joint"

    def _joint_toward(self, servo, pivot_idx, handle_idx, cx, cy, seed):
        """Servo value that rotates `servo` so its handle points from the pivot toward (cx, cy) mm.
        Uses a numerical 1-DOF sensitivity so the sign math is automatic."""
        pts = self._fk(seed)[0]
        px, py = pts[pivot_idx]
        a_now = math.atan2(pts[handle_idx][1] - py, pts[handle_idx][0] - px)
        a_want = math.atan2(cy - py, cx - px)
        da = (a_want - a_now + math.pi) % (2 * math.pi) - math.pi
        vv = dict(seed); vv[servo] = self.clamp(servo, vv[servo] + 5)
        h2 = self._fk(vv)[0][handle_idx]
        sens = ((math.atan2(h2[1] - py, h2[0] - px) - a_now + math.pi) % (2 * math.pi) - math.pi) / 5.0
        if abs(sens) < 1e-4:
            return None
        return self.clamp(servo, int(round(seed[servo] + max(-40, min(40, da / sens)))))

    def _arm_press(self, e):
        self._drag = None
        f = getattr(self, "_arm_f", None)
        if not f:
            return
        bx, by = self._arm_bx, self._arm_by
        pts = self._fk(self.setpoint)[0]

        def scr(i):
            return bx + pts[i][0] * f, by + pts[i][1] * f
        mode = self._drag_mode_name()
        if mode == "Tip (free)":                           # drag the tip; whole arm reaches it in 2D
            tx, ty = scr(4)
            if (e.x - tx) ** 2 + (e.y - ty) ** 2 <= 34 * 34:
                a0 = math.degrees(self._fk(self.setpoint)[1])   # lock the tool angle at grab time (no twist)
                self._drag = ("ik", {"a0": a0})
            return
        best, bestd = None, 32 * 32                        # single joint: nearest handle
        for servo, piv, hnd in self._ARM_HANDLES:
            hx, hy = scr(hnd)
            d = (e.x - hx) ** 2 + (e.y - hy) ** 2
            if d < bestd:
                bestd, best = d, (servo, piv, hnd)
        if best:
            self._drag = ("single", best)

    def _arm_drag(self, e):
        d = getattr(self, "_drag", None)
        f = getattr(self, "_arm_f", None)
        if not d or not f:
            return
        cx = (e.x - self._arm_bx) / f; cy = (e.y - self._arm_by) / f   # cursor in mm (FK frame)
        mode, info = d
        if mode == "ik":
            changed = self._arm_ik2d((cx, cy), self.setpoint, a0=info["a0"])
        else:                                                   # single joint
            servo, piv, hnd = info
            new = self._joint_toward(servo, piv, hnd, cx, cy, self.setpoint)
            if new is None:
                return
            changed = {servo: new}
        for s, v in changed.items():
            self.setpoint[s] = v; self._set_slider(s, v); self._stamp(s); self._render_servo(s)
        if changed and self.connected:
            self.queue_move(changed, duration=150)
        self._draw_arm(); self._draw_top()

    def _arm_release(self, _e):
        if getattr(self, "_drag", None):
            self._drag = None
            if self.connected:
                self._sync_after(300)

    def _top_press(self, e):
        self._tdrag = None
        R = getattr(self, "_top_R", None)
        if not R:
            return
        r = math.hypot(e.x - self._top_cx, e.y - self._top_cy)
        if r < 8:
            return
        self._tdrag = 6 if r > R * 0.5 else 2      # outer ring = base, inner = wrist rotation

    def _top_drag(self, e):
        s = getattr(self, "_tdrag", None)
        if not s:
            return
        ang = math.degrees(math.atan2(e.y - self._top_cy, e.x - self._top_cx))
        if ang > 90:
            ang -= 360                              # unwrap so the full needle range is reachable
        v = self.clamp(s, int(round(500 + (ang + 90) / 0.3)))   # inverse of the needle angle map
        self.setpoint[s] = v; self._set_slider(s, v); self._stamp(s); self._render_servo(s)
        if self.connected:
            self.queue_move({s: v}, duration=150)
        self._draw_top(); self._draw_arm()

    def _top_release(self, _e):
        if getattr(self, "_tdrag", None):
            self._tdrag = None
            if self.connected:
                self._sync_after(300)

    def _reach_profile(self):
        """Radial reachability of the tool tip in the side-view plane, as (rmax, rmin) lists indexed
        by integer degrees (0..359) of the tip direction from the shoulder pivot. Sampled over the
        joint-limited range of servos 5/4/3 and cached — recomputed only when a limit (delta) changes."""
        key = tuple(self.limits(s) for s in (5, 4, 3))
        if getattr(self, "_reach_key", None) == key:
            return self._reach_prof
        rmax = [0.0] * 360
        rmin = [float("inf")] * 360
        (l5, h5), (l4, h4), (l3, h3) = key
        N = 30
        def lin(a, b):
            return [a + (b - a) * i / N for i in range(N + 1)]
        for v5 in lin(l5, h5):
            for v4 in lin(l4, h4):
                for v3 in lin(l3, h3):
                    x, y = self._fk({1: 500, 2: 500, 3: v3, 4: v4, 5: v5, 6: 500})[0][4]
                    b = int(math.degrees(math.atan2(-y, x)) % 360)
                    r = math.hypot(x, y)
                    if r > rmax[b]:
                        rmax[b] = r
                    if r < rmin[b]:
                        rmin[b] = r
        self._reach_key = key
        self._reach_prof = (rmax, rmin)
        return self._reach_prof

    def _clamp_reachable(self, target):
        """Clamp a side-view tip target (mm) to the arm's REAL reachable ring for its direction, so
        dragging locks at the true outer edge and the inner dead-zone the arm can't fold into —
        instead of chasing an impossible point (which stalls the IK or triggers a recovery jump)."""
        rmax, rmin = self._reach_profile()
        tx, ty = target
        r = math.hypot(tx, ty)
        ang = math.atan2(-ty, tx)
        b = int(math.degrees(ang) % 360)
        if rmax[b] <= 0.0:                       # this exact direction is unreachable → nearest covered
            for off in range(1, 181):
                if rmax[(b + off) % 360] > 0.0:
                    b = (b + off) % 360; ang = math.radians(b); break
                if rmax[(b - off) % 360] > 0.0:
                    b = (b - off) % 360; ang = math.radians(b); break
        hi = rmax[b]
        if hi <= 0.0:
            return target                        # nothing reachable anywhere (should not happen)
        lo = rmin[b] if rmin[b] != float("inf") else 0.0
        hi = max(hi - 4.0, 0.0)                  # a few mm inside each edge (off the singular boundary,
        lo = min(lo + 4.0, hi)                   # and room for the soft orientation-hold compromise)
        rc = min(hi, max(lo, r))
        return (rc * math.cos(ang), -rc * math.sin(ang))

    def _ik2d_once(self, target, seed, joints, iters, lam, step, a0=None, wo=0.4):
        """One damped-least-squares pass bringing the TCP to `target` (mm) from `seed`.
        When a0 (desired tool world-angle, deg) is given for the 3-joint chain, a SOFT orientation
        task (weight wo, deg→mm) is added so the wrist holds the tool angle instead of twisting —
        position stays the priority (weight 1), so the tip still tracks the cursor in a straight line.
        Returns (solution dict, residual mm)."""
        n = len(joints)
        ori = a0 is not None and n == 3

        def cl(s, x):
            lo, hi = self.limits(s)
            return lo if x < lo else hi if x > hi else x

        def wrap(dg):
            return (dg + 180.0) % 360.0 - 180.0
        v = {s: float(seed.get(s, CENTER)) for s in joints}
        base = dict(seed)
        for _ in range(iters):
            base.update({s: v[s] for s in joints})
            pts, aw, _o = self._fk(base)
            tp = pts[4]; awd = math.degrees(aw)
            ex, ey = target[0] - tp[0], target[1] - tp[1]
            eo = wo * wrap(a0 - awd) if ori else 0.0
            if ex * ex + ey * ey < 0.5 and (not ori or abs(wrap(a0 - awd)) < 0.5):
                break
            Jx = [0.0] * n; Jy = [0.0] * n; Jo = [0.0] * n
            for jc, s in enumerate(joints):
                keep = base[s]; base[s] = cl(s, v[s] + 1.0)
                p2, aw2, _ = self._fk(base); base[s] = keep
                Jx[jc] = p2[4][0] - tp[0]; Jy[jc] = p2[4][1] - tp[1]
                Jo[jc] = wo * wrap(math.degrees(aw2) - awd) if ori else 0.0
            if ori:
                rows = (Jx, Jy, Jo); ev = (ex, ey, eo)                 # 3 tasks, 3 joints
                A = [[sum(rows[k][i] * rows[k][j] for k in range(3)) + (lam * lam if i == j else 0.0)
                      for j in range(3)] for i in range(3)]
                bvec = [sum(rows[k][i] * ev[k] for k in range(3)) for i in range(3)]
                dq = _solve3(A, bvec)
                for jc, s in enumerate(joints):
                    v[s] = cl(s, v[s] + max(-step, min(step, dq[jc])))
            else:
                a = sum(Jx[k] * Jx[k] for k in range(n)) + lam * lam
                b = sum(Jx[k] * Jy[k] for k in range(n))
                dd = sum(Jy[k] * Jy[k] for k in range(n)) + lam * lam
                det = a * dd - b * b
                if abs(det) < 1e-9:
                    break
                y0 = (dd * ex - b * ey) / det; y1 = (-b * ex + a * ey) / det
                for jc, s in enumerate(joints):
                    dq = Jx[jc] * y0 + Jy[jc] * y1
                    v[s] = cl(s, v[s] + max(-step, min(step, dq)))
        sol = {s: int(round(v[s])) for s in joints}
        tp = self._fk({**base, **sol})[0][4]
        return sol, math.hypot(tp[0] - target[0], tp[1] - target[1])

    def _arm_ik2d(self, target, seed, joints=(5, 4, 3), iters=18, lam=0.12, step=60.0, a0=None, wo=0.4):
        """2D IK on the side view: bring the tool tip (TCP) to `target` (mm) using `joints`.
        With `a0` (the tool angle captured when the drag started) it holds that orientation softly, so
        the wrist stops twisting and extension stays straight. Robust to messy dragging: the target is
        first clamped to the reachable ring, and if the warm solve stalls in a joint-limit lock it
        retries from neutral/perturbed seeds — so the arm can always be dragged back out of a bad pose."""
        def cl(s, x):
            lo, hi = self.limits(s)
            return lo if x < lo else hi if x > hi else x
        # 1) lock the target to the REAL joint-limited reachable ring for its direction (outer edge +
        #    inner dead-zone), so an off-canvas yank can't chase an impossible point.
        if tuple(joints) == (5, 4, 3):
            target = self._clamp_reachable(target)
        # 2) warm start from the current pose — smooth and continuous during normal dragging.
        cur = self._fk(seed)[0][4]
        init_err = math.hypot(cur[0] - target[0], cur[1] - target[1])
        warm, werr = self._ik2d_once(target, seed, joints, iters, lam, step, a0, wo)
        if werr <= 8.0 or werr < 0.5 * init_err:       # reached, or clearly progressing → keep it
            return warm
        # 3) genuine limit-lock: escape from neutral + perturbed seeds; among those that clearly beat
        #    the warm result, take the one needing the smallest joint change (least visible jump).
        def jd(sol):
            return max(abs(sol[s] - seed.get(s, CENTER)) for s in joints)
        best, berr, bj = warm, werr, jd(warm)
        cands = [{s: CENTER for s in joints}]
        for dv in (-250, 250):
            cands.append({s: cl(s, seed.get(s, CENTER) + dv) for s in joints})
        for c in cands:
            s2 = dict(seed); s2.update(c)
            sol, err = self._ik2d_once(target, s2, joints, iters, lam, step, a0, wo)
            if err < werr - 1.0 and (err < berr - 1.0 or (abs(err - berr) <= 1.0 and jd(sol) < bj)):
                best, berr, bj = sol, err, jd(sol)
        return best

    def _build_axes(self, parent):
        p = self.pal
        card = ttk.LabelFrame(parent, text="  Axes  ", style="Card.TLabelframe", padding=(18, 14, 18, 16))
        card.grid(row=1, column=0, sticky="ew", pady=(18, 0))
        card.columnconfigure(2, weight=1)
        ttk.Label(card, text="Axis  (⇄ = inverted)", style="Col.TLabel").grid(row=0, column=0, columnspan=2, sticky="w")
        ttk.Label(card, text="hold", style="Col.TLabel").grid(row=0, column=3)
        ttk.Label(card, text="Target", style="Col.TLabel").grid(row=0, column=4, columnspan=2, sticky="e")
        ttk.Label(card, text="Read", style="Col.TLabel").grid(row=0, column=6, sticky="e", padx=(10, 0))

        self.consigne_lbl, self.live_lbl, self.delta_lbl, self.deg_lbl = {}, {}, {}, {}
        self.seek_cv, self.torque_cv, self.scales, self.badge_cv = {}, {}, {}, {}
        self.pos_var = {}
        for r, s in enumerate(SERVOS, start=1):
            lo, hi = self.limits(s)
            self.pos_var[s] = tk.IntVar(value=self.setpoint[s])
            self.badge_cv[s] = tk.Canvas(card, width=24, height=22, bg=p["surface"], highlightthickness=0)
            round_rect(self.badge_cv[s], 1, 1, 23, 21, 7, fill=p["surface2"], outline=p["border"])
            self.badge_cv[s].create_text(12, 11, text=str(s), fill=p["muted"], font=self.f_h2)
            self.badge_cv[s].grid(row=r, column=0, sticky="w", pady=10, padx=(0, 8))
            nm = ttk.Frame(card, style="TFrame"); nm.grid(row=r, column=1, sticky="w")
            top = ttk.Frame(nm, style="TFrame"); top.grid(row=0, column=0, sticky="w")
            ttk.Label(top, text=NOMS[s][0], style="Name.TLabel").pack(side="left")
            if s in INVERSES:
                ttk.Label(top, text="  ⇄", style="Sub.TLabel").pack(side="left")
            ttk.Label(nm, text=NOMS[s][1], style="Sub.TLabel").grid(row=1, column=0, sticky="w")

            sc = tk.Scale(card, from_=lo, to=hi, orient="horizontal", resolution=1, showvalue=False,
                          variable=self.pos_var[s], length=240, sliderlength=26, width=13, bg=p["track"],
                          fg=p["text"], troughcolor=p["trough"], activebackground=p["accent_fill"],
                          highlightthickness=0, bd=0, sliderrelief="flat",
                          command=lambda v, s=s: self._on_slider(s, v))
            sc.grid(row=r, column=2, sticky="ew", padx=12)
            sc.bind("<ButtonRelease-1>", lambda _e, s=s: self._on_release(s))
            sc.bind("<Button-1>", lambda _e, s=s: self._arm(s))
            self.scales[s] = sc

            self.torque_cv[s] = tk.Canvas(card, width=24, height=24, bg=p["surface"], highlightthickness=0)
            self.torque_cv[s].grid(row=r, column=3, padx=(0, 12))
            self.torque_cv[s].bind("<Button-1>", lambda _e, s=s: self.toggle_torque(s))

            cw = ttk.Frame(card, style="TFrame"); cw.grid(row=r, column=4, columnspan=2, sticky="e", padx=(0, 12))
            self.consigne_lbl[s] = tk.Label(cw, text=str(self.setpoint[s]), fg=p["accent_fill"], bg=p["surface"],
                                            font=self.f_live, width=5, anchor="e")
            self.consigne_lbl[s].grid(row=0, column=0, sticky="e")
            self.deg_lbl[s] = tk.Label(cw, text="", fg=p["muted"], bg=p["surface"], font=self.f_delta, anchor="e")
            self.deg_lbl[s].grid(row=1, column=0, sticky="e")

            lw = ttk.Frame(card, style="TFrame"); lw.grid(row=r, column=6, sticky="e", padx=(10, 0))
            self.live_lbl[s] = tk.Label(lw, text="—", fg=p["live"], bg=p["surface"], font=self.f_live, width=5, anchor="e")
            self.live_lbl[s].grid(row=0, column=0, sticky="e")
            self.delta_lbl[s] = tk.Label(lw, text="Δ —", fg=p["muted"], bg=p["surface"], font=self.f_delta, anchor="e")
            self.delta_lbl[s].grid(row=1, column=0, sticky="e")

            self.seek_cv[s] = tk.Canvas(card, height=10, bg=p["surface"], highlightthickness=0)
            self.seek_cv[s].grid(row=r, column=2, sticky="ew", padx=12, pady=(44, 0))
            self.seek_cv[s].bind("<Configure>", lambda _e, s=s: self._draw_seek(s))
        for s in SERVOS:
            self._draw_torque(s)

    def _align_seeks(self):
        for s in SERVOS:
            if s in self.scales:
                off = max(30, self.scales[s].winfo_height() - 2)
                self.seek_cv[s].grid_configure(pady=(off, 0))

    def _build_commands(self, parent):
        p = self.pal
        card = ttk.LabelFrame(parent, text="  Controls  ", style="Card.TLabelframe", padding=(18, 12, 18, 14))
        card.grid(row=2, column=0, sticky="ew", pady=(18, 0))
        card.columnconfigure(0, weight=1)

        self.dur_var = tk.IntVar(value=800)
        self.step_var = tk.IntVar(value=50)
        self.live_var = tk.BooleanVar(value=False)
        self.follow_var = tk.BooleanVar(value=True)
        self.auto_var = tk.BooleanVar(value=False)

        rowc = [0]                          # labeled sub-groups stacked top-to-bottom
        def group(title, sep=True):
            ttk.Label(card, text=title, style="Col.TLabel").grid(row=rowc[0], column=0, sticky="w", pady=(0, 4))
            rowc[0] += 1
            inner = ttk.Frame(card, style="TFrame"); inner.grid(row=rowc[0], column=0, sticky="w")
            rowc[0] += 1
            if sep:
                ttk.Separator(card, orient="horizontal").grid(row=rowc[0], column=0, sticky="ew", pady=(10, 10))
                rowc[0] += 1
            return inner

        # --- Move : "make the arm move" actions (Home lives in the header now) ---
        mv = group("Move")
        RButton(mv, self, "↧ Stow", command=self.go_stow).pack(side="left", padx=(0, 8))
        RButton(mv, self, "〰 Wave", command=self.wave).pack(side="left", padx=(0, 8))
        self.pause_btn = RButton(mv, self, "⏸", command=self.toggle_pause); self.pause_btn.pack(side="left")

        # --- Presets : pick a named pose, Go executes it ---
        pr = group("Presets")
        self.preset_cb = ttk.Combobox(pr, state="readonly", width=20, values=list(PRESETS))
        self.preset_cb.pack(side="left", padx=(0, 6))
        self.preset_cb.bind("<<ComboboxSelected>>", lambda _e: self.goto_preset(self.preset_cb.get()))
        RButton(pr, self, "Go", command=lambda: self.goto_preset(self.preset_cb.get()), kind="accent").pack(side="left")

        # --- Read & sync : move data between the UI targets and the servos ---
        rs = group("Read & sync")
        RButton(rs, self, "Read", command=self.request_read).pack(side="left", padx=(0, 8))
        RButton(rs, self, "Sync", command=self.recaler).pack(side="left", padx=(0, 8))
        RButton(rs, self, "Mirror", command=self.mirror).pack(side="left")

        # --- Follow modes : the live-tracking switches ---
        fm = group("Follow modes")
        RToggle(fm, self, self.live_var, "Live follow").pack(side="left", padx=(0, 16))
        RToggle(fm, self, self.follow_var, "Sliders follow reads").pack(side="left", padx=(0, 16))
        RToggle(fm, self, self.auto_var, "Auto re-read", command=self._auto_toggled).pack(side="left")

        # --- Motion settings : how a move executes (lightest weight, at the bottom) ---
        ms = group("Motion settings")
        ttk.Label(ms, text="Duration", style="Col.TLabel").pack(side="left")
        tk.Scale(ms, from_=100, to=3000, resolution=50, orient="horizontal", variable=self.dur_var, length=140,
                 showvalue=False, bg=p["surface"], fg=p["muted"], troughcolor=p["surface2"], activebackground=p["accent_fill"],
                 highlightthickness=0, bd=0, sliderrelief="flat", sliderlength=20, width=8,
                 command=lambda _v: self.dur_lbl.config(text=str(self.dur_var.get()))).pack(side="left", padx=(6, 4))
        self.dur_lbl = tk.Label(ms, text="800", fg=p["muted"], bg=p["surface"], font=self.f_num, width=4)
        self.dur_lbl.pack(side="left", padx=(0, 14))
        ttk.Label(ms, text="Step", style="Col.TLabel").pack(side="left")
        ttk.Spinbox(ms, from_=1, to=200, width=4, justify="center", textvariable=self.step_var).pack(side="left", padx=(6, 14))
        ttk.Label(ms, text="Speed curve", style="Col.TLabel").pack(side="left")
        self.prof_cb = ttk.Combobox(ms, state="readonly", width=12,
                                    values=list(EASING_PRESETS) + ["Custom"])
        self.prof_cb.set(self.profile)
        self.prof_cb.pack(side="left", padx=(6, 6))
        self.prof_cb.bind("<<ComboboxSelected>>", lambda _e: self._on_profile_cb())
        self.curve_cv = tk.Canvas(ms, width=48, height=32, bg=p["surface2"], highlightthickness=1,
                                  highlightbackground=p["border"])
        self.curve_cv.pack(side="left", padx=(0, 6))
        RButton(ms, self, "Curve…", command=self._open_curve_editor, kind="ghost", small=True).pack(side="left")
        RButton(ms, self, "⚙ Max delta…", command=self._open_settings, kind="ghost", small=True).pack(side="left", padx=(12, 0))
        self._draw_curve()

        # --- Phone control : IMU tilt (Joints/Tip) or camera position (Camera) over Wi-Fi UDP ---
        ph = group("Phone control (Wi-Fi)", sep=False)
        self.phone_var = tk.BooleanVar(value=False)
        RToggle(ph, self, self.phone_var, "Follow phone", command=self._on_phone_toggle).pack(side="left", padx=(0, 16))
        ttk.Label(ph, text="Control", style="Col.TLabel").pack(side="left")
        self.phone_ctrl = ttk.Combobox(ph, state="readonly", width=8, values=["Joints", "Tip", "Camera"])
        self.phone_ctrl.set("Joints")
        self.phone_ctrl.pack(side="left", padx=(6, 14))
        self.phone_ctrl.bind("<<ComboboxSelected>>", lambda _e: self._on_phone_mode())
        RButton(ph, self, "Set center", command=self._phone_set_center, small=True).pack(side="left", padx=(0, 10))
        ttk.Label(ph, text="Sensitivity", style="Col.TLabel").pack(side="left")
        self.phone_gain = tk.IntVar(value=4)
        ttk.Spinbox(ph, from_=1, to=12, width=4, justify="center", textvariable=self.phone_gain).pack(side="left", padx=(6, 14))
        ttk.Label(ph, text="Deadband", style="Col.TLabel").pack(side="left")
        self.phone_dead = tk.IntVar(value=1)
        ttk.Spinbox(ph, from_=0, to=40, width=4, justify="center", textvariable=self.phone_dead).pack(side="left", padx=(6, 12))
        ttk.Label(ph, text="Smooth", style="Col.TLabel").pack(side="left")
        self.phone_smooth = tk.IntVar(value=8)
        ttk.Spinbox(ph, from_=1, to=9, width=4, justify="center", textvariable=self.phone_smooth).pack(side="left", padx=(6, 14))
        self.phone_lbl = tk.Label(ph, text="waiting for phone…", fg=p["muted"], bg=p["surface"], font=self.f_small)
        self.phone_lbl.pack(side="left")

    def _build_rec(self, parent):
        p = self.pal
        card = ttk.LabelFrame(parent, text="  Gesture recording  ", style="Card.TLabelframe", padding=(18, 14, 18, 16))
        card.grid(row=0, column=0, sticky="nsew")
        card.columnconfigure(0, weight=1); card.rowconfigure(2, weight=1)
        top = ttk.Frame(card, style="TFrame"); top.grid(row=0, column=0, columnspan=2, sticky="ew")
        self.rec_btn = RButton(top, self, "● Record", command=self.toggle_record, kind="danger")
        self.rec_btn.pack(side="left")
        self.rec_info = tk.Label(top, text="—", fg=p["live"], bg=p["surface"], font=self.f_small)
        self.rec_info.pack(side="left", padx=8)
        e = ttk.Frame(card, style="TFrame"); e.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(8, 6))
        e.columnconfigure(0, weight=1)
        self.rec_name = tk.StringVar()
        ttk.Entry(e, textvariable=self.rec_name).grid(row=0, column=0, sticky="ew", padx=(0, 6))
        RButton(e, self, "Save", command=self.save_recording, kind="accent").grid(row=0, column=1)
        wrap = tk.Frame(card, bg=p["border"]); wrap.grid(row=2, column=0, columnspan=2, sticky="nsew")
        wrap.rowconfigure(0, weight=1); wrap.columnconfigure(0, weight=1)
        self.rec_liste = tk.Listbox(wrap, exportselection=False, activestyle="none", bg=p["surface2"], fg=p["text"],
                                    selectbackground=p["accent_fill"], selectforeground=p["on_accent"], highlightthickness=0,
                                    bd=0, relief="flat", font=self.f_body, height=6)
        self.rec_liste.grid(row=0, column=0, sticky="nsew", padx=1, pady=1)
        self.rec_liste.bind("<Double-Button-1>", lambda _e: self.replay())
        r = ttk.Frame(card, style="TFrame"); r.grid(row=3, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        r.columnconfigure((0, 1), weight=1)
        RButton(r, self, "▶ Replay", command=self.replay).grid(row=0, column=0, sticky="ew", padx=(0, 6))
        RButton(r, self, "Delete", command=self.delete_recording, kind="danger").grid(row=0, column=1, sticky="ew", padx=(6, 0))
        self._refresh_recs()

    def _build_log(self, parent):
        p = self.pal
        cadre = ttk.Frame(parent, style="Bg.TFrame", padding=(0, 18, 0, 0))
        cadre.grid(row=1, column=0, columnspan=2, sticky="ew")
        cadre.columnconfigure(0, weight=1)
        ttk.Label(cadre, text="Log", style="Eyebrow.TLabel").grid(row=0, column=0, sticky="w", pady=(0, 4))
        boite = tk.Frame(cadre, bg=p["border"]); boite.grid(row=1, column=0, sticky="ew")
        boite.columnconfigure(0, weight=1)
        self.log_txt = tk.Text(boite, height=5, wrap="word", state="disabled", font=self.f_mono,
                               background=p["surface"], foreground=p["muted"], insertbackground=p["text"],
                               relief="flat", bd=0, padx=10, pady=8, highlightthickness=0)
        self.log_txt.grid(row=0, column=0, sticky="ew", padx=1, pady=1)
        sb = ttk.Scrollbar(boite, command=self.log_txt.yview); sb.grid(row=0, column=1, sticky="ns")
        self.log_txt["yscrollcommand"] = sb.set
        if self.log_lines:
            self.log_txt["state"] = "normal"
            self.log_txt.insert("end", "\n".join(self.log_lines[-60:]) + "\n")
            self.log_txt.see("end"); self.log_txt["state"] = "disabled"

    def _bind_mousewheel(self):
        def on_wheel(e):
            n = self.winfo_containing(e.x_root, e.y_root)
            while n is not None:
                if isinstance(n, (tk.Listbox, tk.Text)):
                    return
                n = getattr(n, "master", None)
            num = getattr(e, "num", None)
            delta = -1 if num == 4 else 1 if num == 5 else (-1 if e.delta > 0 else 1)
            self.scv.yview_scroll(delta, "units")
        self.bind_all("<Button-4>", on_wheel)
        self.bind_all("<Button-5>", on_wheel)
        self.bind_all("<MouseWheel>", on_wheel)

    # ================= drawing =================
    def limits(self, s):
        d = max(0, min(DELTA_LIMIT, int(self.delta.get(s, DELTA_DEFAULT))))
        return (max(FULL_MIN, CENTER - d), min(FULL_MAX, CENTER + d))

    def clamp(self, s, v):
        lo, hi = self.limits(s)
        return max(lo, min(hi, int(v)))

    def _update_consigne(self, s):
        self.consigne_lbl[s].config(text=str(int(self.setpoint[s])))
        self.deg_lbl[s].config(text=f"{round(self.setpoint[s] * DEG_PER_UNIT)}°")

    def _draw_torque(self, s):
        cv = self.torque_cv[s]; cv.delete("all"); p = self.pal
        if self.torque.get(s, True):
            cv.create_oval(5, 5, 19, 19, fill=p["muted"], outline=p["muted"])       # held
        else:
            cv.create_oval(5, 5, 19, 19, outline=p["warn"], width=2)                # free

    def _draw_seek(self, s):
        cv = self.seek_cv[s]; p = self.pal
        w = cv.winfo_width() or 240; h = 10
        cv.delete("all")
        round_rect(cv, 1, 1, w - 1, h - 1, 4, fill=p["surface2"], outline=p["border"])
        lo, hi = self.limits(s); span = max(1, hi - lo)
        pc = min(1, max(0, (self.setpoint[s] - lo) / span))
        r = self.reelles.get(s)
        pl = min(1, max(0, (r - lo) / span)) if r is not None else pc
        x1, x2 = sorted((pc * w, pl * w))
        if x2 - x1 > 1:
            cv.create_rectangle(x1, 2, x2, h - 2, outline="", fill=p["track"])
        cv.create_line(pc * w, 0, pc * w, h, fill=p["accent_fill"], width=2)                       # target
        cx = min(w - 3, max(3, pl * w))
        cv.create_polygon(cx - 4, 0, cx + 4, 0, cx, 7, fill=p["live"], outline="")                 # read position

    def _fk(self, v):
        """Side-view kinematics, in mm relative to the base (y downward).
        Real angles: 0.24°/unit (0–1000 ↔ 0–240°)."""
        r = math.radians

        def rot(s, val):
            d = (val - 500) * DEG_PER_UNIT * JOINT_SIGN.get(s, 1)
            return -d if s in INVERSES else d
        a_up = 90 + rot(5, v[5])                 # shoulder
        a_fore = a_up + rot(4, v[4])             # elbow
        a_wa = a_fore + rot(3, v[3])             # wrist (flex)
        O = (0.0, 0.0)                           # base = shoulder pivot (the base is not a segment)
        J3 = (O[0] + SEG_MM["upper"] * math.cos(r(a_up)), O[1] - SEG_MM["upper"] * math.sin(r(a_up)))
        J4 = (J3[0] + SEG_MM["fore"] * math.cos(r(a_fore)), J3[1] - SEG_MM["fore"] * math.sin(r(a_fore)))
        J5 = (J4[0] + SEG_MM["wrist"] * math.cos(r(a_wa)), J4[1] - SEG_MM["wrist"] * math.sin(r(a_wa)))
        TCP = (J5[0] + SEG_MM["grip"] * math.cos(r(a_wa)), J5[1] - SEG_MM["grip"] * math.sin(r(a_wa)))
        return [O, J3, J4, J5, TCP], r(a_wa), (v[1] / 1000) * 20 + 6

    def _chain(self, cv, v, color, width, bx, by, f):
        pts, wa, opn = self._fk(v)
        scr = [(bx + x * f, by + y * f) for (x, y) in pts]
        cv.create_line(*[c for pt in scr for c in pt], fill=color, width=width,
                       capstyle="round", joinstyle="round")
        for (x, y) in scr[1:-1]:
            cv.create_oval(x - 3, y - 3, x + 3, y + 3, fill=color, outline=color)
        tcp = scr[-1]
        jx, jy = math.cos(wa), -math.sin(wa); px, py = -jy, jx
        op = opn * f
        for sgn in (1, -1):
            cv.create_line(tcp[0], tcp[1], tcp[0] + jx * 12 * f + px * op * sgn,
                           tcp[1] + jy * 12 * f + py * op * sgn,
                           fill=color, width=max(2, width - 3), capstyle="round")

    def _draw_arm(self):
        cv = self.armcv; p = self.pal
        w = cv.winfo_width() or 520; h = cv.winfo_height() or 300
        cv.delete("all")
        for gx in range(0, w, 24):
            cv.create_line(gx, 0, gx, h, fill=p["grid"])
        for gy in range(0, h, 24):
            cv.create_line(0, gy, w, gy, fill=p["grid"])
        reach_v = sum(SEG_MM.values()) - SEG_MM["riser"]                  # reach from the base (shoulder at the base)
        reach_h = reach_v                                                 # max reach (semicircle)
        f = max(0.05, min((w - 60) / (2 * reach_h), (h - 50) / reach_v))  # fixed scale (no jump)
        bx, by = w / 2, h - 24
        self._arm_bx, self._arm_by, self._arm_f = bx, by, f   # screen↔mm transform for drag-to-pose
        rp = reach_v * f
        cv.create_arc(bx - rp, by - rp, bx + rp, by + rp, start=0, extent=180,
                      style="arc", outline=p["faint"], dash=(2, 5))
        cv.create_line(8, by, w - 8, by, fill=p["faint"])
        round_rect(cv, bx - 26, by, bx + 26, by + 22, 7, fill=p["surface2"], outline=p["border"])
        self._chain(cv, self.setpoint, p["accent_fill"], 7, bx, by, f)
        if self.reelles:
            reads = {s: self.reelles.get(s, self.setpoint[s]) for s in SERVOS}
            self._chain(cv, reads, p["live"], 6, bx, by, f)
        # draggable joint handles (elbow→shoulder, wrist→elbow, tip→wrist-flex)
        pts, _wa, _opn = self._fk(self.setpoint)
        for idx in (1, 2, 4):
            hx, hy = bx + pts[idx][0] * f, by + pts[idx][1] * f
            cv.create_oval(hx - 6, hy - 6, hx + 6, hy + 6, outline=p["accent_fill"], width=2, fill=p["surface"])
        cv.create_oval(bx - 5, by - 5, bx + 5, by + 5, fill=(p["live"] if self.reelles else p["accent_fill"]), outline="")
        cv.create_text(bx, 16, text="Side view · drag joints to pose", fill=p["muted"], font=self.f_small)
        self._update_tcp()

    def _draw_top(self):
        cv = self.topcv; p = self.pal
        w = cv.winfo_width() or 520; h = cv.winfo_height() or 300
        cv.delete("all")
        cx, cy = w / 2, h / 2 - 6
        R = max(20, min(w, h) / 2 - 34)
        self._top_cx, self._top_cy, self._top_R = cx, cy, R   # for drag
        cv.create_oval(cx - R, cy - R, cx + R, cy + R, outline=p["border"])
        cv.create_oval(cx - R * 0.5, cy - R * 0.5, cx + R * 0.5, cy + R * 0.5, outline=p["faint"], dash=(2, 3))
        for a in range(0, 360, 30):
            rad = math.radians(a)
            cv.create_line(cx + (R - 6) * math.cos(rad), cy + (R - 6) * math.sin(rad),
                           cx + R * math.cos(rad), cy + R * math.sin(rad), fill=p["faint"])

        def needle(val, r, color, wdt, dot=False):
            ang = math.radians(-90 + (val - 500) / 500 * 150)
            ex, ey = cx + r * math.cos(ang), cy + r * math.sin(ang)
            cv.create_line(cx, cy, ex, ey, fill=color, width=wdt, capstyle="round")
            if dot:
                cv.create_oval(ex - 6, ey - 6, ex + 6, ey + 6, outline=color, width=2, fill=p["surface"])
        needle(self.setpoint[6], R - 10, p["accent_fill"], 3, dot=True)     # base handle (outer)
        needle(self.setpoint[2], R * 0.42, p["accent_fill"], 3, dot=True)   # wrist-rotation handle (inner)
        if self.reelles:
            needle(self.reelles.get(6, self.setpoint[6]), R - 10, p["live"], 3)
            needle(self.reelles.get(2, self.setpoint[2]), R * 0.42, p["live"], 3)
        cv.create_oval(cx - 4, cy - 4, cx + 4, cy + 4, fill=p["muted"], outline="")
        cv.create_text(cx, cy + R + 18, text="Top view · drag base & wrist", fill=p["muted"], font=self.f_small)

    def _render_servo(self, s):
        self._update_consigne(s); self._draw_torque(s); self._draw_seek(s)
        r = self.reelles.get(s)
        self.live_lbl[s].config(text=(str(r) if r is not None else "—"))
        d = (self.setpoint[s] - r) if r is not None else None
        self.delta_lbl[s].config(
            text=("Δ —" if d is None else f"Δ {'+' if d > 0 else ''}{d}"),
            fg=(self.pal["live"] if (d is not None and abs(d) < 3) else
                self.pal["warn"] if (d is not None and abs(d) > 60) else self.pal["muted"]))

    def _render_all(self):
        for s in SERVOS:
            self._set_slider(s, self.setpoint[s]); self._render_servo(s)
        self._draw_arm(); self._draw_top()
        self.feed_lbl.config(text=f"{self.feed}%")
        self.pause_btn.set_text("▶" if self.paused else "⏸")
        self.rec_btn.set_text("■ Stop" if self.recording else "● Record")

    # ================= interactions =================
    def _arm(self, s):
        self.armed = s

    def _stamp(self, s):
        self.lastcmd[s] = time.monotonic()

    def _set_slider(self, s, val):
        self._suppress = True
        self.pos_var[s].set(int(val)); self._suppress = False

    def _on_slider(self, s, v):
        if self._suppress:
            return
        self.setpoint[s] = int(float(v)); self._stamp(s)
        self._update_consigne(s); self._draw_seek(s); self._draw_arm(); self._draw_top()
        if self.live_var.get():
            self.queue_move({s: self.setpoint[s]}, duration=min(self.dur_var.get(), 150))

    def _on_release(self, s):
        # Dragging a slider (outside "follow") → move of ONE servo along the speed curve.
        # Start = the arm's READ position (the slider is already on the target after the drag),
        # otherwise start==target and nothing would move.
        if self.live_var.get():
            return
        depart = self.reelles.get(s, self.setpoint[s])
        if depart == self.setpoint[s]:
            return
        self._eased_move({s: self.setpoint[s]}, start={s: depart})

    def nudge(self, s, delta):
        self.setpoint[s] = self.clamp(s, self.setpoint[s] + delta); self._stamp(s)
        self._set_slider(s, self.setpoint[s]); self._render_servo(s); self._draw_arm(); self._draw_top()
        self.queue_move({s: self.setpoint[s]}); self._sync_after(self.dur_var.get())

    def toggle_torque(self, s):
        if self.torque.get(s, True):
            self.jobs.put(("release", {"ids": [s]}))
        else:
            v = self.reelles.get(s, self.setpoint[s])
            self.setpoint[s] = self.clamp(s, v); self._stamp(s); self._set_slider(s, self.setpoint[s])
            self.queue_move({s: self.setpoint[s]}, duration=400); self.torque[s] = True
        self._draw_torque(s)

    def gripper(self, kind):
        lo, hi = self.limits(1)
        demi = round((CENTER + hi) / 2)          # "half" more open (between center and open)
        v = hi if kind == "open" else lo if kind == "close" else demi
        self.grip_open = (kind == "open")
        etiq = {"open": "open", "close": "close"}.get(kind, "half")
        self._eased_move({1: v}, note=f"Gripper: {etiq}.")

    def recaler(self):
        t = {s: self.clamp(s, self.reelles[s]) for s in SERVOS if self.reelles.get(s) is not None}
        if t:
            self._eased_move(t, note="Synced to read positions.")

    def mirror(self):
        t = {s: self.clamp(s, 1000 - self.setpoint[s]) for s in (2, 6)}
        self._eased_move(t, note="Mirror of axes 2 and 6.")

    def wave(self):
        """Hand wave: repeated oscillation of ONE servo (by default the wrist, axis 2) along
        the speed curve — demonstrates easing on a single axis. Bounded by the software
        limits; stops with Stop/Esc."""
        if not self.connected:
            self.log("Arm not connected — wave ignored."); return
        # wrist (axis 2): local rotation, no collision risk — hand wave
        self.jobs.put(("wave", {"servo": 2, "cycles": 4,
                                "duration": max(450, self.dur_var.get()), "amp": 130}))

    def set_profile(self, name):
        self.profile = name
        if name in EASING_PRESETS:
            self.ease_pts = EASING_PRESETS[name]
        if hasattr(self, "prof_cb"):
            self.prof_cb.set(name)
        self._draw_curve()
        self._save_settings()

    def _on_profile_cb(self):
        name = self.prof_cb.get()
        if name == "Custom":
            self._open_curve_editor()
        else:
            self.set_profile(name)

    def _draw_curve(self, cv=None, pts=None):
        cv = cv or getattr(self, "curve_cv", None)
        if cv is None:
            return
        p = self.pal; pts = pts or self.ease_pts
        w = int(cv["width"]); h = int(cv["height"]); m = 4
        x0, y0, x1, y1 = m, h - m, w - m, m
        cv.delete("all")
        cv.create_line(x0, y0, x1, y1, fill=p["faint"])          # diagonal = linear
        p1x, p1y, p2x, p2y = pts
        line = []
        for i in range(25):
            t = i / 24
            line += [x0 + (x1 - x0) * _bez1(t, p1x, p2x), y0 + (y1 - y0) * _bez1(t, p1y, p2y)]
        cv.create_line(*line, fill=p["accent_fill"], width=2)

    def _open_curve_editor(self):
        p = self.pal
        win = tk.Toplevel(self); win.title("Speed curve (Bézier)"); win.configure(bg=p["surface"])
        win.transient(self); win.resizable(False, False)
        pad = ttk.Frame(win, style="TFrame", padding=16); pad.grid()
        ttk.Label(pad, text="Control points of the cubic-bezier curve (0–1).", style="Muted.TLabel").grid(
            row=0, column=0, columnspan=4, sticky="w")
        ttk.Label(pad, text="Start (0,0) and end (1,1) fixed; P1 = ease-in, P2 = ease-out.",
                  style="Col.TLabel").grid(row=1, column=0, columnspan=4, sticky="w", pady=(0, 10))
        prev = tk.Canvas(pad, width=180, height=140, bg=p["surface2"], highlightthickness=1,
                         highlightbackground=p["border"])
        prev.grid(row=2, column=0, columnspan=4, pady=(0, 12))
        vars_ = [tk.DoubleVar(value=self.ease_pts[i]) for i in range(4)]

        def refresh(*_a):
            try:
                pts = tuple(max(0.0, min(1.0, float(v.get()))) for v in vars_)
            except (ValueError, tk.TclError):
                return
            self._draw_curve(prev, pts)
        for i, lab in enumerate(("P1 x", "P1 y", "P2 x", "P2 y")):
            ttk.Label(pad, text=lab, style="TLabel").grid(row=3, column=i, padx=4)
            sp = ttk.Spinbox(pad, from_=0, to=1, increment=0.05, width=6, justify="center", textvariable=vars_[i])
            sp.grid(row=4, column=i, padx=4)
            vars_[i].trace_add("write", refresh)
        bar = ttk.Frame(pad, style="TFrame"); bar.grid(row=5, column=0, columnspan=4, sticky="ew", pady=(14, 0))
        chips = ttk.Frame(bar, style="TFrame"); chips.pack(side="left")
        for name in EASING_PRESETS:
            RButton(chips, self, name, kind="ghost", small=True,
                    command=lambda n=name: [vars_[i].set(EASING_PRESETS[n][i]) for i in range(4)]).pack(side="left", padx=2)

        def apply():
            self.ease_pts = tuple(max(0.0, min(1.0, float(v.get()))) for v in vars_)
            match = next((n for n, e in EASING_PRESETS.items() if e == self.ease_pts), "Custom")
            self.set_profile(match)
            win.destroy()
        RButton(bar, self, "Apply", command=apply, kind="accent").pack(side="right")
        refresh(); win.grab_set()

    def goto_preset(self, name):
        pose = PRESETS.get(name)
        if not pose or not self.connected:
            if not self.connected:
                self.log("Arm not connected — preset ignored.")
            return
        start = dict(self.setpoint)
        cibles = {int(s): self.clamp(int(s), v) for s, v in pose.items()}
        self.jobs.put(("goto", {"targets": cibles, "duration": self.dur_var.get(),
                                "start": start, "note": f"Preset '{name}'."}))
        self._sync_after(self.dur_var.get())

    def _on_feed(self):
        self.feed = int(self.feed_var.get()); self.feed_lbl.config(text=f"{self.feed}%")

    def _on_conn_mode(self):
        self.conn_mode = "ble" if self.conn_cb.get() == "Bluetooth" else "usb"
        self.jobs.put(("connect", {"mode": self.conn_mode}))

    def _set_battery(self, mv):
        self._batt_mv = mv
        lbl = getattr(self, "batt_lbl", None)
        if lbl is None or not lbl.winfo_exists():
            return
        v = mv / 1000.0
        col = self.pal["live"] if v >= 7.0 else self.pal["warn"] if v >= 6.6 else self.pal["danger"]
        lbl.config(text=f"Batt {v:.1f} V", fg=col)

    def toggle_pause(self):
        self.paused = not self.paused
        (self.run_gate.clear if self.paused else self.run_gate.set)()
        self.log("Pause." if self.paused else "Resume.")
        self.pause_btn.set_text("▶" if self.paused else "⏸")

    # ================= keyboard =================
    def _on_key(self, e):
        w = self.focus_get()
        if isinstance(w, (tk.Entry, ttk.Entry, tk.Spinbox, ttk.Spinbox, tk.Listbox, tk.Text)):
            return
        k = e.keysym
        if k in "123456":
            self.armed = int(k); return
        step = self.step_var.get()
        if e.state & 0x1:
            step = 1
        elif e.state & 0x4:
            step = self.step_var.get() * 5
        if k in ("Up", "Right"):
            self.nudge(self.armed, step)
        elif k in ("Down", "Left"):
            self.nudge(self.armed, -step)
        elif k == "space":
            lo, hi = self.limits(1); v = lo if self.grip_open else hi; self.grip_open = not self.grip_open
            self._eased_move({1: v}, note="Gripper (space).")
        elif k in ("h", "H"):
            self.go_home()
        elif k == "Escape":
            self.stop()

    # ================= commands =================
    def log(self, texte, niveau="info"):
        line = time.strftime("%H:%M:%S ") + texte
        self.log_lines.append(line); self.log_lines = self.log_lines[-200:]
        self.log_txt["state"] = "normal"
        self.log_txt.insert("end", line + "\n"); self.log_txt.see("end")
        if int(self.log_txt.index("end-1c").split(".")[0]) > 300:
            self.log_txt.delete("1.0", "100.0")
        self.log_txt["state"] = "disabled"

    def _set_status(self, etat, texte):
        court = {"ok": "● Connected", "ko": "● Offline", "wait": "● Connecting…"}[etat]
        self.status_pill.set(etat, court); self.log(texte)

    def queue_move(self, cibles, duration=None):
        if not self.connected:
            self.log("Arm not connected — move ignored."); return
        propres = {s: self.clamp(s, p) for s, p in cibles.items()}
        for s in propres:
            self.torque[s] = True
        with self.lock:
            self.pending.update(propres)
            self.pending_dur = int(duration if duration is not None else self.dur_var.get())

    def _sync_after(self, duration):
        if self.connected:
            self.after(int(duration) + 350, self.request_read)

    def _eased_move(self, cibles, note=None, start=None):
        """Discrete move (gripper, mirror, sync, poses, released slider…) that goes through the
        speed CURVE: the worker (_goto) interpolates setpoint + preview along the curve.
        `start` = real interpolation start; by default the current setpoint, but it can be
        forced (e.g. released slider: the setpoint is ALREADY on the target, the real start is the
        arm's read position)."""
        if not self.connected:
            self.log("Arm not connected — move ignored."); return
        start = dict(self.setpoint) if start is None else {int(s): int(v) for s, v in start.items()}
        propres = {int(s): self.clamp(int(s), int(v)) for s, v in cibles.items()}
        for s in propres:
            self._stamp(s)                                   # protect the axis from the "follow" re-read
        dur = self.dur_var.get()
        self.jobs.put(("goto", {"targets": propres, "duration": dur, "start": start, "note": note}))
        self._sync_after(dur)

    def go_home(self):
        start = dict(self.setpoint)                      # real start position (before update)
        for s, v in HOME.items():
            self.setpoint[s] = self.clamp(s, v); self._set_slider(s, self.setpoint[s])
        self._render_all()
        self.jobs.put(("goto", {"targets": dict(self.setpoint), "duration": 1500,
                                "start": start, "note": "Return to neutral position."}))
        self._sync_after(1500)

    def go_stow(self):
        self.jobs.put(("park", {}))
        self._sync_after(2200)

    def request_read(self):
        if not self.connected:
            self.log("Arm not connected — cannot read."); return
        if not self.read_pending:
            self.read_pending = True; self.jobs.put(("read", None))

    def release(self):
        if not self.connected:
            return
        self.jobs.put(("release", {"ids": SERVOS}))
        self.auto_var.set(True); self._auto_toggled()

    def stop(self):
        self.stop_flag.set(); self.run_gate.set(); self.paused = False; self.recording = False
        with self.lock:
            self.pending.clear()
        while not self.jobs.empty():
            try:
                self.jobs.get_nowait()
            except queue.Empty:
                break
        self.read_pending = False   # a "read" job may have been purged without posting read_done
        self._capture_pending = False
        self.log("Stop.", "warn")

    def _auto_toggled(self):
        if self.auto_var.get():
            self._auto_tick()

    def _auto_tick(self):
        if not self.auto_var.get():
            return
        if not self._phone_engaged():        # a read mid-stream can cancel a move → skip while following
            self.request_read()
        self.after(1000, self._auto_tick)

    def _battery_tick(self):
        if self.connected and not self._phone_engaged():
            self.jobs.put(("battery", None))
        self.after(20000, self._battery_tick)

    # ================= phone control (Wi-Fi UDP: IMU orientation or camera position) =================
    def _phone_listener(self):
        """Receive UDP packets on one port. Two kinds, auto-detected:
          • 'azimuth,pitch,roll'  from a phone IMU streamer  → orientation
          • 'P,x,y,z'             from camera_tracker.py      → phone 3D position (mm)
        Runs in its own daemon thread; only stores the latest — the UI-thread tick drives the arm."""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("0.0.0.0", PHONE_UDP_PORT))
        except OSError as e:
            self._post("log", f"Phone UDP port {PHONE_UDP_PORT} unavailable: {e}")
            return
        while True:
            try:
                data, _addr = sock.recvfrom(2048)
            except OSError:
                break
            txt = data.decode("ascii", "ignore").strip()
            is_pos = txt[:2] == "P,"
            if is_pos:
                txt = txt[2:]
            vals = []
            for tok in txt.replace("\r", " ").replace("\n", " ").split(","):
                try:
                    vals.append(float(tok.strip()))
                except ValueError:
                    pass
            if len(vals) >= 3:
                with self._phone_lock:
                    if is_pos:
                        self._phone_pos = (vals[0], vals[1], vals[2])
                    else:
                        self._phone_orient = (vals[0], vals[1], vals[2])
                    self._phone_ts = time.monotonic()

    def _phone_set_center(self):
        with self._phone_lock:
            o = self._phone_orient; pos = self._phone_pos
        self._phone_out = {}; self._phone_sent = {}; self._phone_last_send = 0.0
        self._oe_x = {}; self._oe_dx = {}; self._ph_last_t = 0.0    # reset the 1-Euro filter so nothing jumps
        if self._phone_mode() == "Camera":
            if pos is None:
                self.log("No camera data — run camera_tracker.py and aim the phone at the marker.", "warn")
                return False
            self._phone_pos_center = pos
            self._tip_center = fk_pose(self.setpoint)
            self.log(f"Camera center set (x {pos[0]:.0f}, y {pos[1]:.0f}, z {pos[2]:.0f}).")
            return True
        if o is None:
            self.log("No phone data yet — start the phone app first.", "warn"); return False
        self._phone_center_o = o
        self._phone_center_sp = {s: int(self.setpoint[s]) for s in (5, 6, 2)}
        self._tip_center = fk_pose(self.setpoint)
        self.log(f"Phone center set (az {o[0]:.0f}, pitch {o[1]:.0f}, roll {o[2]:.0f}).")
        return True

    def _phone_mode(self):
        cb = getattr(self, "phone_ctrl", None)
        return cb.get() if cb is not None else "Joints"

    def _phone_engaged(self):
        return bool(getattr(self, "phone_var", None) and self.phone_var.get())

    def _on_phone_mode(self):
        if getattr(self, "phone_var", None) and self.phone_var.get():
            self._phone_set_center()         # re-zero for the new mode so nothing jumps

    def _on_phone_toggle(self):
        if self.phone_var.get():
            if not self.connected:
                self.log("Connect the arm first.", "warn"); self.phone_var.set(False); return
            if not self._phone_set_center():
                self.phone_var.set(False); return
            how = "move the phone" if self._phone_mode() == "Camera" else "tilt the phone"
            self.log(f"Follow phone: ON ({self._phone_mode()}) — {how} to move the tip (toggle off to stop).")
        else:
            self.log("Follow phone: OFF.")

    def _phone_smooth_params(self):
        """Map the Smooth slider (1..9) to 1-Euro filter parameters.
        Low slider = responsive (high cutoff, low lag); high slider = very smooth."""
        try:
            sm = max(1, min(9, int(self.phone_smooth.get())))
        except (tk.TclError, ValueError):
            sm = 8
        mincut = 2.5 - (sm - 1) * 0.275     # sm1 -> 2.5 Hz (snappy) ... sm9 -> 0.3 Hz (glassy)
        beta = 0.05 - (sm - 1) * 0.005      # sm1 -> 0.05 (very speed-adaptive) ... sm9 -> 0.01
        return mincut, beta

    def _one_euro(self, s, x, dt, mincut, beta, dcut=1.0):
        """1-Euro filter (Casiez et al.) on servo `s`. Heavy smoothing when the input is still,
        low lag when it moves fast — the standard for de-jittering live human input."""
        xp = self._oe_x.get(s)
        if xp is None:
            self._oe_x[s] = x; self._oe_dx[s] = 0.0
            return x
        dxr = (x - xp) / dt                                  # raw derivative (units/s)
        ad = 1.0 / (1.0 + (1.0 / (2.0 * math.pi * dcut)) / dt)
        dxf = ad * dxr + (1.0 - ad) * self._oe_dx.get(s, 0.0)
        self._oe_dx[s] = dxf
        cutoff = mincut + beta * abs(dxf)                   # speed-adaptive cutoff
        a = 1.0 / (1.0 + (1.0 / (2.0 * math.pi * cutoff)) / dt)
        xf = a * x + (1.0 - a) * xp
        self._oe_x[s] = xf
        return xf

    def _phone_tick(self):
        mode = self._phone_mode()
        engaged = self._phone_engaged()
        with self._phone_lock:
            o = self._phone_orient; pos = self._phone_pos; ts = self._phone_ts
        src = pos if mode == "Camera" else o
        fresh = src is not None and (time.monotonic() - ts) < 1.5
        lbl = getattr(self, "phone_lbl", None)
        if lbl is not None and lbl.winfo_exists():
            oor = engaged and mode in ("Tip", "Camera") and self._phone_reach_err > 6
            extra = f"  · out of reach ({self._phone_reach_err:.0f} mm)" if oor else ""
            if mode == "Camera":
                if pos is None:
                    lbl.config(text=f"waiting for camera tracker → UDP {PHONE_UDP_PORT}", fg=self.pal["muted"])
                elif not fresh:
                    lbl.config(text="camera idle (no data)", fg=self.pal["warn"])
                else:
                    lbl.config(text=f"phone  x {pos[0]:.0f}  y {pos[1]:.0f}  z {pos[2]:.0f}{extra}",
                               fg=(self.pal["warn"] if extra else self.pal["live"]))
            elif o is None:
                lbl.config(text=f"waiting for phone → {self._lan_ip()}:{PHONE_UDP_PORT}", fg=self.pal["muted"])
            elif not fresh:
                lbl.config(text="phone idle (no data)", fg=self.pal["warn"])
            else:
                lbl.config(text=f"az {o[0]:.0f}  pitch {o[1]:.0f}  roll {o[2]:.0f}{extra}",
                           fg=(self.pal["warn"] if extra else self.pal["live"]))
        if not (engaged and self.connected and fresh):
            self.after(33, self._phone_tick); return
        raw = None
        if mode == "Camera" and self._phone_pos_center is not None and self._tip_center is not None:
            try:
                scale = max(1, int(self.phone_gain.get())) / 4.0     # Sensitivity 4 → 1:1 (mm tip per mm phone)
            except (tk.TclError, ValueError):
                scale = 1.0
            pcx, pcy, pcz = self._phone_pos_center
            x0, y0, z0 = self._tip_center
            dx, dy, dz = pos[0] - pcx, pos[1] - pcy, pos[2] - pcz    # phone move (mm) in the marker frame
            # map phone(marker) axes -> arm base frame; flip a sign here if a direction comes out reversed:
            target = (x0 + scale * (-dz), y0 + scale * dx, z0 + scale * (-dy))
            raw, self._phone_reach_err = ik_solve(target, self.setpoint, self.limits, iters=15)
        elif mode != "Camera" and self._phone_center_o is not None:
            try:
                gain = max(1, int(self.phone_gain.get()))
            except (tk.TclError, ValueError):
                gain = 4
            caz, cp, cr = self._phone_center_o
            dzone = 1.0                                       # small; the 1-Euro filter kills the jitter

            def d(x):
                return 0.0 if abs(x) < dzone else x
            daz = ((o[0] - caz + 180) % 360) - 180
            if mode == "Tip" and self._tip_center is not None:
                x0, y0, z0 = self._tip_center
                target = (x0 + gain * d(daz), y0 + gain * d(o[2] - cr), z0 + gain * d(o[1] - cp))
                raw, self._phone_reach_err = ik_solve(target, self.setpoint, self.limits, iters=15)
            else:
                cs = self._phone_center_sp
                raw = {5: self.clamp(5, int(round(cs[5] + gain * d(o[1] - cp)))),
                       6: self.clamp(6, int(round(cs[6] + gain * d(o[2] - cr)))),
                       2: self.clamp(2, int(round(cs[2] + gain * d(daz))))}
                self._phone_reach_err = 0.0
        if raw:
            now = time.monotonic()
            dt = (now - self._ph_last_t) if self._ph_last_t else 0.033
            self._ph_last_t = now
            dt = max(0.008, min(0.10, dt))                    # clamp dt so a stall can't blow up the filter
            mincut, beta = self._phone_smooth_params()
            filt = {}
            for s, val in raw.items():
                fx = self._one_euro(s, float(val), dt, mincut, beta)
                filt[s] = self.clamp(s, int(round(fx)))
            try:
                dead = max(0, int(self.phone_dead.get()))
            except (tk.TclError, ValueError):
                dead = 1
            # Stream continuously: re-send whenever the smoothed target shifts past a tiny
            # threshold. The 1-Euro filter already removes jitter, so there is no hard deadband
            # to stutter on. duration > tick so consecutive moves overlap into one glide.
            moved = max((abs(filt[s] - self._phone_sent.get(s, self.setpoint[s])) for s in filt), default=0)
            if moved > dead:
                self.queue_move(filt, duration=90)
                for s, v in filt.items():
                    self._phone_sent[s] = v
                    self.setpoint[s] = v; self._set_slider(s, v); self._stamp(s); self._render_servo(s)
                self._draw_arm(); self._draw_top()
        self.after(33, self._phone_tick)

    def _lan_ip(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]; s.close(); return ip
        except Exception:
            return "this-PC"

    # ================= delta settings =================
    def _open_settings(self):
        p = self.pal
        win = tk.Toplevel(self); win.title("Max delta per motor"); win.configure(bg=p["surface"])
        win.transient(self); win.resizable(False, False)
        pad = ttk.Frame(win, style="TFrame", padding=16); pad.grid()
        ttk.Label(pad, text="Amplitude of each motor around the center (500).", style="Muted.TLabel").grid(row=0, column=0, columnspan=2, sticky="w")
        ttk.Label(pad, text="Range = 500 − delta … 500 + delta (clamped to 0–1000).", style="Col.TLabel").grid(row=1, column=0, columnspan=2, sticky="w", pady=(0, 10))
        self.delta_vars = {}
        for i, s in enumerate(SERVOS):
            lab = NOMS[s][0] + (" ⇄" if s in INVERSES else "")
            ttk.Label(pad, text=f"{s} · {lab}", style="TLabel").grid(row=i + 2, column=0, sticky="w", pady=3)
            v = tk.IntVar(value=self.delta[s]); self.delta_vars[s] = v
            row = ttk.Frame(pad, style="TFrame"); row.grid(row=i + 2, column=1, sticky="e")
            ttk.Spinbox(row, from_=0, to=DELTA_LIMIT, increment=10, width=6, justify="center", textvariable=v).grid(row=0, column=0)
            lbl = ttk.Label(row, text="", style="Col.TLabel", width=12); lbl.grid(row=0, column=1, padx=(8, 0))

            def upd(*_a, s=s, v=v, lbl=lbl):
                try:
                    d = max(0, min(DELTA_LIMIT, int(v.get())))
                except (ValueError, tk.TclError):
                    d = self.delta[s]
                lbl.config(text=f"{max(0, 500 - d)}–{min(1000, 500 + d)}")
            v.trace_add("write", upd); upd()
        bar = ttk.Frame(pad, style="TFrame"); bar.grid(row=len(SERVOS) + 2, column=0, columnspan=2, sticky="ew", pady=(14, 0))
        RButton(bar, self, "Reset (350)", command=lambda: [self.delta_vars[s].set(DELTA_DEFAULT) for s in SERVOS], kind="ghost").pack(side="left")
        RButton(bar, self, "Apply", command=lambda: self._apply_settings(win), kind="accent").pack(side="right")
        win.grab_set()

    def _apply_settings(self, win):
        for s in SERVOS:
            try:
                self.delta[s] = max(0, min(DELTA_LIMIT, int(self.delta_vars[s].get())))
            except (ValueError, tk.TclError):
                pass
        for s in SERVOS:
            lo, hi = self.limits(s)
            self.scales[s].config(from_=lo, to=hi)
            self.setpoint[s] = self.clamp(s, self.setpoint[s]); self._set_slider(s, self.setpoint[s]); self._render_servo(s)
        self._draw_arm(); self._save_settings()
        self.log("Max delta: " + ", ".join(f"{s}:±{self.delta[s]}" for s in SERVOS))
        win.destroy()

    # ================= poses / recordings =================
    def _refresh_poses(self):
        if getattr(self, "ed_poses", None) is not None and self.ed_poses.winfo_exists():
            self._ed_fill_poses()

    def _refresh_recs(self):
        self.rec_liste.delete(0, "end")
        for n in self.recordings:
            self.rec_liste.insert("end", n)

    def _sel(self, lb):
        s = lb.curselection()
        return lb.get(s[0]) if s else None

    def toggle_record(self):
        if self.recording:
            self.recording = False
        else:
            self.stop_flag.clear(); self.jobs.put(("record", None))

    def save_recording(self):
        if not self.last_recording:
            self.log("No recording to save."); return
        nom = self.rec_name.get().strip() or f"recording {len(self.recordings) + 1}"
        self.recordings[nom] = {"keyframes": self.last_recording}
        self._refresh_recs(); self.rec_name.set(""); self._save(REC_FILE, self.recordings)
        self.log(f"Recording '{nom}' ({len(self.last_recording)} frames) saved.")

    def replay(self):
        nom = self._sel(self.rec_liste)
        kf = self.recordings.get(nom, {}).get("keyframes") if nom else self.last_recording
        if not kf or not self.connected:
            return
        self.stop_flag.clear(); self.jobs.put(("replay", {"keyframes": kf}))
        self.log(f"Replay ({len(kf)} frames).")

    def delete_recording(self):
        nom = self._sel(self.rec_liste)
        if nom and messagebox.askyesno("Delete", f"Delete '{nom}'?"):
            self.recordings.pop(nom, None); self._refresh_recs(); self._save(REC_FILE, self.recordings)

    def import_poses(self):
        chemin = filedialog.askopenfilename(filetypes=[("JSON", "*.json")])
        if not chemin:
            return
        try:
            self.poses.update(json.loads(Path(chemin).read_text()))
            self._refresh_poses(); self._save(POSES_FILE, self.poses); self.log("Poses imported.")
        except Exception as e:
            self.log(f"Import failed: {e}", "warn")

    def export_poses(self):
        chemin = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("JSON", "*.json")])
        if chemin:
            Path(chemin).write_text(json.dumps(self.poses, indent=2)); self.log("Poses exported.")

    # ================= editor & sequencer (tab) =================
    def open_sequencer(self):
        if hasattr(self, "nb") and hasattr(self, "seqtab"):
            self.nb.select(self.seqtab)

    def _build_sequencer(self, parent):
        """Full tab as a numbered left-to-right workflow: 1 Library · 2 Edit pose · 3 Sequence.
        Steps, durations, pauses, reordering, loops. Three columns that fill the tab."""
        p = self.pal
        parent.columnconfigure(0, weight=4, uniform="sq")
        parent.columnconfigure(1, weight=4, uniform="sq")
        parent.columnconfigure(2, weight=5, uniform="sq")
        parent.rowconfigure(0, weight=1)

        def sub(parent_, title, r):          # muted section sub-header
            ttk.Label(parent_, text=title, style="Col.TLabel").grid(row=r, column=0, sticky="w", pady=(12, 4))

        # ---------- column A : 1 · Library ----------
        A = ttk.LabelFrame(parent, text="  1 · Library  ", style="Card.TLabelframe", padding=(16, 12, 16, 16))
        A.grid(row=0, column=0, sticky="nsew", padx=(16, 8), pady=16)
        A.columnconfigure(0, weight=1); A.rowconfigure(0, weight=1)
        wl = tk.Frame(A, bg=p["border"]); wl.grid(row=0, column=0, sticky="nsew")
        wl.rowconfigure(0, weight=1); wl.columnconfigure(0, weight=1)
        self.ed_poses = tk.Listbox(wl, exportselection=False, activestyle="none", bg=p["surface2"], fg=p["text"],
                                   selectbackground=p["accent_fill"], selectforeground=p["on_accent"],
                                   highlightthickness=0, bd=0, relief="flat", font=self.f_body)
        self.ed_poses.grid(row=0, column=0, sticky="nsew", padx=1, pady=1)
        self.ed_poses.bind("<<ListboxSelect>>", lambda _e: self._ed_pose_selected())
        self.ed_poses.bind("<Double-Button-1>", lambda _e: self._ed_goto())
        ttk.Label(A, text="Double-click a pose = go to it.", style="Muted.TLabel").grid(row=1, column=0, sticky="w", pady=(6, 0))
        ttk.Label(A, text="Pose name", style="Col.TLabel").grid(row=2, column=0, sticky="w", pady=(12, 2))
        self.ed_name = tk.StringVar()
        ttk.Entry(A, textvariable=self.ed_name).grid(row=3, column=0, sticky="ew", pady=(0, 2))
        sub(A, "Edit", 4)
        b1 = ttk.Frame(A, style="TFrame"); b1.grid(row=5, column=0, sticky="w")
        RButton(b1, self, "New", command=self._ed_new, small=True).pack(side="left", padx=(0, 6))
        RButton(b1, self, "Duplicate", command=self._ed_dup, small=True).pack(side="left", padx=(0, 6))
        RButton(b1, self, "Rename", command=self._ed_rename, small=True).pack(side="left", padx=(0, 6))
        RButton(b1, self, "Delete", command=self._ed_del, kind="danger", small=True).pack(side="left")
        sub(A, "File", 6)
        io = ttk.Frame(A, style="TFrame"); io.grid(row=7, column=0, sticky="ew")
        io.columnconfigure((0, 1), weight=1)
        RButton(io, self, "Import…", command=self.import_poses, kind="ghost", small=True).grid(row=0, column=0, sticky="ew", padx=(0, 6))
        RButton(io, self, "Export…", command=self.export_poses, kind="ghost", small=True).grid(row=0, column=1, sticky="ew", padx=(6, 0))

        # ---------- column B : 2 · Edit pose ----------
        B = ttk.LabelFrame(parent, text="  2 · Edit pose  ", style="Card.TLabelframe", padding=(16, 12, 16, 16))
        B.grid(row=0, column=1, sticky="nsew", padx=8, pady=16)
        B.columnconfigure(0, weight=1)
        ttk.Label(B, text="Preview · side view", style="Col.TLabel").grid(row=0, column=0, sticky="w", pady=(0, 4))
        self.ed_cv = tk.Canvas(B, height=200, bg=p["bg"], highlightthickness=1, highlightbackground=p["border"])
        self.ed_cv.grid(row=1, column=0, sticky="ew")
        self.ed_cv.bind("<Configure>", lambda _e: self._ed_draw_preview())
        ttk.Label(B, text="Axes  ·  units 0–1000  ·  indicative angle", style="Col.TLabel").grid(row=2, column=0, sticky="w", pady=(12, 4))
        self.ed_spin = {}; self.ed_deg = {}
        gr = ttk.Frame(B, style="TFrame"); gr.grid(row=3, column=0, sticky="new"); gr.columnconfigure(1, weight=1)
        for i, s in enumerate(SERVOS):
            ttk.Label(gr, text=f"{s} · {NOMS[s][0]} {NOMS[s][1]}", style="TLabel").grid(row=i, column=0, sticky="w", pady=4)
            var = tk.IntVar(value=self.setpoint[s]); self.ed_spin[s] = var
            ttk.Spinbox(gr, from_=0, to=1000, increment=5, width=8, justify="center", textvariable=var).grid(row=i, column=1, sticky="e", padx=8)
            var.trace_add("write", lambda *_a, ss=s: self._ed_spin_changed(ss))
            self.ed_deg[s] = ttk.Label(gr, text="", style="Muted.TLabel", width=6); self.ed_deg[s].grid(row=i, column=2, sticky="e")
            self._ed_deg_one(s)
        sub(B, "Apply", 4)
        b2 = ttk.Frame(B, style="TFrame"); b2.grid(row=5, column=0, sticky="w")
        RButton(b2, self, "⤓ Capture position", command=self._ed_capture, small=True).pack(side="left", padx=(0, 6))
        RButton(b2, self, "Go →", command=self._ed_goto, small=True).pack(side="left", padx=(0, 6))
        RButton(b2, self, "Save", command=self._ed_save, kind="accent", small=True).pack(side="left")

        # ---------- column C : 3 · Sequence (block scenario) ----------
        C = ttk.LabelFrame(parent, text="  3 · Sequence  ", style="Card.TLabelframe", padding=(16, 12, 16, 16))
        C.grid(row=0, column=2, sticky="nsew", padx=(8, 16), pady=16)
        C.columnconfigure(0, weight=1); C.rowconfigure(0, weight=1)
        wr = tk.Frame(C, bg=p["border"]); wr.grid(row=0, column=0, sticky="nsew")
        wr.rowconfigure(0, weight=1); wr.columnconfigure(0, weight=1)
        self.ed_steps = tk.Listbox(wr, exportselection=False, activestyle="none", bg=p["surface2"], fg=p["text"],
                                   selectbackground=p["accent_fill"], selectforeground=p["on_accent"],
                                   highlightthickness=0, bd=0, relief="flat", font=self.f_body)
        self.ed_steps.grid(row=0, column=0, sticky="nsew", padx=1, pady=1)
        self.ed_steps.bind("<<ListboxSelect>>", lambda _e: self._ed_step_selected())
        sub(C, "Add block", 1)
        pal = ttk.Frame(C, style="TFrame"); pal.grid(row=2, column=0, sticky="w")
        for txt, bt in (("+ Pose", "pose"), ("+ Rotor", "rotor"), ("+ Gripper", "gripper"),
                        ("+ Wait", "wait"), ("+ Action", "action")):
            RButton(pal, self, txt, command=lambda b=bt: self._ed_add_block(b), small=True).pack(side="left", padx=(0, 5))
        sub(C, "Arrange", 3)
        b3 = ttk.Frame(C, style="TFrame"); b3.grid(row=4, column=0, sticky="w")
        RButton(b3, self, "↑ Up", command=lambda: self._ed_move_step(-1), small=True).pack(side="left", padx=(0, 6))
        RButton(b3, self, "↓ Down", command=lambda: self._ed_move_step(1), small=True).pack(side="left", padx=(0, 6))
        RButton(b3, self, "Duplicate", command=self._ed_dup_step, small=True).pack(side="left", padx=(0, 6))
        RButton(b3, self, "Remove", command=self._ed_rm_step, kind="danger", small=True).pack(side="left")
        sub(C, "Edit block", 5)
        self.ed_ed = ttk.Frame(C, style="TFrame"); self.ed_ed.grid(row=6, column=0, sticky="w")
        sub(C, "Playback", 7)
        pl = ttk.Frame(C, style="TFrame"); pl.grid(row=8, column=0, sticky="w")
        ttk.Label(pl, text="Loops (0 = ∞)", style="TLabel").grid(row=0, column=0, sticky="w")
        self.ed_loops = tk.IntVar(value=self.seq_loops)
        ttk.Spinbox(pl, from_=0, to=999, increment=1, width=6, textvariable=self.ed_loops).grid(row=0, column=1, padx=6)
        RButton(pl, self, "▶ Play", command=self._ed_play, kind="accent").grid(row=0, column=2, padx=(12, 6))
        RButton(pl, self, "■ Stop", command=self.stop, kind="danger", small=True).grid(row=0, column=3)
        RButton(pl, self, "⌂ Home", command=self.go_home, small=True).grid(row=0, column=4, padx=(8, 0))
        RButton(pl, self, "Release", command=self.release, kind="danger", small=True).grid(row=0, column=5, padx=(8, 0))
        ttk.Label(C, text="Blocks run top-to-bottom · Stop/pause aware · speed curve applies to moves.",
                  style="Muted.TLabel").grid(row=9, column=0, sticky="w", pady=(10, 0))

        self._ed_fill_poses(); self._ed_fill_steps(); self._ed_draw_preview(); self._ed_build_editor()

    # ----- editor helpers -----
    def _ed_fill_poses(self):
        self.ed_poses.delete(0, "end")
        for n in self.poses:
            self.ed_poses.insert("end", n)

    def _ed_fill_steps(self):
        self.ed_steps.delete(0, "end")
        for i, st in enumerate(self.sequence):
            self.ed_steps.insert("end", f"{i + 1}.  {self._block_desc(st)}")
            self.ed_steps.itemconfig(i, foreground=self._block_color(st))

    def _block_desc(self, st):
        def gi(k, d):
            try:
                return int(st.get(k, d))
            except (ValueError, TypeError):
                return d
        t = st.get("type")
        if t == "pose":
            return f"▸ {st.get('pose', '?')}   ·   {gi('dur', 0)} ms"
        if t == "wait":
            return f"⏸ Wait {gi('ms', 0)} ms"
        if t == "rotor":
            s = gi("servo", 6)
            return f"⟳ {NOMS.get(s, ('servo', ''))[0]} → {gi('value', 0)}   ·   {gi('dur', 0)} ms"
        if t == "gripper":
            return f"✊ Gripper: {st.get('mode', '?')}"
        if t == "action":
            return {"home": "⌂ Home", "stow": "↧ Stow", "wave": "〰 Wave",
                    "release": "⏻ Release"}.get(st.get("action"), "Action")
        return "? block"

    def _block_color(self, st):
        p = self.pal
        return {"pose": p["accent_fill"], "rotor": p["live"], "gripper": p["warn"],
                "wait": p["muted"],
                "action": (p["danger"] if st.get("action") == "release" else p["text"])}.get(st.get("type"), p["text"])

    def _ed_deg_one(self, s):
        try:
            self.ed_deg[s].config(text=f"{round(int(self.ed_spin[s].get()) * DEG_PER_UNIT)}°")
        except (tk.TclError, ValueError):
            self.ed_deg[s].config(text="—")

    def _ed_spin_changed(self, s):
        self._ed_deg_one(s)
        self._ed_draw_preview()

    def _ed_draw_preview(self):
        """Quick preview (side view) of the pose being edited, without touching the arm."""
        cv = getattr(self, "ed_cv", None)
        if cv is None or not cv.winfo_exists():
            return
        p = self.pal
        w = cv.winfo_width() or 260; h = cv.winfo_height() or 190
        cv.delete("all")
        for gx in range(0, w, 22):
            cv.create_line(gx, 0, gx, h, fill=p["grid"])
        for gy in range(0, h, 22):
            cv.create_line(0, gy, w, gy, fill=p["grid"])
        reach = sum(SEG_MM.values()) - SEG_MM["riser"]
        f = max(0.05, min((w - 36) / (2 * reach), (h - 30) / reach))
        bx, by = w / 2, h - 14
        rp = reach * f
        cv.create_arc(bx - rp, by - rp, bx + rp, by + rp, start=0, extent=180,
                      style="arc", outline=p["faint"], dash=(2, 5))
        cv.create_line(6, by, w - 6, by, fill=p["faint"])
        round_rect(cv, bx - 18, by, bx + 18, by + 14, 6, fill=p["surface2"], outline=p["border"])
        v = {}
        for s in SERVOS:
            try:
                v[s] = int(self.ed_spin[s].get())
            except (tk.TclError, ValueError):
                v[s] = self.setpoint[s]
        self._chain(cv, v, p["accent_fill"], 6, bx, by, f)
        cv.create_oval(bx - 4, by - 4, bx + 4, by + 4, fill=p["accent_fill"], outline="")

    def _ed_fields(self):
        vals = {}
        for s in SERVOS:
            try:
                vals[s] = self.clamp(s, int(self.ed_spin[s].get()))
            except (tk.TclError, ValueError):
                vals[s] = self.setpoint[s]
        return vals

    def _ed_load_fields(self, pose):
        for s in SERVOS:
            self.ed_spin[s].set(int(pose.get(str(s), pose.get(s, self.setpoint[s]))))

    def _ed_pose_selected(self):
        nom = self._sel(self.ed_poses)
        if nom and nom in self.poses:
            self.ed_name.set(nom); self._ed_load_fields(self.poses[nom])

    def _ed_new(self):
        for s in SERVOS:
            self.ed_spin[s].set(CENTER)
        self.ed_name.set("")

    def _ed_capture(self):
        # Fill immediately from the best current knowledge (feels instant)…
        src = self.reelles if self.reelles else self.setpoint
        for s in SERVOS:
            self.ed_spin[s].set(int(src.get(s, self.setpoint[s])))
        # …then ask for a fresh read so a hand-posed arm captures its real angles.
        if self.connected:
            self._capture_pending = True
            self.request_read()
        else:
            self.log("Arm not connected — captured the commanded position.")

    def _ed_goto(self):
        self._eased_move(self._ed_fields(), note="Preview (editor).")

    def _ed_save(self):
        nom = self.ed_name.get().strip() or self._sel(self.ed_poses) or f"pose {len(self.poses) + 1}"
        self.poses[nom] = {str(s): v for s, v in self._ed_fields().items()}
        self._save(POSES_FILE, self.poses); self._refresh_poses()
        try:
            self.ed_poses.selection_clear(0, "end"); self.ed_poses.selection_set(list(self.poses).index(nom))
        except ValueError:
            pass
        self.log(f"Pose '{nom}' saved.")

    def _ed_dup(self):
        nom = self._sel(self.ed_poses)
        if not nom:
            return
        base = f"{nom} (copy)"; i = 2; new = base
        while new in self.poses:
            new = f"{nom} (copy {i})"; i += 1
        self.poses[new] = dict(self.poses[nom]); self._save(POSES_FILE, self.poses); self._refresh_poses()

    def _ed_rename(self):
        old = self._sel(self.ed_poses); new = self.ed_name.get().strip()
        if not old or not new or new == old:
            return
        if new in self.poses:
            self.log(f"'{new}' already exists.", "warn"); return
        self.poses = {new if k == old else k: v for k, v in self.poses.items()}
        for st in self.sequence:
            if st.get("type") == "pose" and st.get("pose") == old:
                st["pose"] = new
        self._save(POSES_FILE, self.poses); self._ed_persist(); self._refresh_poses(); self._ed_fill_steps()

    def _ed_del(self):
        nom = self._sel(self.ed_poses)
        if not nom or not messagebox.askyesno("Delete", f"Delete pose '{nom}'?"):
            return
        self.poses.pop(nom, None)
        self.sequence = [st for st in self.sequence if not (st.get("type") == "pose" and st.get("pose") == nom)]
        self._save(POSES_FILE, self.poses); self._ed_persist(); self._refresh_poses(); self._ed_fill_steps()
        self._ed_build_editor()

    def _ed_sel_step(self):
        s = self.ed_steps.curselection()
        return s[0] if s else None

    def _ed_int(self, var, default):
        try:
            return int(var.get())
        except (tk.TclError, ValueError):
            return int(default)

    def _ed_add_block(self, btype):
        if btype == "pose":
            nom = self._sel(self.ed_poses) or next(iter(self.poses), None)
            if not nom:
                self.log("No poses yet — save a pose first.", "warn"); return
            blk = {"type": "pose", "pose": nom, "dur": 800}
        elif btype == "rotor":
            blk = {"type": "rotor", "servo": 6, "value": 500, "dur": 600}
        elif btype == "gripper":
            blk = {"type": "gripper", "mode": "open"}
        elif btype == "wait":
            blk = {"type": "wait", "ms": 500}
        elif btype == "action":
            blk = {"type": "action", "action": "home"}
        else:
            return
        i = self._ed_sel_step()
        idx = len(self.sequence) if i is None else i + 1
        self.sequence.insert(idx, blk)
        self._ed_persist(); self._ed_fill_steps()
        self.ed_steps.selection_clear(0, "end"); self.ed_steps.selection_set(idx); self.ed_steps.see(idx)
        self._ed_build_editor()

    def _ed_step_selected(self):
        self._ed_build_editor()

    def _ed_build_editor(self):
        """Rebuild the inline parameter editor for the selected block (varies by block type)."""
        ed = getattr(self, "ed_ed", None)
        if ed is None or not ed.winfo_exists():
            return
        for w in ed.winfo_children():
            w.destroy()
        self.ed_bvars = {}
        i = self._ed_sel_step()
        if i is None or i >= len(self.sequence):
            ttk.Label(ed, text="Select a block to edit its parameters.", style="Muted.TLabel").pack(side="left")
            return
        st = self.sequence[i]; t = st.get("type")

        def spin(label, key, default, frm, to, inc):
            ttk.Label(ed, text=label, style="Col.TLabel").pack(side="left", padx=(0, 4))
            var = tk.IntVar(value=int(st.get(key, default)))
            ttk.Spinbox(ed, from_=frm, to=to, increment=inc, width=7, justify="center", textvariable=var).pack(side="left", padx=(0, 10))
            self.ed_bvars[key] = var

        def combo(label, key, values, cur, width):
            ttk.Label(ed, text=label, style="Col.TLabel").pack(side="left", padx=(0, 4))
            cb = ttk.Combobox(ed, state="readonly", width=width, values=values)
            cb.set(cur); cb.pack(side="left", padx=(0, 10))
            self.ed_bvars[key] = cb

        if t == "pose":
            combo("Pose", "pose", list(self.poses) or ["—"], st.get("pose", "—"), 16)
            spin("Duration ms", "dur", 800, 100, 30000, 100)
        elif t == "wait":
            spin("Wait ms", "ms", 500, 0, 60000, 100)
        elif t == "rotor":
            names = [f"{s} · {NOMS[s][0]} {NOMS[s][1]}" for s in SERVOS]
            cur = int(st.get("servo", 6))
            combo("Servo", "servo", names, f"{cur} · {NOMS[cur][0]} {NOMS[cur][1]}", 18)
            spin("Value 0–1000", "value", 500, 0, 1000, 5)
            spin("Duration ms", "dur", 600, 100, 30000, 100)
        elif t == "gripper":
            combo("Mode", "mode", ["open", "half", "close"], st.get("mode", "open"), 8)
        elif t == "action":
            combo("Action", "action", ["home", "stow", "wave", "release"], st.get("action", "home"), 10)
        RButton(ed, self, "Apply", command=self._ed_apply_block, kind="accent", small=True).pack(side="left", padx=(6, 0))

    def _ed_apply_block(self):
        i = self._ed_sel_step()
        if i is None or i >= len(self.sequence):
            return
        st = self.sequence[i]; t = st.get("type"); bv = self.ed_bvars

        def gi(key, default):
            try:
                return int(bv[key].get())
            except (tk.TclError, ValueError, KeyError):
                return default
        if t == "pose":
            st["pose"] = bv["pose"].get(); st["dur"] = gi("dur", 800)
        elif t == "wait":
            st["ms"] = gi("ms", 500)
        elif t == "rotor":
            try:
                st["servo"] = int(bv["servo"].get().split(" ")[0])
            except (ValueError, KeyError):
                pass
            st["value"] = max(0, min(1000, gi("value", 500))); st["dur"] = gi("dur", 600)
        elif t == "gripper":
            st["mode"] = bv["mode"].get()
        elif t == "action":
            st["action"] = bv["action"].get()
        self._ed_persist(); self._ed_fill_steps()
        self.ed_steps.selection_clear(0, "end"); self.ed_steps.selection_set(i)

    def _ed_move_step(self, d):
        i = self._ed_sel_step()
        if i is None or not (0 <= i + d < len(self.sequence)):
            return
        self.sequence[i], self.sequence[i + d] = self.sequence[i + d], self.sequence[i]
        self._ed_persist(); self._ed_fill_steps()
        self.ed_steps.selection_clear(0, "end"); self.ed_steps.selection_set(i + d); self._ed_build_editor()

    def _ed_dup_step(self):
        i = self._ed_sel_step()
        if i is None:
            return
        self.sequence.insert(i + 1, dict(self.sequence[i]))
        self._ed_persist(); self._ed_fill_steps()
        self.ed_steps.selection_clear(0, "end"); self.ed_steps.selection_set(i + 1); self._ed_build_editor()

    def _ed_rm_step(self):
        i = self._ed_sel_step()
        if i is None:
            return
        self.sequence.pop(i); self._ed_persist(); self._ed_fill_steps(); self._ed_build_editor()

    def _ed_play(self):
        if not self.connected:
            self.log("Arm not connected — cannot play."); return
        suite = []                                             # resolve blocks here (no Tk reads off the worker thread)
        for st in self.sequence:
            t = st.get("type")
            if t == "pose":
                pose = self.poses.get(st.get("pose"))
                if not pose:
                    continue
                cibles = {int(s): self.clamp(int(s), v) for s, v in pose.items()}
                suite.append({"type": "pose", "name": st.get("pose"), "cibles": cibles, "dur": int(st.get("dur", 800))})
            elif t == "wait":
                suite.append({"type": "wait", "ms": max(0, int(st.get("ms", 500)))})
            elif t == "rotor":
                s = int(st.get("servo", 6))
                suite.append({"type": "rotor", "servo": s, "name": NOMS.get(s, ("servo", ""))[0],
                              "value": self.clamp(s, int(st.get("value", 500))), "dur": int(st.get("dur", 600))})
            elif t == "gripper":
                lo, hi = self.limits(1); mode = st.get("mode", "open")
                v = hi if mode == "open" else lo if mode == "close" else round((CENTER + hi) / 2)
                suite.append({"type": "gripper", "mode": mode, "value": self.clamp(1, v), "dur": 600})
            elif t == "action":
                suite.append({"type": "action", "action": st.get("action", "home")})
        if not suite:
            self.log("Empty scenario — add blocks.", "warn"); return
        self.seq_loops = self._ed_int(self.ed_loops, self.seq_loops); self._ed_persist()
        self.stop_flag.clear()
        self.jobs.put(("play", {"suite": suite, "loops": self.seq_loops}))
        self.log(f"Playing scenario: {len(suite)} blocks, loops={self.seq_loops or '∞'}.")

    def _ed_persist(self):
        try:
            self.seq_loops = int(self.ed_loops.get())
        except (tk.TclError, ValueError, AttributeError):
            pass
        self._save(SEQ_FILE, {"steps": self.sequence, "loops": self.seq_loops})

    # ================= persistence =================
    def _load(self, path):
        try:
            if path.exists():
                return json.loads(path.read_text())
        except Exception:
            pass
        return {}

    def _save(self, path, obj):
        try:
            path.write_text(json.dumps(obj, indent=2, ensure_ascii=False))
        except Exception as e:
            self.log(f"Save failed: {e}", "warn")

    def _load_settings(self):
        try:
            if SETTINGS_FILE.exists():
                d = json.loads(SETTINGS_FILE.read_text())
                self.theme = d.get("theme", "dark")
                self.pal = dict(LIGHT if self.theme == "light" else DARK)
                for s in SERVOS:
                    if str(s) in d.get("delta", {}):
                        self.delta[s] = max(0, min(DELTA_LIMIT, int(d["delta"][str(s)])))
                prof = {"lineaire": "Linear", "doux": "Smooth", "Linéaire": "Linear", "Doux": "Smooth",
                        "Accélérer": "Accelerate", "Ralentir": "Decelerate", "Vif": "Snappy",
                        "Ferme": "Firm"}.get(d.get("profile"), d.get("profile"))
                if prof:
                    self.profile = prof
                ez = d.get("ease")
                if isinstance(ez, list) and len(ez) == 4:
                    self.ease_pts = tuple(float(x) for x in ez)
                elif self.profile in EASING_PRESETS:
                    self.ease_pts = EASING_PRESETS[self.profile]
        except Exception:
            pass

    def _save_settings(self):
        try:
            SETTINGS_FILE.write_text(json.dumps({"theme": self.theme, "profile": self.profile,
                                                 "ease": list(self.ease_pts),
                                                 "delta": {str(s): self.delta[s] for s in SERVOS}}, indent=2))
        except Exception:
            pass

    # ================= worker thread =================
    def _post(self, kind, payload=None):
        self.events.put((kind, payload))

    def _scaled(self, dur):
        return max(0, int(dur * 100 / max(1, self.feed)))

    def _sleep_gated(self, ms):
        """Interruptible wait: respects Stop (stop_flag) and Pause (run_gate)."""
        end = max(0, ms) / 1000.0
        slept = 0.0
        while slept < end:
            if self.stop_flag.is_set():
                return
            self.run_gate.wait()
            d = min(0.03, end - slept)
            time.sleep(d); slept += d

    def _mark_down(self, msg):
        self.connected = False; self._post("status", ("ko", msg))

    def _worker(self):
        while True:
            actif = False
            if not self.recording:
                with self.lock:
                    cibles, dur = self.pending, self.pending_dur
                    self.pending = {}
                if cibles and self.connected:
                    try:
                        self.arm.move(cibles, self._scaled(dur))
                    except DEVICE_ERRORS as e:
                        self._mark_down(f"Write failed: {e}")
                    actif = True
            try:
                name, payload = self.jobs.get_nowait()
            except queue.Empty:
                name = None
            if name:
                try:
                    getattr(self, f"_job_{name}")(payload)
                except DEVICE_ERRORS as e:
                    self._mark_down(f"Device error: {e}")
                except Exception as e:
                    self._post("log", f"Error: {e}")
                actif = True
            if not actif:
                time.sleep(0.02)

    def _job_connect(self, payload):
        mode = (payload or {}).get("mode", self.conn_mode)
        try:
            self._post("status", ("wait", f"Connecting ({'Bluetooth' if mode == 'ble' else 'USB'})…"))
            self.arm.connect(make_transport(mode, self.ble_target))
            self.connected = True; self.conn_mode = mode
            self._post("status", ("ok", f"Connected: {self.arm.info}"))
            lues = self.arm.read_positions()
            if lues:
                self._post("positions", lues)
            mv = self.arm.read_battery()
            if mv:
                self._post("battery", mv)
        except Exception as e:
            self.connected = False; self._post("status", ("ko", f"Not connected: {e}"))

    def _job_read(self, _):
        try:
            lues = self.arm.read_positions()
        except DEVICE_ERRORS as e:
            self._mark_down(f"Read failed: {e}"); lues = None
        if lues:
            self._post("positions", lues)
        self._post("read_done")

    def _job_battery(self, _):
        try:
            mv = self.arm.read_battery()
        except DEVICE_ERRORS as e:
            self._mark_down(f"Battery read failed: {e}"); return
        if mv:
            self._post("battery", mv)

    def _goto(self, targets, dur, start=None, log_seg=True):
        if self.recording:
            return 0
        targets = {int(s): self.clamp(int(s), v) for s, v in targets.items()}
        # real interpolation start: provided by the caller (captured BEFORE
        # changing the setpoint), otherwise the current setpoint. Avoids the "jump" when
        # the setpoint has already been placed on the target (e.g. Home) → start == target.
        start = dict(self.setpoint) if start is None else {int(s): int(v) for s, v in start.items()}
        # decide easing on the REQUESTED duration (before speed scaling), otherwise a
        # high speed silently fell back to linear.
        eased = self.profile != "Linear" and self.ease_pts != (0.0, 0.0, 1.0, 1.0) and dur >= 200
        dur = self._scaled(dur)
        blocked = 0                                   # ms actually blocked here (for the caller)
        if eased and dur >= 120:
            p1x, p1y, p2x, p2y = self.ease_pts
            # LONG segments (~100 ms). Each segment is a move(pos, seg) order that the
            # servo runs at speed ≈ distance/seg. Since the distance follows the Bézier
            # curve (small at the ends, large in the middle), the SPEED varies → the arm
            # slows down then speeds up then slows down. Micro-steps too short (~30 ms)
            # were averaged by the controller and gave a constant speed.
            n = max(4, min(14, dur // 100))
            seg = max(60, dur // n)
            if log_seg:
                self._post("log", f"{self.profile}: {n} segments of {seg} ms along the curve.")
            for k in range(1, n + 1):
                if self.stop_flag.is_set():
                    return blocked
                self.run_gate.wait()
                e = bezier_ease(k / n, p1x, p1y, p2x, p2y)
                inter = {s: int(round(start.get(s, v) + (v - start.get(s, v)) * e)) for s, v in targets.items()}
                self.arm.move(inter, seg)
                self._post("command", inter)          # also animates the preview along the curve
                time.sleep(seg / 1000)
                blocked += seg
        else:
            if self.stop_flag.is_set():          # do not start one last move after Stop
                return blocked
            if log_seg and self.profile != "Linear":
                # the operator chose a curve but the duration was too short to segment it
                self._post("log", f"Linear: {dur} ms too short for the '{self.profile}' curve.")
            self.arm.move(targets, dur)
        for s in targets:
            self.torque[s] = True
        self._post("command", targets)
        return blocked

    def _job_goto(self, payload):
        self.stop_flag.clear(); self._goto(payload["targets"], payload.get("duration", 800), payload.get("start"))
        if payload.get("note"):
            self._post("log", payload["note"])

    def _do_park(self):
        """Ordered stow (collision-safe), reusable by the Stow job and the scenario player."""
        self._post("log", "Stowing the arm…")
        for s in STOW_ORDER:
            if self.stop_flag.is_set():
                return
            self.run_gate.wait()
            blocked = self._goto({s: self.clamp(s, STOW[s])}, 900, log_seg=False)
            time.sleep(max(0, self._scaled(900) - blocked) / 1000 + 0.05)   # curve mode already waited
        self._post("log", "Stow complete.")

    def _do_wave(self, servo, cycles, dur, amp):
        s = int(servo)
        lo, hi = self.limits(s)                                    # per-servo software limits
        base = self.reelles.get(s, self.setpoint.get(s, CENTER))
        a = max(lo, min(hi, base - amp)); b = max(lo, min(hi, base + amp))
        self._post("log", f"Wave on axis {s}: {a}↔{b}, {cycles} cycles ({self.profile}).")
        legs = []
        for _ in range(int(cycles)):
            legs += [b, a]
        legs.append(base)                                         # back to center at the end
        prev = base
        for tgt in legs:
            if self.stop_flag.is_set():
                break
            self.run_gate.wait()
            blocked = self._goto({s: tgt}, dur, start={s: prev}, log_seg=False)
            time.sleep(max(0, self._scaled(dur) - blocked) / 1000)  # also paces Linear mode
            prev = tgt
        self._post("log", "Wave complete.")

    def _job_park(self, _):
        self.stop_flag.clear(); self._do_park()

    def _job_wave(self, payload):
        self.stop_flag.clear()
        self._do_wave(int(payload["servo"]), int(payload.get("cycles", 4)),
                      int(payload.get("duration", 600)), int(payload.get("amp", 130)))
        self._post("read")

    def _job_release(self, payload):
        ids = [int(i) for i in (payload.get("ids") or SERVOS) if int(i) in SERVOS]
        self.arm.unload(ids)
        for s in ids:
            self.torque[s] = False
        self._post("torque", {s: False for s in ids})
        self._post("log", "Servos released." if len(ids) == 6 else f"Servo(s) {ids} released.")

    def _job_play(self, payload):
        # suite: list of resolved typed blocks (dicts). loops: number of passes (0 = infinite).
        suite = payload["suite"]
        loops = int(payload.get("loops", 0))
        self.stop_flag.clear()
        tour = 0
        while not self.stop_flag.is_set():
            for blk in suite:
                if self.stop_flag.is_set():
                    break
                self.run_gate.wait()
                t = blk.get("type")
                if t == "wait":
                    self._post("log", f"→ wait {blk['ms']} ms")
                    self._sleep_gated(blk["ms"])
                elif t == "pose":
                    self._post("log", f"→ {blk['name']}")
                    blocked = self._goto(blk["cibles"], blk["dur"], log_seg=False)
                    self._sleep_gated(max(0, self._scaled(blk["dur"]) - blocked))
                elif t == "rotor":
                    self._post("log", f"→ {blk['name']} → {blk['value']}")
                    blocked = self._goto({blk["servo"]: blk["value"]}, blk["dur"], log_seg=False)
                    self._sleep_gated(max(0, self._scaled(blk["dur"]) - blocked))
                elif t == "gripper":
                    self._post("log", f"→ gripper {blk['mode']}")
                    blocked = self._goto({1: blk["value"]}, blk["dur"], log_seg=False)
                    self._sleep_gated(max(0, self._scaled(blk["dur"]) - blocked))
                elif t == "action":
                    a = blk.get("action")
                    if a == "home":
                        self._post("log", "→ Home")
                        blocked = self._goto({s: self.clamp(s, CENTER) for s in SERVOS}, 1500, log_seg=False)
                        self._sleep_gated(max(0, self._scaled(1500) - blocked))
                    elif a == "stow":
                        self._do_park()
                    elif a == "wave":
                        self._do_wave(2, 4, 500, 130)
                    elif a == "release":
                        self._post("log", "→ Release")
                        self.arm.unload(SERVOS)
                        for s in SERVOS:
                            self.torque[s] = False
                        self._post("torque", {s: False for s in SERVOS})
            tour += 1
            if loops > 0 and tour >= loops:
                break
        self._post("log", "Scenario complete."); self._post("read")

    def _job_record(self, _):
        self.arm.unload(SERVOS)
        self.torque = {s: False for s in SERVOS}
        self._post("torque", {s: False for s in SERVOS})
        self.rec_buf = []; self.rec_t0 = time.monotonic(); self.recording = True
        self._post("recording", True)
        while self.recording and not self.stop_flag.is_set():
            try:
                lues = self.arm.read_positions()
            except DEVICE_ERRORS as e:
                self._mark_down(f"Read failed: {e}"); break
            if lues:
                now = time.monotonic(); self._post("positions", lues)
                self.rec_buf.append({"t_ms": int((now - self.rec_t0) * 1000),
                                     "pos": {str(s): int(lues.get(s, 500)) for s in SERVOS}})
                self._post("recframes", len(self.rec_buf))
            time.sleep(0.09)
        kept = []
        for fr in self.rec_buf:
            if not kept or any(abs(int(fr["pos"][str(s)]) - int(kept[-1]["pos"][str(s)])) >= DEADBAND for s in SERVOS):
                kept.append(fr)
        if self.rec_buf and (not kept or kept[-1] is not self.rec_buf[-1]):
            kept.append(self.rec_buf[-1])
        self.last_recording = kept; self.recording = False
        self._post("recording", False)
        self._post("log", f"Recording complete: {len(kept)} frames.")

    def _job_replay(self, payload):
        kf = payload["keyframes"]
        if not kf:
            return
        self.stop_flag.clear()
        prev = kf[0]["t_ms"]
        first = {int(s): self.clamp(int(s), int(v)) for s, v in kf[0]["pos"].items()}
        self.arm.move(first, 300); self._post("command", first); time.sleep(0.3)
        for fr in kf[1:]:
            if self.stop_flag.is_set():
                break
            self.run_gate.wait()
            dt = max(20, fr["t_ms"] - prev); prev = fr["t_ms"]
            tg = {int(s): self.clamp(int(s), int(v)) for s, v in fr["pos"].items()}
            self.arm.move(tg, self._scaled(dt)); self._post("command", tg)
            time.sleep(self._scaled(dt) / 1000)
        for s in SERVOS:
            self.torque[s] = True
        self._post("log", "Replay complete."); self._post("read")

    # ================= event pump =================
    def _pump(self):
        while True:
            try:
                kind, payload = self.events.get_nowait()
            except queue.Empty:
                break
            if kind == "status":
                self._set_status(*payload)
            elif kind == "positions":
                self.reelles.update(payload)
                if getattr(self, "_capture_pending", False) and getattr(self, "ed_spin", None):
                    for s in SERVOS:
                        if s in self.reelles:
                            self.ed_spin[s].set(int(self.reelles[s]))
                    self._capture_pending = False
                    self.log("Live position captured in the editor.")
                if self.follow_var.get() and not self.recording:
                    now = time.monotonic()
                    for s, v in payload.items():
                        if s in self.setpoint and now - self.lastcmd[s] > 1.2:
                            self.setpoint[s] = v; self._set_slider(s, v)
                for s in SERVOS:
                    if s in self.live_lbl:
                        self._render_servo(s)
                self._draw_arm(); self._draw_top()
            elif kind == "command":
                for s, v in payload.items():
                    self.setpoint[s] = v; self._set_slider(s, v); self.torque[s] = True; self._render_servo(s)
                self._draw_arm(); self._draw_top()
            elif kind == "torque":
                self.torque.update(payload)
                for s in payload:
                    self._draw_torque(s)
            elif kind == "recording":
                self.recording = payload
                self.rec_btn.kind = "accent" if payload else "danger"
                self.rec_btn.set_text("■ Stop" if payload else "● Record")
                self.rec_info.config(text=("recording…" if payload else
                                           (f"{len(self.last_recording)} frames — ready to save" if self.last_recording else "—")))
            elif kind == "recframes":
                self.rec_info.config(text=f"{payload} frames")
            elif kind == "read_done":
                self.read_pending = False
            elif kind == "read":
                self.request_read()
            elif kind == "battery":
                self._set_battery(payload)
            elif kind == "log":
                self.log(payload)
        self.after(40, self._pump)

    def _on_close(self):
        self.stop_flag.set(); self.run_gate.set()   # release a paused worker before closing
        self.recording = False; self.arm.close(); self.destroy()


if __name__ == "__main__":
    App().mainloop()
