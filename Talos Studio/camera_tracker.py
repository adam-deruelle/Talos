#!/usr/bin/env python3
"""xArm camera tracker — read the phone's DroidCam MJPEG, find a fixed ArUco marker,
compute the phone's 3D position relative to it, and stream it to the xArm app over UDP as
"P,x,y,z" (mm). Non-AR: plain single-marker pose estimation. Run separately from the app:

    XARM_CAMERA_URL=http://<phone-ip>:4747/video venv/bin/python camera_tracker.py [udp_port]

Requires opencv-contrib-python (installed in venv). The app listens on UDP 8899.
Use the marker in scratchpad/aruco_marker_id0.png (DICT_4X4_50 id 0) printed/shown flat.
"""
import os
import socket
import sys
import time

import cv2
import cv2.aruco as aruco
import numpy as np

URL = os.environ.get("XARM_CAMERA_URL", "")  # DroidCam MJPEG stream, e.g. http://<phone-ip>:4747/video
MARKER_MM = 52.6                            # physical marker side length (mm)
DEST = ("127.0.0.1", int(sys.argv[1]) if len(sys.argv) > 1 else 8899)
JUMP_MM = 250.0                             # reject implausible per-frame jumps (outliers)
ALPHA = 0.5                                 # light smoothing at the source

if not URL:
    sys.exit("set XARM_CAMERA_URL to the MJPEG stream, e.g. http://<phone-ip>:4747/video")

DICT = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
det = aruco.ArucoDetector(DICT, aruco.DetectorParameters())
hm = MARKER_MM / 2.0
OBJ = np.array([[-hm, hm, 0], [hm, hm, 0], [hm, -hm, 0], [-hm, -hm, 0]], dtype=np.float32)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def open_cap():
    while True:
        cap = cv2.VideoCapture(URL)
        if cap.isOpened():
            try:
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)     # keep latency low (don't queue stale frames)
            except Exception:
                pass
            ok, fr = cap.read()
            if ok:
                return cap, fr
            cap.release()
        print("waiting for DroidCam stream at", URL, "…"); time.sleep(1.5)


cap, frame = open_cap()
H, W = frame.shape[:2]
f = 0.9 * W
K = np.array([[f, 0, W / 2], [0, f, H / 2], [0, 0, 1]], dtype=np.float32)
D = np.zeros(5, dtype=np.float32)
print(f"tracker up: {W}x{H}, marker {MARKER_MM}mm → streaming 'P,x,y,z' to {DEST[0]}:{DEST[1]}")

prev = None
smooth = None
lost = 0
while True:
    ok, frame = cap.read()
    if not ok:
        lost += 1
        if lost > 30:
            cap.release(); cap, frame = open_cap(); lost = 0
        continue
    lost = 0
    corners, ids, _ = det.detectMarkers(frame)
    if ids is None or 0 not in ids.flatten():
        continue
    c = corners[list(ids.flatten()).index(0)][0].astype(np.float32)
    ok2, rvec, tvec = cv2.solvePnP(OBJ, c, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok2:
        continue
    R, _ = cv2.Rodrigues(rvec)
    pos = (-R.T @ tvec).flatten()                       # phone position in marker frame (mm)
    if prev is not None and np.linalg.norm(pos - prev) > JUMP_MM:
        continue                                        # drop outlier
    prev = pos
    smooth = pos if smooth is None else ALPHA * pos + (1 - ALPHA) * smooth
    sock.sendto(f"P,{smooth[0]:.1f},{smooth[1]:.1f},{smooth[2]:.1f}".encode("ascii"), DEST)
