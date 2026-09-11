#!/usr/bin/env python3
"""
xArm — CONSOLE : serveur local du panneau de contrôle web.

Architecture (validée) :
  • La classe XArm (protocole USB-HID) est reprise telle quelle du GUI Tkinter.
  • UN SEUL thread de travail possède le périphérique HID (non thread-safe,
    lecture bloquante 500 ms) : les threads HTTP ne touchent JAMAIS au device,
    ils ne font qu'empiler des ordres et lire un instantané d'état.
  • Le frontend interroge /state en boucle courte (polling) ; aucune SSE.
  • Toute la sécurité (bornage de course, inversion des servos 2/4/6) est
    appliquée côté serveur : le client est considéré comme non fiable.

Lancement : venv/bin/python web/xarm_server.py
"""

import json
import os
import queue
import secrets
import shutil
import socket
import subprocess
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse, unquote

try:
    import hid
except ImportError:
    hid = None

# Selon la version, hidapi lève HIDException (≠ OSError) sur débranchement :
# on doit attraper les deux, sinon une déconnexion tue le thread de travail.
DEVICE_ERRORS = (OSError,) if hid is None else (OSError, hid.HIDException)

# ----------------------------------------------------------------------
# Protocole (identique au GUI d'origine — ne pas réécrire)
# ----------------------------------------------------------------------
VENDOR_ID = 0x0483
PRODUCT_ID = 0x5750
REPORT_SIZE = 64

CMD_SERVO_MOVE = 0x03
CMD_MULT_SERVO_POS_READ = 0x15
CMD_SERVO_UNLOAD = 0x14

SERVOS = list(range(1, 7))          # 1 = pince ... 6 = base
SAFE_MIN, SAFE_MAX = 200, 800
FULL_MIN, FULL_MAX = 0, 1000
HOME = {s: 500 for s in SERVOS}
DEADBAND = 20

NOMS = {
    1: "pince",
    2: "poignet (rotation)",
    3: "poignet (flexion)",
    4: "coude",
    5: "épaule",
    6: "base",
}

INVERSES = {2, 4, 6}
MIROIR = FULL_MIN + FULL_MAX

# Pose de repli compacte (dans la plage sûre) + ordre de repli sans collision.
STOW = {1: 500, 2: 500, 3: 500, 4: 300, 5: 720, 6: 500}
STOW_ORDER = [3, 4, 5, 6, 2, 1]

BASE_DIR = Path(__file__).resolve().parent
STATIC_DIR = BASE_DIR / "static"
POSES_FILE = Path.home() / ".xarm_poses.json"
REC_FILE = Path.home() / ".xarm_recordings.json"

PREFERRED_PORT = 8787
MAX_BODY = 64 * 1024


def flip(servo_id, pos):
    """Position affichée ↔ position matérielle. Involutif."""
    return MIROIR - pos if servo_id in INVERSES else pos


class XArm:
    """Accès au contrôleur. Utilisé uniquement depuis le thread de travail."""

    def __init__(self):
        self.dev = None
        self.info = ""

    @property
    def ouvert(self):
        return self.dev is not None

    def connect(self):
        if hid is None:
            raise RuntimeError("module 'hid' introuvable — pip install hid")
        self.close()
        self.dev = hid.Device(VENDOR_ID, PRODUCT_ID)
        self.info = f"{self.dev.manufacturer} {self.dev.product}".strip()

    def close(self):
        if self.dev is not None:
            try:
                self.dev.close()
            except Exception:
                pass
        self.dev = None

    def _send(self, cmd, data=()):
        frame = bytes([0x55, 0x55, len(data) + 2, cmd]) + bytes(data)
        self.dev.write(b"\x00" + frame.ljust(REPORT_SIZE, b"\x00"))

    def move(self, cibles, duration):
        duration = max(0, min(30000, int(duration)))
        data = [len(cibles), duration & 0xFF, (duration >> 8) & 0xFF]
        for servo_id, pos in cibles.items():
            pos = flip(servo_id, int(pos))
            data += [servo_id, pos & 0xFF, (pos >> 8) & 0xFF]
        self._send(CMD_SERVO_MOVE, data)

    def unload(self, ids=SERVOS):
        ids = list(ids)
        self._send(CMD_SERVO_UNLOAD, [len(ids)] + ids)

    def read_positions(self, ids=SERVOS):
        ids = list(ids)
        self._send(CMD_MULT_SERVO_POS_READ, [len(ids)] + ids)
        data = self.dev.read(REPORT_SIZE, timeout=500)
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


# ----------------------------------------------------------------------
# Contrôleur : thread de travail + état partagé
# ----------------------------------------------------------------------
class Controller:
    def __init__(self):
        self.arm = XArm()
        self.alive = True

        # tampon de mouvements fusionné (dernier gagnant par servo)
        self.pending = {}
        self.pending_dur = 800
        self.move_lock = threading.Lock()

        self.jobs = queue.Queue()
        self.wake = threading.Event()
        self.stop_flag = threading.Event()
        self.run_gate = threading.Event()   # posé = en marche ; effacé = pause
        self.run_gate.set()

        # état publié
        self.state_lock = threading.Lock()
        self.seq = 0
        self.connected = False
        self.info = ""
        self.error = ""
        self.command = dict(HOME)
        self.read = {}
        self.read_ts = 0.0
        self.torque = {s: True for s in SERVOS}
        self.range_mode = "safe"
        self.limits = {s: dict(min=SAFE_MIN, max=SAFE_MAX) for s in SERVOS}
        self.feed = 100                     # avance globale % (25..200)
        self.profile = "lineaire"           # lineaire | doux
        self.autoread = False
        self.autoread_interval = 1.0
        self.recording = False
        self.replaying = False
        self.playing = False
        self.rec_frames = 0
        self.last_recording = []
        self.log_lines = []                 # (ts, niveau, texte)

        self._read_request = False
        self._rec_buf = []
        self._rec_t0 = 0.0
        self._last_client = time.monotonic()

        self.poses = self._load(POSES_FILE)
        self.recordings = self._load(REC_FILE)

        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        self.jobs.put(("connect", None))
        self.wake.set()

    # --- utilitaires état -------------------------------------------
    def log(self, texte, niveau="info"):
        with self.state_lock:
            self.log_lines.append((time.strftime("%H:%M:%S"), niveau, texte))
            self.log_lines = self.log_lines[-200:]
            self.seq += 1

    def _bump(self):
        with self.state_lock:
            self.seq += 1

    def snapshot(self):
        with self.state_lock:
            return {
                "connected": self.connected,
                "info": self.info,
                "error": self.error,
                "command": {str(s): self.command[s] for s in SERVOS},
                "read": {str(s): v for s, v in self.read.items()},
                "read_age_ms": int((time.monotonic() - self.read_ts) * 1000)
                if self.read_ts else None,
                "torque": {str(s): self.torque[s] for s in SERVOS},
                "range_mode": self.range_mode,
                "limits": {str(s): self.limits[s] for s in SERVOS},
                "feed": self.feed,
                "profile": self.profile,
                "autoread": self.autoread,
                "autoread_interval_ms": int(self.autoread_interval * 1000),
                "recording": self.recording,
                "replaying": self.replaying,
                "playing": self.playing,
                "rec_frames": self.rec_frames,
                "has_recording": len(self.last_recording) > 0,
                "poses": list(self.poses.keys()),
                "recordings": list(self.recordings.keys()),
                "paused": not self.run_gate.is_set(),
                "log": [{"t": t, "n": n, "m": m} for (t, n, m) in self.log_lines[-60:]],
                "seq": self.seq,
                "ts": time.time(),
            }

    def touch_client(self):
        self._last_client = time.monotonic()

    def _clamp(self, s, p):
        lo, hi = self.limits[s]["min"], self.limits[s]["max"]
        return max(lo, min(hi, int(p)))

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
            self.log(f"Enregistrement impossible : {e}", "warn")

    # --- API appelée par les threads HTTP (jamais le device) ---------
    def enqueue_move(self, targets, duration=None):
        clean = {}
        for k, v in targets.items():
            try:
                s = int(k)
            except (TypeError, ValueError):
                continue
            if s in SERVOS:
                clean[s] = self._clamp(s, v)
        if not clean:
            return
        with self.move_lock:
            self.pending.update(clean)
            if duration is not None:
                self.pending_dur = int(duration)
        with self.state_lock:
            self.command.update(clean)
            for s in clean:
                self.torque[s] = True
            self.seq += 1
        self.wake.set()

    def job(self, name, payload=None):
        self.jobs.put((name, payload))
        self.wake.set()

    def set_range(self, mode):
        mode = "full" if mode == "full" else "safe"
        lo, hi = (FULL_MIN, FULL_MAX) if mode == "full" else (SAFE_MIN, SAFE_MAX)
        with self.state_lock:
            self.range_mode = mode
            for s in SERVOS:
                self.limits[s] = dict(min=lo, max=hi)
            self.seq += 1
        if mode == "full":
            self.log("Course étendue 0–1000 : vérifiez les butées mécaniques.", "warn")
        else:
            self.log("Course sûre 200–800.")

    def set_limit(self, s, which, value):
        if s in SERVOS and which in ("min", "max"):
            with self.state_lock:
                self.limits[s][which] = max(FULL_MIN, min(FULL_MAX, int(value)))
                self.seq += 1

    def set_feed(self, feed):
        with self.state_lock:
            self.feed = max(25, min(200, int(feed)))
            self.seq += 1

    def set_profile(self, profile):
        with self.state_lock:
            self.profile = "doux" if profile == "doux" else "lineaire"
            self.seq += 1

    def set_autoread(self, enabled, interval_ms=None):
        with self.state_lock:
            self.autoread = bool(enabled)
            if interval_ms:
                self.autoread_interval = max(0.15, min(5.0, interval_ms / 1000))
            self.seq += 1

    def request_read(self):
        self._read_request = True
        self.wake.set()

    def stop(self):
        self.stop_flag.set()
        self.run_gate.set()
        with self.move_lock:
            self.pending.clear()
        while True:
            try:
                self.jobs.get_nowait()
            except queue.Empty:
                break
        with self.state_lock:
            self.playing = self.replaying = False
            self.seq += 1
        self.log("Arrêt : file de commandes vidée.", "warn")
        self.wake.set()

    def pause(self, paused):
        if paused:
            self.run_gate.clear()
            self.log("Feed-hold : mouvement suspendu.")
        else:
            self.run_gate.set()
            self.log("Reprise.")
        self._bump()

    def save_pose(self, name, positions=None):
        name = (name or "").strip() or f"pose {len(self.poses) + 1}"
        if positions:
            src = positions
        else:
            with self.state_lock:
                src = dict(self.command)
        self.poses[name] = {str(s): int(src[int(s)] if int(s) in src else src.get(str(s), 500))
                            for s in SERVOS}
        self._save(POSES_FILE, self.poses)
        self.log(f"Pose « {name} » enregistrée.")
        self._bump()

    def delete_pose(self, name):
        if self.poses.pop(name, None) is not None:
            self._save(POSES_FILE, self.poses)
            self.log(f"Pose « {name} » supprimée.")
            self._bump()

    def import_poses(self, data):
        if isinstance(data, dict):
            self.poses.update(data)
            self._save(POSES_FILE, self.poses)
            self.log(f"{len(data)} pose(s) importée(s).")
            self._bump()

    def save_recording(self, name):
        name = (name or "").strip() or f"traj {len(self.recordings) + 1}"
        if self.last_recording:
            self.recordings[name] = {"keyframes": self.last_recording}
            self._save(REC_FILE, self.recordings)
            self.log(f"Enregistrement « {name} » ({len(self.last_recording)} images) sauvé.")
            self._bump()

    def delete_recording(self, name):
        if self.recordings.pop(name, None) is not None:
            self._save(REC_FILE, self.recordings)
            self._bump()

    def shutdown(self):
        self.alive = False
        self.stop_flag.set()
        self.run_gate.set()
        self.wake.set()

    # --- boucle de travail (possède le device) -----------------------
    def _run(self):
        while self.alive:
            timeout = 0.1 if self.recording else (
                self.autoread_interval if self.autoread else 0.5)
            self.wake.wait(timeout)
            self.wake.clear()
            if not self.alive:
                break

            if not self.recording:
                self._drain_moves()

            self._service_one_job()

            if self.recording:
                self._sample_recording()
            elif self._read_request or (self.autoread and
                    time.monotonic() - self.read_ts >= self.autoread_interval):
                self._read_request = False
                self._do_read()

        try:
            self.arm.close()
        except Exception:
            pass

    def _drain_moves(self):
        with self.move_lock:
            cibles = self.pending
            self.pending = {}
            dur = self.pending_dur
        if cibles and self.connected:
            try:
                self.arm.move(cibles, self._scaled(dur))
            except DEVICE_ERRORS as e:
                self._mark_down(f"Écriture impossible : {e}")

    def _service_one_job(self):
        try:
            name, payload = self.jobs.get_nowait()
        except queue.Empty:
            return
        try:
            getattr(self, f"_job_{name}")(payload)
        except DEVICE_ERRORS as e:
            self._mark_down(f"Erreur périphérique : {e}")
        except Exception as e:
            self.log(f"Erreur : {e}", "warn")

    def _scaled(self, dur):
        with self.state_lock:
            feed = self.feed
        return max(0, int(dur * 100 / max(1, feed)))

    def _mark_down(self, msg):
        with self.state_lock:
            self.connected = False
            self.error = msg
            self.seq += 1
        self.log(msg, "warn")

    def _do_read(self):
        if not self.connected:
            return
        try:
            lues = self.arm.read_positions()
        except DEVICE_ERRORS as e:
            self._mark_down(f"Lecture impossible : {e}")
            return
        if lues:
            with self.state_lock:
                self.read.update(lues)
                self.read_ts = time.monotonic()
                self.seq += 1

    def _sample_recording(self):
        if not self.connected:
            return
        try:
            lues = self.arm.read_positions()
        except DEVICE_ERRORS as e:
            self._mark_down(f"Lecture impossible : {e}")
            return
        if lues:
            now = time.monotonic()
            with self.state_lock:
                self.read.update(lues)
                self.read_ts = now
            frame = {"t_ms": int((now - self._rec_t0) * 1000),
                     "pos": {str(s): int(lues.get(s, self.read.get(s, 500)))
                             for s in SERVOS}}
            self._rec_buf.append(frame)
            with self.state_lock:
                self.rec_frames = len(self._rec_buf)
                self.seq += 1

    def _goto(self, targets, dur, note=None):
        """Mouvement discret, avec profil d'accélération optionnel."""
        if self.recording:
            return  # servos relâchés pour l'apprentissage : ne pas les ré-engager
        targets = {int(s): self._clamp(int(s), v) for s, v in targets.items()}
        with self.state_lock:
            start = dict(self.command)
            profile = self.profile
        dur = self._scaled(dur)
        if profile == "doux" and dur >= 250:
            n = max(2, min(30, dur // 40))
            for k in range(1, n + 1):
                if self.stop_flag.is_set():
                    return
                self.run_gate.wait()
                f = k / n
                e = f * f * (3 - 2 * f)          # smoothstep (courbe en S)
                inter = {s: int(round(start.get(s, v) + (v - start.get(s, v)) * e))
                         for s, v in targets.items()}
                self.arm.move(inter, max(1, dur // n))
                time.sleep(max(1, dur // n) / 1000)
        else:
            self.arm.move(targets, dur)
        with self.state_lock:
            self.command.update(targets)
            for s in targets:
                self.torque[s] = True
            self.seq += 1
        if note:
            self.log(note)

    # --- jobs (exécutés sur le thread de travail) --------------------
    def _job_connect(self, _):
        try:
            self.arm.connect()
            with self.state_lock:
                self.connected = True
                self.info = self.arm.info
                self.error = ""
                self.seq += 1
            self.log(f"Connecté : {self.arm.info}")
            self._do_read()
        except Exception as e:
            with self.state_lock:
                self.connected = False
                self.error = str(e)
                self.seq += 1
            self.log(f"Non connecté : {e}", "warn")

    def _job_home(self, _):
        self.stop_flag.clear()
        self._goto(dict(HOME), 1500, "Retour Home.")

    def _job_goto(self, payload):
        self.stop_flag.clear()
        self._goto(payload["targets"], payload.get("duration", 800),
                   payload.get("note"))

    def _job_release(self, payload):
        ids = payload.get("ids") if payload else None
        ids = ids or SERVOS
        ids = [int(i) for i in ids if int(i) in SERVOS]
        self.arm.unload(ids)
        with self.state_lock:
            for s in ids:
                self.torque[s] = False
            self.seq += 1
        if len(ids) == len(SERVOS):
            self.set_autoread(True, 800)
            self.log("Servos relâchés. Bougez le bras : la lecture suit.")
        else:
            noms = ", ".join(str(i) for i in ids)
            self.log(f"Servo(s) {noms} relâché(s).")

    def _job_park(self, payload):
        self.stop_flag.clear()
        self.log("Repli sécurisé…")
        for s in STOW_ORDER:
            if self.stop_flag.is_set():
                self.log("Repli interrompu.", "warn")
                return
            self.run_gate.wait()
            self._goto({s: STOW[s]}, 900)
            time.sleep(self._scaled(900) / 1000 + 0.05)
        if payload and payload.get("release"):
            self.arm.unload(SERVOS)
            with self.state_lock:
                for s in SERVOS:
                    self.torque[s] = False
                self.seq += 1
            self.log("Arrêt sécurisé : bras au repli puis relâché.")
        else:
            self.log("Repli terminé.")

    def _job_play_poses(self, payload):
        dur = payload.get("duration", 800)
        pause = payload.get("pause", 300)
        loop = payload.get("loop", False)
        names = payload.get("names") or list(self.poses.keys())
        if not names:
            return
        self.stop_flag.clear()
        with self.state_lock:
            self.playing = True
            self.seq += 1
        self.log(f"Lecture de {len(names)} pose(s).")
        try:
            first = True
            while (first or loop) and not self.stop_flag.is_set():
                first = False
                for nom in names:
                    if self.stop_flag.is_set():
                        break
                    self.run_gate.wait()
                    pose = self.poses.get(nom)
                    if not pose:
                        continue
                    targets = {int(s): p for s, p in pose.items()}
                    self._goto(targets, dur)
                    time.sleep((self._scaled(dur) + pause) / 1000)
        finally:
            with self.state_lock:
                self.playing = False
                self.seq += 1
            self.log("Séquence terminée.")

    def _job_record_start(self, _):
        self.arm.unload(SERVOS)
        self._rec_buf = []
        self._rec_t0 = time.monotonic()
        with self.state_lock:
            self.recording = True
            self.rec_frames = 0
            for s in SERVOS:
                self.torque[s] = False
            self.seq += 1
        self.set_autoread(False)
        self.log("Enregistrement gestuel : bougez le bras à la main.")

    def _job_record_stop(self, _):
        buf = self._rec_buf
        kept = []
        for fr in buf:
            if not kept:
                kept.append(fr)
                continue
            last = kept[-1]["pos"]
            if any(abs(int(fr["pos"][str(s)]) - int(last[str(s)])) >= DEADBAND
                   for s in SERVOS):
                kept.append(fr)
        if buf and kept and kept[-1] is not buf[-1]:
            kept.append(buf[-1])
        self.last_recording = kept
        with self.state_lock:
            self.recording = False
            self.rec_frames = len(kept)
            self.seq += 1
        dur = kept[-1]["t_ms"] if kept else 0
        self.log(f"Enregistrement terminé : {len(kept)} images, {dur} ms.")

    def _job_replay(self, payload):
        if self.recording:
            return  # pas de rejeu pendant un enregistrement gestuel
        kf = payload.get("keyframes")
        if not kf and payload.get("name"):
            rec = self.recordings.get(payload["name"])
            kf = rec.get("keyframes") if rec else None
        if not kf:
            kf = self.last_recording
        if not kf:
            return
        self.stop_flag.clear()
        with self.state_lock:
            self.replaying = True
            self.seq += 1
        self.log(f"Rejeu de {len(kf)} images.")
        try:
            prev_t = kf[0]["t_ms"]
            first = {int(s): int(v) for s, v in kf[0]["pos"].items()}
            self.arm.move({s: self._clamp(s, v) for s, v in first.items()}, 300)
            time.sleep(0.3)
            for fr in kf[1:]:
                if self.stop_flag.is_set():
                    break
                self.run_gate.wait()
                dt = max(20, fr["t_ms"] - prev_t)
                prev_t = fr["t_ms"]
                tg = {int(s): self._clamp(int(s), int(v)) for s, v in fr["pos"].items()}
                self.arm.move(tg, self._scaled(dt))
                with self.state_lock:
                    self.command.update(tg)
                    self.seq += 1
                time.sleep(self._scaled(dt) / 1000)
        finally:
            with self.state_lock:
                self.replaying = False
                self.seq += 1
            self.log("Rejeu terminé.")


# ----------------------------------------------------------------------
# Serveur HTTP
# ----------------------------------------------------------------------
class Handler(BaseHTTPRequestHandler):
    server_version = "xArmConsole/1.0"
    protocol_version = "HTTP/1.1"

    # -- garde-fous sécurité --
    def _host_ok(self):
        host = self.headers.get("Host", "")
        return host in self.server.allowed_hosts

    def _origin_ok(self):
        origin = self.headers.get("Origin")
        if origin is None:
            return True                      # requêtes non-navigateur (curl) : pas de CSRF
        return origin in self.server.allowed_origins

    def _token_ok(self):
        return self.headers.get("X-XArm-Token") == self.server.token

    def log_message(self, *a):
        pass                                 # silencieux

    # -- helpers --
    def _send_json(self, obj, code=200):
        body = json.dumps(obj).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _send_file(self, path, ctype):
        try:
            data = path.read_bytes()
        except OSError:
            self.send_error(404)
            return
        if path.name == "index.html":
            data = data.replace(b"__XARM_TOKEN__", self.server.token.encode())
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def _body_json(self):
        try:
            n = int(self.headers.get("Content-Length", 0) or 0)
        except (TypeError, ValueError):
            return None
        if n <= 0:
            return {}
        if n > MAX_BODY:
            return None
        raw = self.rfile.read(n)
        try:
            return json.loads(raw or b"{}")
        except (ValueError, UnicodeDecodeError):
            return None

    # -- GET --
    def do_GET(self):
        if not self._host_ok():
            self.send_error(403)
            return
        path = urlparse(self.path).path
        ctrl = self.server.ctrl

        if path == "/state":
            ctrl.touch_client()
            self._send_json(ctrl.snapshot())
            return
        if path == "/poses/export":
            self._send_json(ctrl.poses)
            return

        static = {
            "/": (STATIC_DIR / "index.html", "text/html; charset=utf-8"),
            "/index.html": (STATIC_DIR / "index.html", "text/html; charset=utf-8"),
            "/style.css": (STATIC_DIR / "style.css", "text/css; charset=utf-8"),
            "/app.js": (STATIC_DIR / "app.js", "application/javascript; charset=utf-8"),
        }
        if path in static:
            self._send_file(*static[path])
            return
        self.send_error(404)

    # -- POST / DELETE --
    def do_POST(self):
        path = urlparse(self.path).path
        # /shutdown arrive via navigator.sendBeacon (pas d'en-tête possible) :
        # protégé par Host+Origin, dispensé du jeton.
        need_token = path != "/shutdown"
        if not self._host_ok() or not self._origin_ok() or (need_token and not self._token_ok()):
            self.send_error(403)
            return
        ctrl = self.server.ctrl
        ctrl.touch_client()
        body = self._body_json()
        if body is None:
            self.send_error(400)
            return

        parts = [p for p in path.split("/") if p]

        try:
            if path == "/move":
                ctrl.enqueue_move(body.get("targets", {}), body.get("duration_ms"))
            elif path == "/read":
                ctrl.request_read()
            elif path == "/home":
                ctrl.job("home")
            elif path == "/goto":
                ctrl.job("goto", {"targets": body.get("targets", {}),
                                  "duration": body.get("duration_ms", 800),
                                  "note": body.get("note")})
            elif path == "/release":
                ctrl.job("release", {"ids": body.get("ids")})
            elif path == "/park":
                ctrl.job("park", {"release": bool(body.get("release"))})
            elif path == "/stop":
                ctrl.stop()
            elif path == "/pause":
                ctrl.pause(bool(body.get("paused", True)))
            elif path == "/reconnect":
                ctrl.job("connect")
            elif path == "/range":
                ctrl.set_range(body.get("mode", "safe"))
            elif path == "/limit":
                ctrl.set_limit(int(body["servo"]), body["which"], body["value"])
            elif path == "/feed":
                ctrl.set_feed(body.get("feed", 100))
            elif path == "/profile":
                ctrl.set_profile(body.get("profile", "lineaire"))
            elif path == "/autoread":
                ctrl.set_autoread(body.get("enabled", False),
                                  body.get("interval_ms"))
            elif path == "/poses":
                ctrl.save_pose(body.get("name"), body.get("positions"))
            elif path == "/poses/import":
                ctrl.import_poses(body)
            elif path == "/poses/play":
                ctrl.job("play_poses", {"duration": body.get("duration_ms", 800),
                                        "pause": body.get("pause_ms", 300),
                                        "loop": bool(body.get("loop")),
                                        "names": body.get("names")})
            elif len(parts) == 3 and parts[0] == "poses" and parts[2] == "goto":
                nom = unquote(parts[1])
                pose = ctrl.poses.get(nom)
                if pose:
                    ctrl.job("goto", {"targets": {int(s): p for s, p in pose.items()},
                                      "duration": body.get("duration_ms", 800),
                                      "note": f"Pose « {nom} »."})
            elif path == "/record/start":
                ctrl.job("record_start")
            elif path == "/record/stop":
                ctrl.job("record_stop")
            elif path == "/recordings":
                ctrl.save_recording(body.get("name"))
            elif path == "/replay":
                ctrl.job("replay", {"name": body.get("name")})
            elif path == "/shutdown":
                self._send_json({"ok": True})
                self.server.request_shutdown()
                return
            else:
                self.send_error(404)
                return
        except (KeyError, ValueError, TypeError) as e:
            self.send_error(400, str(e))
            return
        self._send_json({"ok": True}, 202)

    def do_DELETE(self):
        if not self._host_ok() or not self._origin_ok() or not self._token_ok():
            self.send_error(403)
            return
        parts = [p for p in urlparse(self.path).path.split("/") if p]
        ctrl = self.server.ctrl
        if len(parts) == 2 and parts[0] == "poses":
            ctrl.delete_pose(unquote(parts[1]))
        elif len(parts) == 2 and parts[0] == "recordings":
            ctrl.delete_recording(unquote(parts[1]))
        else:
            self.send_error(404)
            return
        self._send_json({"ok": True})


class ArmServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, addr, ctrl, token):
        super().__init__(addr, Handler)
        self.ctrl = ctrl
        self.token = token
        port = self.server_address[1]
        self.allowed_hosts = {f"127.0.0.1:{port}", f"localhost:{port}"}
        self.allowed_origins = {f"http://127.0.0.1:{port}", f"http://localhost:{port}"}
        self._shutting = False

    def request_shutdown(self):
        if not self._shutting:
            self._shutting = True
            self.ctrl.shutdown()
            threading.Thread(target=self.shutdown, daemon=True).start()


# ----------------------------------------------------------------------
# Lancement navigateur (mode application si possible)
# ----------------------------------------------------------------------
def open_browser(url):
    for exe in ("google-chrome", "google-chrome-stable", "chromium",
                "chromium-browser", "brave-browser", "microsoft-edge"):
        p = shutil.which(exe)
        if p:
            try:
                subprocess.Popen([p, f"--app={url}", "--new-window"],
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                return f"{exe} --app"
            except OSError:
                pass
    ff = shutil.which("firefox")
    if ff:
        prof = _firefox_app_profile(url)
        try:
            if prof:
                subprocess.Popen([ff, "--no-remote", "--profile", prof,
                                  "--new-window", url],
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                return "firefox (profil app)"
            subprocess.Popen([ff, "--new-window", url],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return "firefox"
        except OSError:
            pass
    try:
        webbrowser.open(url)
        return "webbrowser"
    except Exception:
        return "aucun"


def _firefox_app_profile(url):
    """Profil Firefox dédié + userChrome.css masquant la barre d'outils
    pour un rendu proche d'une application. Meilleur effort."""
    try:
        prof = Path.home() / ".xarm" / "ff-profile"
        (prof / "chrome").mkdir(parents=True, exist_ok=True)
        (prof / "user.js").write_text(
            'user_pref("toolkit.legacyUserProfileCustomizations.stylesheets", true);\n'
            'user_pref("browser.tabs.warnOnClose", false);\n'
            'user_pref("browser.shell.checkDefaultBrowser", false);\n'
            'user_pref("browser.aboutwelcome.enabled", false);\n'
            'user_pref("datareporting.policy.dataSubmissionEnabled", false);\n'
            'user_pref("browser.startup.homepage_override.mstone", "ignore");\n'
        )
        (prof / "chrome" / "userChrome.css").write_text(
            "#TabsToolbar, #nav-bar, #navigator-toolbox > * { visibility: collapse !important; }\n"
            "#navigator-toolbox { max-height: 0 !important; min-height: 0 !important; }\n"
        )
        return str(prof)
    except OSError:
        return None


def watchdog(ctrl, server):
    """Ferme le serveur quand la fenêtre est fermée (silence de polling)."""
    grace = 20.0
    while ctrl.alive:
        time.sleep(2.0)
        if time.monotonic() - ctrl._last_client > grace:
            server.request_shutdown()
            return


def main():
    if hid is None:
        print("Attention : module 'hid' absent — l'interface se lance mais "
              "le bras restera « hors ligne ».", file=sys.stderr)
    token = secrets.token_urlsafe(24)
    ctrl = Controller()

    port = int(os.environ.get("XARM_PORT", PREFERRED_PORT))
    try:
        server = ArmServer(("127.0.0.1", port), ctrl, token)
    except OSError:
        server = ArmServer(("127.0.0.1", 0), ctrl, token)
        port = server.server_address[1]

    url = f"http://127.0.0.1:{port}/"
    if os.environ.get("XARM_NO_BROWSER"):
        mode = "désactivé (XARM_NO_BROWSER)"
    else:
        mode = open_browser(url)
    print(f"xArm Console → {url}  (navigateur : {mode})", flush=True)
    threading.Thread(target=watchdog, args=(ctrl, server), daemon=True).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        server.request_shutdown()
    finally:
        ctrl.shutdown()


if __name__ == "__main__":
    main()
