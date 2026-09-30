#!/usr/bin/env python3
"""vlm-view — the BIG demo window: LIVE video + latest VLM overlay.

The annotated topic only updates once per VLM cycle (~10s) — fine for rviz,
too laggy for a demo screen. This viewer instead renders the LIVE camera
stream (~30fps) and overlays the most recent perception result on top:
grounding boxes, measured distances, description, OCR, and the age of the
analysis. Qt on purpose — OpenCV's GTK highgui aborts in this container.

Keys:  f = toggle fullscreen   q / Esc = close
"""

import json
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtGui import QImage, QPixmap
from python_qt_binding.QtWidgets import QApplication, QLabel
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

from vlm_perception.common import DEFAULT_CONFIG, load_config

ACCENT = (0, 220, 255)   # BGR — VLM (semantic) color
DET_COLOR = (80, 220, 80)  # BGR — detector (real-time) color
BOX_FRESH_S = 4.0    # VLM boxes full-strength this long after an analysis
BOX_TTL_S = 10.0     # then fade; NEVER drawn past this age (stale geometry lies)
DET_TTL_S = 1.0      # detector boxes are real-time; older than this = camera issue


def _iou(a, b):
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0, ix2 - ix1) * max(0, iy2 - iy1)
    ua = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - inter
    return inter / ua if ua > 0 else 0.0


class FeedSub(Node):
    """Live frames + latest perception payload, latest-wins slots."""

    def __init__(self, cfg: dict):
        super().__init__("vlm_view")
        self.bridge = CvBridge()
        self._lock = threading.Lock()
        self._frame = None          # live BGR frame
        self._jpeg = None           # fallback: annotated jpeg (RGB-only mode)
        self._result, self._result_ts = None, 0.0
        self._early_desc, self._early_ts = None, 0.0

        self._dets, self._dets_ts = None, 0.0
        det_topic = (cfg.get("detector") or {}).get("publish_topic", "/vlm/detections")
        self.create_subscription(String, det_topic, self._on_dets, 10)
        self.live = bool(cfg["depth"].get("enabled"))
        if self.live:  # live color stream exists only when the driver runs
            self.create_subscription(Image, cfg["depth"]["color_topic"],
                                     self._on_color, qos_profile_sensor_data)
        else:          # RGB-only mode: fall back to the per-cycle annotated frame
            self.create_subscription(CompressedImage, cfg["output"]["annotated_topic"],
                                     self._on_jpeg, qos_profile_sensor_data)
        self.create_subscription(String, cfg["output"]["perception_topic"],
                                 self._on_result, 10)
        # Early channel: app.py streams the reply and publishes the description
        # the moment its field completes — seconds before the full payload.
        self.create_subscription(String, cfg["output"]["description_topic"],
                                 self._on_early_desc, 10)
        self.get_logger().info(f"[vlm-view] live={self.live} — f=fullscreen, q=quit")

    def _on_color(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        with self._lock:
            self._frame = frame

    def _on_jpeg(self, msg: CompressedImage):
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is not None:
            with self._lock:
                self._frame = frame

    def _on_dets(self, msg: String):
        try:
            d = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._lock:
            self._dets, self._dets_ts = d, time.time()

    def _on_result(self, msg: String):
        try:
            r = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._lock:
            self._result, self._result_ts = r, time.time()

    def _on_early_desc(self, msg: String):
        # Streamed description — kept SEPARATE from the full result on purpose:
        # merging it would reset the result timestamp and rejuvenate stale VLM
        # boxes (ghost boxes). Banner text updates early; box aging stays honest.
        if msg.data:
            with self._lock:
                self._early_desc, self._early_ts = msg.data, time.time()

    def snapshot(self):
        with self._lock:
            result, ts, desc_ts = self._result, self._result_ts, self._result_ts
            if self._early_desc and self._early_ts > ts:
                result = dict(result or {}, description=self._early_desc)
                result.setdefault("parse_ok", True)
                desc_ts = self._early_ts
            return (self._frame, result, ts, self._dets, self._dets_ts, desc_ts)


def _wrap_text(text, max_width_px, font_scale=0.55, thickness=1):
    """Word-wrap text to a pixel width for the banner (measured, not guessed)."""
    words = str(text).split()
    lines, cur = [], ""
    for word in words:
        trial = (cur + " " + word).strip()
        tw = cv2.getTextSize(trial, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)[0][0]
        if tw <= max_width_px or not cur:
            cur = trial
        else:
            lines.append(cur)
            cur = word
    if cur:
        lines.append(cur)
    return lines or [""]


def draw_overlay(frame, result, age_s: float, dets=None, dets_age: float = 1e9,
                 desc_age: float = None):
    """Two-tier overlay: detector boxes are REAL-TIME (move with people, carry
    live depth); VLM adds semantics — matched onto detector tracks by overlap
    when both exist, else the VLM's own (TTL-limited) boxes are shown.
    desc_age: freshness of the (possibly streamed-early) description — boxes
    age by age_s, the banner by desc_age, so an early description can never
    rejuvenate stale boxes."""
    h, w = frame.shape[:2]

    det_objs = (dets or {}).get("objects") or [] if dets_age <= DET_TTL_S else []
    vlm_objs = [o for o in (result.get("objects") or []) if isinstance(o, dict)] if result else []

    if det_objs:
        for o in det_objs:
            box = o.get("box_2d")
            if not box or len(box) != 4:
                continue
            x1, y1 = int(box[0] / 1000 * w), int(box[1] / 1000 * h)
            x2, y2 = int(box[2] / 1000 * w), int(box[3] / 1000 * h)
            cv2.rectangle(frame, (x1, y1), (x2, y2), DET_COLOR, 2)
            label = o.get("cls", "?")
            if o.get("measured_distance_m") is not None:
                label += f" {o['measured_distance_m']}m"
            # semantic enrichment: best-overlapping VLM object's name
            best, best_iou = None, 0.3
            for v in vlm_objs:
                vb = v.get("box_2d")
                if vb and len(vb) == 4:
                    i = _iou(box, vb)
                    if i > best_iou:
                        best, best_iou = v, i
            if best and str(best.get("name", "")).lower() not in label.lower():
                label += f"  [{best['name']}]"
            cv2.putText(frame, label, (x1 + 3, max(16, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, DET_COLOR, 2, cv2.LINE_AA)
        fps = (dets or {}).get("fps")
        if fps:
            cv2.putText(frame, f"detector {fps}fps", (w - 190, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, DET_COLOR, 2, cv2.LINE_AA)

    if result and not det_objs:
        # Stale-box policy: boxes are only valid for the frame the VLM saw.
        # Fresh -> full strength; aging -> dimmed + [stale] tag; past TTL ->
        # NOT drawn (a box floating where a person no longer stands is a lie).
        if age_s <= BOX_FRESH_S:
            box_color, tag = ACCENT, ""
        elif age_s <= BOX_TTL_S:
            box_color, tag = (0, 110, 128), " [stale]"
        else:
            box_color = None
        if box_color is not None:
            for o in result.get("objects") or []:
                box = o.get("box_2d") if isinstance(o, dict) else None
                if not box or len(box) != 4:
                    continue
                try:
                    x1, y1 = int(box[0] / 1000 * w), int(box[1] / 1000 * h)
                    x2, y2 = int(box[2] / 1000 * w), int(box[3] / 1000 * h)
                except (TypeError, ValueError):
                    continue
                cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 2)
                dist = o.get("measured_distance_m", o.get("approx_distance_m"))
                label = (f"{o.get('name', '?')}"
                         + (f" {dist}m" if dist is not None else "") + tag)
                cv2.putText(frame, label, (x1 + 3, max(16, y1 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, box_color, 2, cv2.LINE_AA)
    if result:
        lines = []
        desc = result.get("description") or result.get("status", "")
        if desc:
            # FULL response, word-wrapped to the window width (no truncation);
            # sanity cap so a pathological reply cannot swallow the video area
            lines += _wrap_text(str(desc), w - 20)[:8]
        if result.get("visible_text"):
            ocr = "TEXT: " + " | ".join(map(str, result["visible_text"]))
            lines += _wrap_text(ocr, w - 20)[:4]
        banner_age = desc_age if desc_age is not None else age_s
        lines.append(f"AI updated {banner_age:.0f}s ago" if banner_age < 900
                     else "waiting for first analysis...")
        y0 = h - 22 * len(lines) - 8
        cv2.rectangle(frame, (0, y0 - 6), (w, h), (0, 0, 0), -1)
        y = y0 + 14
        for ln in lines:
            cv2.putText(frame, ln, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (255, 255, 255), 1, cv2.LINE_AA)
            y += 22
    return frame


class ViewWindow(QLabel):
    def __init__(self, sub: FeedSub):
        super().__init__("waiting for camera ...")
        self._sub = sub
        self._frames, self._t0 = 0, time.time()
        self.setWindowTitle("WEDA VLM - Live Annotated View  (f=fullscreen, q=quit)")
        self.setAlignment(Qt.AlignCenter)
        self.setStyleSheet("background-color: #1e1e1e; color: #aaaaaa; font-size: 20px;")
        self.setMinimumSize(640, 400)
        self.resize(1280, 760)
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(33)   # ~30fps render loop

    def _refresh(self):
        frame, result, ts, dets, dts, desc_ts = self._sub.snapshot()
        if frame is None:
            return
        img = draw_overlay(frame.copy(), result, time.time() - ts if ts else 1e9,
                           dets, time.time() - dts if dts else 1e9,
                           time.time() - desc_ts if desc_ts else None)
        h, w = img.shape[:2]
        qimg = QImage(np.ascontiguousarray(img).data, w, h, 3 * w, QImage.Format_BGR888)
        self.setPixmap(QPixmap.fromImage(qimg).scaled(
            self.size(), Qt.KeepAspectRatio, Qt.FastTransformation))
        self._frames += 1
        if time.time() - self._t0 >= 10:
            self._sub.get_logger().info(
                f"[vlm-view] render fps: {self._frames / (time.time() - self._t0):.1f}")
            self._frames, self._t0 = 0, time.time()

    def keyPressEvent(self, ev):
        if ev.key() in (Qt.Key_Q, Qt.Key_Escape):
            self.close()
        elif ev.key() == Qt.Key_F:
            self.showNormal() if self.isFullScreen() else self.showFullScreen()


def main():
    import argparse, os, signal, sys
    ap = argparse.ArgumentParser()
    ap.add_argument("-c", "--config", default=DEFAULT_CONFIG)
    args, ros_args = ap.parse_known_args()
    if not os.environ.get("DISPLAY"):
        sys.exit("[vlm-view] no DISPLAY — GUI viewer needs X11.")
    cfg = load_config(args.config)

    rclpy.init(args=ros_args)
    sub = FeedSub(cfg)

    def _spin():
        try:
            rclpy.spin(sub)
        except Exception:   # ExternalShutdownException on normal quit
            pass
    threading.Thread(target=_spin, daemon=True).start()

    app = QApplication(sys.argv)
    win = ViewWindow(sub)
    win.show()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: app.quit())
    try:
        code = app.exec_()
    except KeyboardInterrupt:
        code = 0
    sub.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
