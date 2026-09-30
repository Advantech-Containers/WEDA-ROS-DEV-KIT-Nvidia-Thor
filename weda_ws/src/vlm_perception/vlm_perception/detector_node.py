#!/usr/bin/env python3
"""vlm-detector — Tier-1 REAL-TIME perception (BYOL: ultralytics).

The VLM (Tier-2) understands scenes but needs seconds per analysis; this node
OWNS the real-time geometry: YOLO detection + ByteTrack IDs at camera rate,
plus a live depth measurement per box. Publishes /vlm/detections (String JSON):

  {"ts": ..., "fps": 27.4, "objects": [
      {"id": 3, "cls": "person", "conf": 0.87,
       "box_2d": [412, 180, 655, 930],          # canonical 0-1000
       "measured_distance_m": 1.24}, ...]}

Gracefully idles (single WARN) when ultralytics isn't installed — it is a
commercial package, hydrated at container start by the BYOL license gate.
No queues anywhere: latest-frame-wins, exactly like the rest of the pipeline.
"""

import json
import os
import shutil
import threading
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from vlm_perception.common import DEFAULT_CONFIG, load_config
from vlm_perception.depth_processor import measure_box_distance

try:
    from ultralytics import YOLO
    _HAS_YOLO = True
except ImportError:
    _HAS_YOLO = False


class DetectorNode(Node):
    def __init__(self, cfg: dict):
        super().__init__("vlm_detector")
        det = cfg.get("detector") or {}
        d = cfg["depth"]
        self.bridge = CvBridge()
        self._lock = threading.Lock()
        self._frame = None
        self._depth = None

        self.pub = self.create_publisher(String, det.get("publish_topic", "/vlm/detections"), 10)

        if not det.get("enabled", True):
            self.get_logger().info("[vlm-detector] disabled in config.")
            return
        if not _HAS_YOLO:
            self.get_logger().warn(
                "[vlm-detector] ultralytics not installed (BYOL) — detector tier idle. "
                "Set ACCEPT_COMMERCIAL_EULA=true in .env and restart the container.")
            return
        if not d.get("enabled"):
            self.get_logger().warn("[vlm-detector] needs the camera driver (depth mode) — idle.")
            return

        self.det_cfg = det
        self.depth_cfg = d
        self.create_subscription(Image, d["color_topic"], self._on_color,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, d["depth_topic"], self._on_depth,
                                 qos_profile_sensor_data)
        self._stopped = False
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        self.get_logger().info(f"[vlm-detector] {det.get('model', 'yolo11n.pt')} "
                               f"@ max {det.get('max_fps', 30)}fps, tracking on")

    def _on_color(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warning(f"[color] dropped frame: {e}",
                                      throttle_duration_sec=30)
            return
        with self._lock:
            self._frame = frame

    def _on_depth(self, msg):
        try:
            raw = self.bridge.imgmsg_to_cv2(msg)
            scale = float(self.depth_cfg.get("depth_scale", 0.001))
            depth_m = raw.astype(np.float32) * (scale if raw.dtype == np.uint16 else 1.0)
        except Exception as e:
            self.get_logger().warning(f"[depth] dropped frame: {e}",
                                      throttle_duration_sec=30)
            return
        with self._lock:
            self._depth = depth_m

    def _ensure_model(self, det) -> str:
        """TensorRT engines are DEVICE-specific — never shipped, always built
        on the device. If the configured .engine is missing, export it from
        the matching .pt once (a few minutes) and persist it in the mounted
        workspace so every later start is instant."""
        path = det.get("model", "yolo26n.engine")
        if not path.endswith(".engine") or os.path.exists(path):
            return path
        # Source .pt: the sibling next to the configured engine if it exists
        # (custom models); else the bare official name, which ultralytics
        # auto-downloads. Never a bare CUSTOM name — that would resolve
        # against cwd and then a doomed GitHub asset lookup.
        sibling = path[:-len(".engine")] + ".pt"
        pt = sibling if os.path.exists(sibling) else os.path.basename(sibling)
        self.get_logger().info(
            f"[vlm-detector] {path} not found — exporting TensorRT FP16 engine "
            f"from {pt} (first run only, ~3 min)...")
        try:
            # makedirs inside the try: an unwritable model dir must land in
            # the same fallback as any other export failure, not raise out
            os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
            exported = YOLO(pt).export(format="engine", imgsz=int(det.get("imgsz", 640)),
                                       half=True, device=0, verbose=False)
            if os.path.abspath(exported) != os.path.abspath(path):
                shutil.move(exported, path)
            self.get_logger().info(f"[vlm-detector] engine ready: {path}")
            return path
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"[vlm-detector] engine export failed ({e}) — "
                                   f"falling back to PyTorch {pt}")
            return pt

    def _load_model(self, det):
        """Model construction runs in the worker thread — a failure here must
        be LOUD and leave the node alive (matching every other degraded mode),
        never a silent daemon-thread death. A corrupt/incompatible .engine
        (e.g. built on different silicon) falls back to its .pt once."""
        path = self._ensure_model(det)
        candidates = [path]
        if path.endswith(".engine"):
            sibling = path[:-len(".engine")] + ".pt"
            candidates.append(sibling if os.path.exists(sibling)
                              else os.path.basename(sibling))
        for cand in dict.fromkeys(candidates):
            try:
                return YOLO(cand, task="detect")  # engines lack task metadata
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f"[vlm-detector] cannot load {cand}: {e}")
        self.get_logger().error(
            "[vlm-detector] no usable model — detector tier idle. If the "
            ".engine is corrupt (interrupted export?), delete it to re-export.")
        return None

    def _loop(self):
        det = self.det_cfg
        model = self._load_model(det)
        if model is None:
            return
        classes = det.get("classes") or None      # None = all COCO classes
        name_of = model.names
        min_period = 1.0 / float(det.get("max_fps", 30))
        lo, hi = float(self.depth_cfg.get("min_m", 0.02)), float(self.depth_cfg.get("max_m", 10.0))
        pct = float(self.depth_cfg.get("bbox_percentile", 25))
        n, t0 = 0, time.time()
        fps = 0.0
        while rclpy.ok() and not self._stopped:
            tick = time.time()
            with self._lock:
                frame, depth = self._frame, self._depth
            if frame is None:
                time.sleep(0.2)
                continue
            try:
                res = model.track(frame, imgsz=int(det.get("imgsz", 640)),
                                  conf=float(det.get("conf", 0.4)),
                                  classes=classes, device=0, persist=True,
                                  tracker=det.get("tracker", "bytetrack.yaml"),
                                  verbose=False)[0]
            except Exception as e:  # noqa: BLE001 — keep the tier alive
                self.get_logger().warn(f"[vlm-detector] inference error: {e}",
                                       throttle_duration_sec=30)
                time.sleep(1.0)
                continue
            objs = []
            boxes = res.boxes
            if boxes is not None and len(boxes):
                xyxyn = boxes.xyxyn.cpu().numpy()
                confs = boxes.conf.cpu().numpy()
                clss = boxes.cls.cpu().numpy().astype(int)
                ids = (boxes.id.cpu().numpy().astype(int)
                       if boxes.id is not None else [-1] * len(boxes))
                for i in range(len(boxes)):
                    b1000 = [round(float(v) * 1000) for v in xyxyn[i]]  # canonical
                    obj = {"id": int(ids[i]), "cls": name_of[clss[i]],
                           "conf": round(float(confs[i]), 2), "box_2d": b1000}
                    if depth is not None:
                        m = measure_box_distance(depth, b1000, pct, lo, hi,
                                                 box_format="qwen_1000")
                        if m is not None:
                            obj["measured_distance_m"] = m
                    objs.append(obj)
            n += 1
            if time.time() - t0 >= 5:
                fps = n / (time.time() - t0)
                n, t0 = 0, time.time()
            self.pub.publish(String(data=json.dumps(
                {"ts": time.time(), "fps": round(fps, 1), "objects": objs})))
            dt = time.time() - tick
            if dt < min_period:
                time.sleep(min_period - dt)


    def destroy_node(self):
        self._stopped = True
        thread = getattr(self, "_thread", None)
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)
        super().destroy_node()


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("-c", "--config", default=DEFAULT_CONFIG)
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = DetectorNode(load_config(args.config))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
