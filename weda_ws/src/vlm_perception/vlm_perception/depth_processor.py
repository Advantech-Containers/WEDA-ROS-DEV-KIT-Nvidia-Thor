#!/usr/bin/env python3
"""vlm-depth — Standalone custom ROS camera driver natively using pyorbbecsdk2.

Connects to the Orbbec USB camera natively, fetches RGB+Depth,
aligns them via AlignFilter, and publishes:
  * /camera/color/image_raw (bgr8)
  * /camera/depth_registered/image_raw (16UC1)
  * /vlm/depth_context (String JSON)
  * /camera/color/camera_info (CameraInfo, real RGB intrinsics)
  * colormap on depth.colormap_topic (CompressedImage; shipped config uses
    /vlm/depth_colormap/compressed)
"""

import json
import time
import threading

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String

from vlm_perception.common import DEFAULT_CONFIG, load_config

_REOPEN_AFTER = 5           # consecutive failures before re-opening the camera
_REOPEN_MAX_DELAY_S = 30.0  # cap on the backoff between re-open attempts

try:
    from pyorbbecsdk import (
        Pipeline, Config, OBSensorType, OBFormat,
        OBAlignMode, VideoFrame, AlignFilter, OBStreamType
    )
    _HAS_ORBBEC = True
except ImportError as e:
    import logging
    logging.getLogger("vlm-depth").error(f"pyorbbecsdk2 import failed: {e}")
    _HAS_ORBBEC = False


# --------------------------------------------------------------------------- #
# Pure functions (no ROS) — the actual depth math
# --------------------------------------------------------------------------- #
def compute_depth_context(depth_m: np.ndarray, grid=(3, 3), near_m: float = 1.0,
                          min_m: float = 0.02, max_m: float = 10.0) -> dict:
    valid = np.isfinite(depth_m) & (depth_m > min_m) & (depth_m < max_m)
    if not valid.any():
        return {"valid": False}

    vals = depth_m[valid]
    h, w = depth_m.shape[:2]
    rows, cols = grid
    cells = []
    for r in range(rows):
        row = []
        for c in range(cols):
            cell = depth_m[r * h // rows:(r + 1) * h // rows,
                           c * w // cols:(c + 1) * w // cols]
            cv = cell[np.isfinite(cell) & (cell > min_m) & (cell < max_m)]
            row.append(round(float(np.median(cv)), 2) if cv.size else None)
        cells.append(row)

    ch, cw = h // 3, w // 3
    center = depth_m[ch:2 * ch, cw:2 * cw]
    center = center[np.isfinite(center) & (center > min_m) & (center < max_m)]

    return {
        "valid": True,
        "range_m": [round(float(vals.min()), 2), round(float(vals.max()), 2)],
        "mean_m": round(float(vals.mean()), 2),
        "center_m": round(float(np.median(center)), 2) if center.size else None,
        "grid_median_m": cells,
        "pct_closer_than_%.1fm" % near_m: round(float((vals < near_m).mean() * 100), 1),
    }


def colorize_depth(depth_m: np.ndarray, min_m: float = 0.02, max_m: float = 10.0) -> np.ndarray:
    valid = depth_m[np.isfinite(depth_m) & (depth_m > min_m) & (depth_m < max_m)]
    if valid.size > 100:
        lo, hi = np.percentile(valid, [2, 98])
        hi = hi if hi > lo else lo + 1e-3
    else:
        lo, hi = 0.0, max_m
    norm = np.nan_to_num((np.clip(depth_m, lo, hi) - lo) / (hi - lo), nan=0.0)
    return cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_JET)


def canonicalize_box(box_2d, box_format: str = "auto",
                     sent_w: int = 0, sent_h: int = 0):
    """Normalize a model's bounding box to CANONICAL 0-1000 space, or None.

    There is NO universal VLM box convention — it is baked in by training:
    Qwen emits 0-1000 ints, LLaVA-family 0-1 floats, others raw pixels.
    Prompting cannot force compliance (verified empirically), so we DETECT:
      * any coord > 1000            -> pixels of the SENT image (needs dims)
      * all coords <= 1.001         -> 0-1 floats
      * otherwise                   -> assume the requested 0-1000
    Ambiguity between 0-1000 and pixels-on-small-images exists; pin
    model_profile.box_format in vlm_app.yaml for models that emit pixels.
    Returns [x1, y1, x2, y2] in 0-1000 (sorted, clamped) or None.
    """
    try:
        vals = [float(v) for v in box_2d]
    except (TypeError, ValueError):
        return None
    if len(vals) != 4 or not all(np.isfinite(vals)):
        return None
    fmt = box_format
    if fmt == "auto":
        if max(vals) > 1000.0:
            fmt = "pixels"
        elif max(vals) <= 1.001:
            fmt = "norm_01"
        else:
            fmt = "qwen_1000"
    if fmt == "norm_01":
        vals = [v * 1000.0 for v in vals]
    elif fmt == "pixels":
        if sent_w <= 0 or sent_h <= 0:
            return None  # cannot rescale without the sent-image dims
        vals = [vals[0] / sent_w * 1000, vals[1] / sent_h * 1000,
                vals[2] / sent_w * 1000, vals[3] / sent_h * 1000]
    elif fmt == "none":
        return None
    x1, x2 = sorted((vals[0], vals[2]))
    y1, y2 = sorted((vals[1], vals[3]))
    box = [max(0.0, min(1000.0, v)) for v in (x1, y1, x2, y2)]
    return box if (box[2] - box[0] >= 5 and box[3] - box[1] >= 5) else None


def measure_box_distance(depth_m: np.ndarray, box_2d, percentile: float = 25,
                         min_m: float = 0.02, max_m: float = 10.0,
                         box_format: str = "auto", sent_w: int = 0, sent_h: int = 0,
                         min_valid_px: int = 30, min_valid_frac: float = 0.05):
    """Object distance from the aligned depth inside a canonicalized box.

    Quality-gated: requires >= min_valid_px valid pixels AND >= min_valid_frac
    of the ROI — a measurement from a sliver of pixels is worse than none
    (it would let misplaced boxes publish confident wrong distances)."""
    box = canonicalize_box(box_2d, box_format, sent_w, sent_h)
    if box is None:
        return None
    h, w = depth_m.shape[:2]
    c1, r1 = int(box[0] / 1000 * w), int(box[1] / 1000 * h)
    c2, r2 = int(box[2] / 1000 * w), int(box[3] / 1000 * h)
    if c2 <= c1 or r2 <= r1:
        return None
    roi = depth_m[r1:r2, c1:c2]
    valid = roi[np.isfinite(roi) & (roi > min_m) & (roi < max_m)]
    if valid.size < max(min_valid_px, int(roi.size * min_valid_frac)):
        return None
    return round(float(np.percentile(valid, percentile)), 2)


# --------------------------------------------------------------------------- #
# Node
# --------------------------------------------------------------------------- #
class DepthProcessorNode(Node):
    def __init__(self, cfg: dict):
        super().__init__("vlm_depth_processor")
        d = cfg["depth"]
        self.scale = float(d.get("depth_scale", 0.001))
        self.grid = tuple(d.get("grid", [3, 3]))
        self.near_m = float(d.get("near_threshold_m", 1.0))
        self.min_m = float(d.get("min_m", 0.02))
        self.max_m = float(d.get("max_m", 10.0))
        self.min_period = 1.0 / float(d.get("publish_rate_hz", 2.0))
        self._last_pub = 0.0

        self.bridge = CvBridge()
        # RELIABLE (default) QoS: matches both best-effort subscribers (rviz,
        # vlm_perception) AND depth_image_proc's reliable subscriptions.
        self.pub_color = self.create_publisher(Image, d["color_topic"], 10)
        self.pub_depth = self.create_publisher(Image, d["depth_topic"], 10)
        self.pub_info = self.create_publisher(CameraInfo,
                                              d.get("camera_info_topic", "/camera/color/camera_info"), 10)
        self.pub_ctx = self.create_publisher(String, d.get("context_topic", "/vlm/depth_context"), 10)
        self.pub_map = self.create_publisher(CompressedImage, d.get("colormap_topic", "/vlm/depth_colormap"), qos_profile_sensor_data)

        if not _HAS_ORBBEC:
            self.get_logger().error("pyorbbecsdk2 is not installed. Camera driver will not start.")
            return

        self._pipeline = None
        self._align_filter = None
        self._cam_info = None
        self.pub_diag = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        if not self._open_pipeline():
            self.get_logger().error(
                "[vlm-depth] camera unavailable at start — the capture loop will keep "
                "trying; depth consumers degrade to unavailable meanwhile")

        self._running = True
        self._fail_run = 0
        self._reopen_delay = 1.0
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def _open_pipeline(self) -> bool:
        """Open the camera and start streaming. Safe to call again after a failure."""
        self._pipeline = Pipeline()
        cfg_orbbec = Config()

        # Open color
        try:
            pl_color = self._pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
            profile_color = pl_color.get_default_video_stream_profile()
            cfg_orbbec.enable_stream(profile_color)
            self.get_logger().info(f"[vlm-depth] Color profile: {profile_color.get_width()}x{profile_color.get_height()} @ {profile_color.get_fps()}fps")
        except Exception as e:
            self.get_logger().error(f"Failed to open color stream: {e}")
            return False

        # Open depth
        try:
            pl_depth = self._pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
            profile_depth = pl_depth.get_default_video_stream_profile()
            cfg_orbbec.enable_stream(profile_depth)
            self.get_logger().info(f"[vlm-depth] Depth profile: {profile_depth.get_width()}x{profile_depth.get_height()} @ {profile_depth.get_fps()}fps")
        except Exception as e:
            self.get_logger().error(f"Failed to open depth stream: {e}")
            return False

        try:
            self._pipeline.start(cfg_orbbec)
        except Exception as e:
            self.get_logger().error(f"Failed to start the camera pipeline: {e}")
            return False

        # Real RGB intrinsics -> CameraInfo (needed by depth_image_proc to
        # project the aligned depth into the 3D pointcloud).
        self._cam_info = None
        try:
            intr = self._pipeline.get_camera_param().rgb_intrinsic
            ci = CameraInfo()
            ci.width, ci.height = int(intr.width), int(intr.height)
            ci.distortion_model = "plumb_bob"
            ci.d = [0.0] * 5
            ci.k = [float(intr.fx), 0.0, float(intr.cx),
                    0.0, float(intr.fy), float(intr.cy),
                    0.0, 0.0, 1.0]
            ci.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            ci.p = [float(intr.fx), 0.0, float(intr.cx), 0.0,
                    0.0, float(intr.fy), float(intr.cy), 0.0,
                    0.0, 0.0, 1.0, 0.0]
            self._cam_info = ci
            self.get_logger().info(
                f"[vlm-depth] intrinsics: fx={intr.fx:.1f} fy={intr.fy:.1f} "
                f"cx={intr.cx:.1f} cy={intr.cy:.1f}")
        except Exception as e:
            self.get_logger().warn(f"camera_param unavailable ({e}) — 3D pointcloud disabled")

        try:
            self._align_filter = AlignFilter(align_to_stream=OBStreamType.COLOR_STREAM)
            self.get_logger().info("AlignFilter (SW depth->color) active")
        except Exception as e:
            self.get_logger().warn(f"AlignFilter failed: {e}")
            self._align_filter = None

        return True

    def _color_to_numpy(self, frame) -> np.ndarray:
        data = np.frombuffer(frame.get_data(), dtype=np.uint8)
        w, h = frame.get_width(), frame.get_height()
        fmt = frame.get_format()

        if fmt == OBFormat.RGB:
            return cv2.cvtColor(data.reshape(h, w, 3), cv2.COLOR_RGB2BGR)
        elif fmt == OBFormat.BGR:
            return data.reshape(h, w, 3)
        elif fmt == OBFormat.MJPG:
            return cv2.imdecode(data, cv2.IMREAD_COLOR)
        elif fmt == OBFormat.YUYV:
            return cv2.cvtColor(data.reshape(h, w, 2), cv2.COLOR_YUV2BGR_YUYV)
        else:
            if data.size == h * w * 3:
                return data.reshape(h, w, 3)
            return None

    def _depth_to_numpy(self, frame) -> np.ndarray:
        w, h = frame.get_width(), frame.get_height()
        data = frame.get_data()
        fmt = frame.get_format()

        if fmt == OBFormat.Y16:
            return np.frombuffer(data, dtype=np.uint16).reshape(h, w)
        byte_data = np.frombuffer(data, dtype=np.uint8)
        if byte_data.size == h * w * 2:
            return byte_data.view(np.uint16).reshape(h, w)
        if byte_data.size == h * w:
            return byte_data.astype(np.uint16) * 32
        return None

    def _close_pipeline(self):
        try:
            if self._pipeline is not None:
                self._pipeline.stop()
        except Exception as e:
            self.get_logger().warn(f"stopping the camera pipeline failed: {e}")
        self._pipeline = None

    def _publish_diagnostic(self, level, message, detail):
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        st = DiagnosticStatus()
        st.level = level
        st.name = "vlm-depth: camera"
        st.message = message
        st.hardware_id = "orbbec"
        if detail:
            st.values = [KeyValue(key="error", value=detail[:200])]
        msg.status = [st]
        self.pub_diag.publish(msg)

    def _capture_loop(self):
        while self._running:
            try:
                frames = self._pipeline.wait_for_frames(100)
                if frames is None:
                    continue

                if self._align_filter is not None:
                    aligned = self._align_filter.process(frames)
                    if aligned is not None:
                        frames = aligned.as_frame_set() if hasattr(aligned, "as_frame_set") else aligned

                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()

                if color_frame is not None and depth_frame is not None:
                    rgb = self._color_to_numpy(color_frame)
                    depth = self._depth_to_numpy(depth_frame)

                    if rgb is None or depth is None:
                        continue

                    # Publish raw images to ROS topics
                    stamp = self.get_clock().now().to_msg()

                    msg_color = self.bridge.cv2_to_imgmsg(rgb, encoding="bgr8")
                    msg_color.header.stamp = stamp
                    msg_color.header.frame_id = "camera_color_optical_frame"
                    self.pub_color.publish(msg_color)

                    msg_depth = self.bridge.cv2_to_imgmsg(depth, encoding="16UC1")
                    msg_depth.header.stamp = stamp
                    # Aligned depth lives in the COLOR camera geometry
                    msg_depth.header.frame_id = "camera_color_optical_frame"
                    self.pub_depth.publish(msg_depth)

                    if self._cam_info is not None:
                        self._cam_info.header.stamp = stamp
                        self._cam_info.header.frame_id = "camera_color_optical_frame"
                        self.pub_info.publish(self._cam_info)

                    # Throttle context publishing (which is only used by VLM)
                    now = time.time()
                    if now - self._last_pub >= self.min_period:
                        self._last_pub = now
                        depth_m = depth.astype(np.float32) * self.scale
                        ctx = compute_depth_context(depth_m, self.grid, self.near_m, self.min_m, self.max_m)
                        self.pub_ctx.publish(String(data=json.dumps(ctx)))

                        ok, buf = cv2.imencode(".jpg", colorize_depth(depth_m, self.min_m, self.max_m))
                        if ok:
                            out = CompressedImage(format="jpeg", data=buf.tobytes())
                            out.header.stamp = stamp
                            out.header.frame_id = "camera_depth_optical_frame"
                            self.pub_map.publish(out)

                self._fail_run = 0
                self._reopen_delay = 1.0
            except Exception as e:
                self._fail_run += 1
                self.get_logger().error(f"Capture loop error: {e}", throttle_duration_sec=30)
                self._publish_diagnostic(
                    DiagnosticStatus.ERROR,
                    f"capture failing ({self._fail_run} consecutive)", str(e))
                # Reading again cannot recover an invalidated pipeline -- the
                # camera has to be re-opened. Back off so a device that is gone
                # for good does not spin, and cap the delay so a replug is
                # picked up without an operator.
                if self._fail_run >= _REOPEN_AFTER:
                    self.get_logger().warn(
                        f"reopening the camera after {self._fail_run} failures",
                        throttle_duration_sec=30)
                    self._close_pipeline()
                    if self._open_pipeline():
                        self.get_logger().info("camera reopened")
                        self._publish_diagnostic(DiagnosticStatus.OK, "camera reopened", "")
                        self._fail_run = 0
                        self._reopen_delay = 1.0
                        continue
                    time.sleep(self._reopen_delay)
                    self._reopen_delay = min(self._reopen_delay * 2, _REOPEN_MAX_DELAY_S)
                else:
                    time.sleep(1.0)

    def destroy_node(self):
        self._running = False
        thread = getattr(self, "_thread", None)
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)
        self._close_pipeline()
        super().destroy_node()


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("-c", "--config", default=DEFAULT_CONFIG)
    args, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args)
    node = DepthProcessorNode(load_config(args.config))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # rclpy's SIGINT handler may have shut the context down already
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
