#!/usr/bin/env python3
"""vlm-perception — the whole application in one readable file.

Pipeline:  latest color frame (+ latest depth context)  →  prompt  →  vLLM
           →  parsed JSON  →  /vlm/perception  /vlm/description  /vlm/annotated

Design rules this file demonstrates (see README for the rationale):
  * Capture runs in a plain thread, never in a ROS callback; the newest frame
    always wins (1-slot mailbox) so latency can never accumulate.
  * One request in flight, ever. A slow generation delays the next tick; it
    never queues stale frames behind itself.
  * Depth grounds the VLM as TEXT: a small grid of median distances is appended
    to the prompt; the model correlates what it sees with how far away it is.
  * Every failure is loud: server down / unparseable reply / stale depth all
    surface in the payload — nothing is silently dropped or coerced.
"""

import base64
import json
import re
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from openai import OpenAI
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

from vlm_perception.common import DEFAULT_CONFIG, load_config
from vlm_perception.depth_processor import canonicalize_box, measure_box_distance


# --------------------------------------------------------------------------- #
# Capture (GStreamer, RGB-only mode) — a thread and a 1-slot mailbox
# --------------------------------------------------------------------------- #
class EncodeError(Exception):
    """JPEG encoding of an outgoing frame failed."""


class CaptureThread(threading.Thread):
    def __init__(self, pipeline: str, backoff_initial: float, backoff_max: float, logger):
        super().__init__(daemon=True, name="capture")
        self.pipeline = " ".join(pipeline.split())
        self.backoff_initial, self.backoff_max = backoff_initial, backoff_max
        self.log = logger
        self._lock = threading.Lock()
        self._frame, self._stamp = None, 0.0

    def latest(self):
        with self._lock:
            return self._frame, self._stamp

    def run(self):
        backoff = self.backoff_initial
        while True:
            self.log.info(f"[capture] opening: {self.pipeline}")
            cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)
            if not cap.isOpened():
                self.log.warning(f"[capture] open failed; retrying in {backoff:.0f}s")
                time.sleep(backoff)
                backoff = min(backoff * 2, self.backoff_max)
                continue
            backoff = self.backoff_initial
            while True:
                ok, frame = cap.read()
                if not ok:
                    self.log.warning("[capture] stream ended/failed; reopening")
                    break
                with self._lock:
                    self._frame, self._stamp = frame, time.time()
            cap.release()
            time.sleep(backoff)


# --------------------------------------------------------------------------- #
# Node
# --------------------------------------------------------------------------- #
class VlmPerceptionNode(Node):
    def __init__(self, cfg: dict):
        super().__init__("vlm_perception")
        self.cfg = cfg
        c, p, d, out = cfg["client"], cfg["perception"], cfg["depth"], cfg["output"]
        self.profile = cfg.get("model_profile") or {}
        self._sent_w, self._sent_h = 0, 0   # dims of the image actually sent

        self.client = OpenAI(base_url=c["base_url"], api_key=c.get("api_key", "x"),
                             timeout=float(c.get("read_timeout_s", 60)), max_retries=0)
        self.model = cfg["server"].get("served_model_name", cfg["server"]["model"])
        self.bridge = CvBridge()
        self.busy = False
        self._last_unreachable_warn = 0.0

        # Latest-wins slots (frames + depth context arrive faster than we consume).
        self._lock = threading.Lock()
        self._ros_frame, self._ros_stamp = None, 0.0
        self._depth_ctx, self._depth_stamp = None, 0.0
        self._depth_raw, self._depth_raw_stamp = None, 0.0
        self._depth_jpeg = None

        subs_group = ReentrantCallbackGroup()
        if d.get("enabled"):
            # Color from the Orbbec driver; the GStreamer capture thread is bypassed.
            self.create_subscription(Image, d["color_topic"], self._on_color,
                                     qos_profile_sensor_data, callback_group=subs_group)
            self.create_subscription(String, d.get("context_topic", "/vlm/depth_context"),
                                     self._on_depth_ctx, 10, callback_group=subs_group)
            # Raw aligned depth for per-object distance MEASUREMENT at the
            # VLM's box_2d: the measurement fact-checks the model.
            self.create_subscription(Image, d["depth_topic"], self._on_depth_raw,
                                     qos_profile_sensor_data, callback_group=subs_group)
            if d.get("send_depth_image_to_vlm"):
                self.create_subscription(CompressedImage, d["colormap_topic"],
                                         self._on_depth_img, qos_profile_sensor_data,
                                         callback_group=subs_group)
            self.capture = None
            self.get_logger().info(f"[source] ROS topic {d['color_topic']} (depth mode)")
        else:
            s = cfg["stream"]
            self.capture = CaptureThread(s["gst_pipeline"],
                                         float(s.get("reconnect_backoff_initial_s", 1.0)),
                                         float(s.get("reconnect_backoff_max_s", 30.0)),
                                         self.get_logger())
            self.capture.start()

        self.pub_perception = self.create_publisher(String, out["perception_topic"], 10)
        self.pub_description = self.create_publisher(String, out["description_topic"], 10)
        self.pub_annotated = self.create_publisher(CompressedImage, out["annotated_topic"],
                                                   qos_profile_sensor_data)

        # The VLM call blocks (up to read_timeout); its own mutually-exclusive
        # group means a long call simply delays the next tick — never queues.
        self.create_timer(float(p.get("interval_s", 10)), self._tick,
                          callback_group=MutuallyExclusiveCallbackGroup())
        self.get_logger().info(f"[vlm-perception] model={self.model} "
                               f"interval={p.get('interval_s')}s depth={bool(d.get('enabled'))}")

    # --- subscription callbacks: store-and-return only, no work here ---------- #
    def _on_color(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warning(f"[color] dropped frame: {e}",
                                      throttle_duration_sec=30)
            return
        with self._lock:
            self._ros_frame, self._ros_stamp = frame, time.time()

    def _on_depth_raw(self, msg: Image):
        try:
            raw = self.bridge.imgmsg_to_cv2(msg)
            scale = float(self.cfg["depth"].get("depth_scale", 0.001))
            depth_m = raw.astype(np.float32) * (scale if raw.dtype == np.uint16 else 1.0)
        except Exception as e:
            self.get_logger().warning(f"[depth] dropped frame: {e}",
                                      throttle_duration_sec=30)
            return
        with self._lock:
            self._depth_raw, self._depth_raw_stamp = depth_m, time.time()

    def _on_depth_ctx(self, msg: String):
        with self._lock:
            self._depth_ctx, self._depth_stamp = msg.data, time.time()

    def _on_depth_img(self, msg: CompressedImage):
        with self._lock:
            self._depth_jpeg = bytes(msg.data)

    # --- helpers --------------------------------------------------------------- #
    def _latest_frame(self):
        if self.capture is not None:
            return self.capture.latest()
        with self._lock:
            return self._ros_frame, self._ros_stamp

    def _depth_block(self):
        """Return (prompt_text, status) for the current depth context."""
        d = self.cfg["depth"]
        if not d.get("enabled"):
            return "", "disabled"
        with self._lock:
            ctx, age = self._depth_ctx, time.time() - self._depth_stamp
        if ctx is None:
            return "", "unavailable"
        if age > float(d.get("max_context_age_s", 2.0)):
            return "", f"stale ({age:.1f}s old)"
        return ("\nDEPTH CONTEXT (meters, from an aligned depth camera; grid rows are "
                f"top-to-bottom, columns left-to-right): {ctx}\n"), "ok"

    def _encode(self, frame) -> str:
        c = self.cfg["client"]
        edge = int(c.get("max_image_edge", 1024))
        h, w = frame.shape[:2]
        if max(h, w) > edge:
            s = edge / max(h, w)
            frame = cv2.resize(frame, (int(w * s), int(h * s)))
        self._sent_w, self._sent_h = frame.shape[1], frame.shape[0]
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY,
                                               int(c.get("jpeg_quality", 85))])
        if not ok:
            raise EncodeError(f"cv2.imencode failed for a {frame.shape} frame")
        return "data:image/jpeg;base64," + base64.b64encode(buf).decode()

    @staticmethod
    def _repair_json(text: str):
        """Salvage a TRUNCATED JSON reply (hit max_tokens mid-object): cut back
        to the last complete value, close any open string, then balance
        brackets. Keeps the (early, usually complete) description + whatever
        objects finished — far better than discarding the whole reply."""
        start = text.find("{")
        if start < 0:
            return None
        frag = text[start:]
        # Trim a dangling partial token after the last comma/bracket
        frag = re.sub(r",\s*\"?[^\"\[\]{}]*$", "", frag)
        # One scan decides BOTH string-state and bracket balance. (A naive
        # quote-count parity check breaks on escaped quotes \" inside OCR text.)
        stack = []
        in_str = False
        prev = ""
        for ch in frag:
            if ch == '"' and prev != "\\":
                in_str = not in_str
            elif not in_str and ch in "{[":
                stack.append(ch)
            elif not in_str and ch in "}]":
                if stack:
                    stack.pop()
            prev = "" if prev == "\\" and ch == "\\" else ch
        if in_str:
            frag += '"'
        frag += "".join("}" if b == "{" else "]" for b in reversed(stack))
        try:
            obj = json.loads(frag)
            return obj if isinstance(obj, dict) else None
        except (json.JSONDecodeError, ValueError):
            return None

    @classmethod
    def _parse(cls, text: str) -> dict:
        """Strict → loose → repair JSON extraction; failure is NEVER silent."""
        for candidate in (text, text[text.find("{"): text.rfind("}") + 1]):
            try:
                obj = json.loads(candidate)
                if isinstance(obj, dict):
                    obj["parse_ok"] = True
                    return obj
            except (json.JSONDecodeError, ValueError):
                continue
        repaired = cls._repair_json(text)
        if repaired is not None:
            repaired["parse_ok"] = True
            repaired["truncated"] = True   # partial objects; description intact
            return repaired
        return {"parse_ok": False, "description": re.sub(r"\s+", " ", text).strip()}

    # --- the perception cycle --------------------------------------------------- #
    def _tick(self):
        if self.busy:  # single-flight; latest frame wins next tick
            return
        frame, stamp = self._latest_frame()
        if frame is None:
            self.get_logger().warning("[tick] no frame yet", throttle_duration_sec=30)
            return
        depth_text, depth_status = self._depth_block()
        # Snapshot the depth map NOW, beside the frame we are about to send —
        # grounding after the ~5-8s generation must measure the scene the
        # model SAW, not the scene as it is when the reply lands.
        with self._lock:
            depth_snap = self._depth_raw
            depth_snap_age = time.time() - self._depth_raw_stamp

        self.busy = True
        t0 = time.time()
        try:
            prompt = self.cfg["perception"]["prompt"]
            if not self.profile.get("expect_ocr", True):
                prompt += ("\nDo NOT include a visible_text field — omit it entirely "
                           "from the JSON.")
            content = [{"type": "text", "text": prompt + depth_text},
                       {"type": "image_url", "image_url": {"url": self._encode(frame)}}]
            if depth_status == "ok" and self.cfg["depth"].get("send_depth_image_to_vlm"):
                with self._lock:
                    djpeg = getattr(self, "_depth_jpeg", None)
                if djpeg:
                    content.append({"type": "image_url", "image_url": {
                        "url": "data:image/jpeg;base64," + base64.b64encode(djpeg).decode()}})

            stream = self.client.chat.completions.create(
                model=self.model, temperature=float(self.cfg["client"].get("temperature", 0.0)),
                # Greedy decode can degenerate into repetition loops (endless
                # " " items in visible_text); a frequency penalty breaks them.
                frequency_penalty=float(self.cfg["client"].get("frequency_penalty", 0.3)),
                max_tokens=int(self.cfg["perception"].get("max_tokens", 256)),
                messages=[{"role": "user", "content": content}], stream=True)
            raw, finish, early_sent = "", None, False
            for chunk in stream:
                if not chunk.choices:
                    continue
                choice = chunk.choices[0]
                delta = getattr(choice.delta, "content", None) if choice.delta else None
                if delta:
                    raw += delta
                    if not early_sent:
                        early = self._early_description(raw)
                        if early:
                            early_sent = True
                            self.pub_description.publish(String(data=early))
                if choice.finish_reason:
                    finish = choice.finish_reason
            reasoning_ok = True
            if self.profile.get("strip_reasoning", True):
                raw, reasoning_ok = self._strip_reasoning(raw, finish)
                if not reasoning_ok:
                    raw = ""  # reasoning truncated mid-<think>: nothing trustworthy
            if raw:
                result = self._parse(raw)
            else:
                result = {"parse_ok": False,
                          "status": "reasoning_truncated" if not reasoning_ok
                          else "empty_reply"}
            self._normalize_schema(result)
            if finish == "length":
                result["finish_reason"] = "length"  # hit max_tokens — raise it or tighten the prompt
        except EncodeError as e:
            self.get_logger().warning(f"[encode] {e}", throttle_duration_sec=30)
            result = {"status": "encode_failed", "error": str(e)[:300], "parse_ok": False}
        except Exception as e:  # noqa: BLE001 — every failure becomes a loud payload
            now = time.time()
            if now - self._last_unreachable_warn > 30:
                self.get_logger().warning(f"[vlm] request failed: {e}")
                self._last_unreachable_warn = now
            result = {"status": "server_unreachable", "error": str(e)[:300], "parse_ok": False}
        finally:
            self.busy = False

        # Publish tail is protected too: a malformed reply must produce a loud
        # payload, never an exception escaping the timer callback (the
        # executor re-raises those in main and the whole node dies).
        try:
            self._ground_objects(result, depth_snap, depth_snap_age)
            result.update(frame_age_s=round(time.time() - stamp, 2),
                          latency_ms=int((time.time() - t0) * 1000),
                          depth_context=depth_status, model=self.model, ts=time.time())
            self.pub_perception.publish(String(data=json.dumps(result)))
            desc = result.get("description")
            # human-facing topic gets CLEAN text only — the parse-failure raw
            # blob stays in the JSON payload where parse_ok=false labels it
            if result.get("parse_ok") and isinstance(desc, str) and desc:
                self.pub_description.publish(String(data=desc))
            self._publish_annotated(frame, result)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"[vlm] publish failed: {e}")
            self.pub_perception.publish(String(data=json.dumps(
                {"status": "publish_error", "error": str(e)[:300], "parse_ok": False})))

    _DESC_RE = re.compile(r'"description"\s*:\s*"((?:[^"\\]|\\.)*)"')

    @staticmethod
    def _clean_text(value) -> str:
        """Publishable text or '' — the gate every description passes before a
        String publish. Non-str values from arbitrary checkpoints would trip a
        C-level assert in rclpy's serializer (hard SIGABRT, uncatchable);
        prompt-echoed schema placeholders ('<at most 2 short...') and lone
        surrogates that survive json.loads but not UTF-8 are dropped here too."""
        if isinstance(value, (list, tuple)):
            value = " ".join(str(v) for v in value)
        elif value is None:
            return ""
        elif not isinstance(value, str):
            value = str(value)
        value = value.strip()
        if value.startswith("<"):        # schema placeholder echoed back
            return ""
        return value.encode("utf-8", errors="replace").decode("utf-8")

    @classmethod
    def _early_description(cls, text: str):
        """Streaming early-publish: description is the FIRST schema field the
        model emits, so it is complete seconds before the object/OCR lists
        finish decoding. The moment its closing quote arrives we publish it —
        the human-facing topic (and the viewer banner) updates ~3-4s sooner.
        Never guesses; each guard below is a known failure mode:
          * reasoning: wait out <think> blocks — including chat templates that
            pre-insert the opening tag so only '</think>' ever streams, and
            prose-first reasoning with no tags at all (reply must START with {)
          * top-level only: '"description"' nested inside objects[] (models
            that reorder fields) must not win the race
          * placeholder echo / lone surrogates: rejected by _clean_text"""
        end = text.rfind("</think>")
        if end != -1:
            text = text[end + len("</think>"):]
        elif "<think" in text:
            return None                  # reasoning still open — keep waiting
        if not text.lstrip().startswith("{"):
            return None                  # prose (likely untagged reasoning)
        m = cls._DESC_RE.search(text)
        if not m or not cls._at_top_level(text, m.start()):
            return None
        try:
            desc = json.loads('"' + m.group(1) + '"')
        except ValueError:
            return None
        return cls._clean_text(desc) or None

    @staticmethod
    def _at_top_level(text: str, pos: int) -> bool:
        """True if pos sits at JSON depth 1 (directly inside the root object).
        String-aware scan so braces inside string values don't count."""
        depth, in_str, esc = 0, False, False
        for ch in text[:pos]:
            if in_str:
                if esc:
                    esc = False
                elif ch == "\\":
                    esc = True
                elif ch == '"':
                    in_str = False
            elif ch == '"':
                in_str = True
            elif ch in "{[":
                depth += 1
            elif ch in "}]":
                depth -= 1
        return depth == 1 and not in_str

    @staticmethod
    def _strip_reasoning(text: str, finish_reason):
        """Remove <think>...</think> (reasoning VLMs). An UNCLOSED think block
        on a length-stop means the answer never started — refuse to salvage
        drafts from inside a model's rumination."""
        if "<think>" not in text:
            return text, True
        if "</think>" not in text:
            return text, False   # unclosed reasoning block: the answer never started
        stripped = re.sub(r"<think>.*?</think>", "", text, flags=re.DOTALL).strip()
        if stripped:
            return stripped, True
        return text, False       # only reasoning, no answer after it

    def _normalize_schema(self, result: dict):
        """Model-agnostic output hygiene: coerce types, drop placeholder echoes,
        validate/derive bearing, record what the model actually provided."""
        if not result.get("parse_ok"):
            return
        # description: the ONE field every consumer trusts as a str — coerce
        # (a list/dict here would SIGABRT rclpy's C serializer downstream)
        result["description"] = self._clean_text(result.get("description"))
        # visible_text: strings only; drop prompt-echo placeholders
        vt = []
        for t in result.get("visible_text") or []:
            if isinstance(t, dict):
                t = t.get("text", "")
            t = str(t).strip()
            if t and not t.startswith("<") and "transcribed verbatim" not in t:
                vt.append(t)
        if self.profile.get("expect_ocr", True):
            result["visible_text"] = vt
        else:
            result.pop("visible_text", None)
        # actions: strings only
        result["actions"] = [a if isinstance(a, str) else str(a.get("name", a))
                             for a in (result.get("actions") or [])
                             if not (isinstance(a, str) and a.startswith("<"))]
        # objects: canonicalize boxes ONCE (downstream consumers trust canonical)
        boxes_in, boxes_ok = 0, 0
        bearings = {"left", "center-left", "center", "center-right", "right"}
        for o in result.get("objects") or []:
            if not isinstance(o, dict):
                continue
            raw_box = o.get("box_2d")
            if raw_box is not None:
                boxes_in += 1
                box = canonicalize_box(raw_box, self.profile.get("box_format", "auto"),
                                       self._sent_w, self._sent_h)
                if box is not None:
                    o["box_2d"] = [round(v) for v in box]   # canonical 0-1000
                    boxes_ok += 1
                    cx = (box[0] + box[2]) / 2
                    o["bearing"] = ("left" if cx < 200 else "center-left" if cx < 400
                                    else "center" if cx < 600 else "center-right"
                                    if cx < 800 else "right")  # derived, model-free
                else:
                    o.pop("box_2d", None)   # unusable geometry — honest absence
            if o.get("bearing") not in bearings:
                o["bearing"] = None
        result["schema_conformance"] = {"boxes_returned": boxes_in,
                                        "boxes_usable": boxes_ok}

    def _ground_objects(self, result: dict, depth_m=None, depth_age: float = None):
        """Measure each object's real distance from the aligned depth at its
        box_2d and fact-check the VLM's estimate: the depth map is
        authoritative; the model only proposes. depth_m/depth_age
        are the snapshot taken WITH the frame at tick start — measuring against
        the current map after the ~5-8s generation would sample a scene the
        model never saw (moving objects get someone else's distance)."""
        d = self.cfg["depth"]
        if not d.get("enabled") or not result.get("parse_ok"):
            return
        if depth_m is None:
            with self._lock:
                depth_m = self._depth_raw
                depth_age = time.time() - self._depth_raw_stamp
        if depth_m is None or depth_age > float(d.get("max_context_age_s", 2.0)):
            return
        tol = float(d.get("distance_tolerance_pct", 15)) / 100.0
        pct = float(d.get("bbox_percentile", 25))
        lo, hi = float(d.get("min_m", 0.02)), float(d.get("max_m", 10.0))
        for obj in result.get("objects") or []:
            if not isinstance(obj, dict) or obj.get("box_2d") is None:
                continue
            # box_2d is already canonical 0-1000 (normalized in _normalize_schema)
            meas = measure_box_distance(depth_m, obj["box_2d"], pct, lo, hi,
                                        box_format="qwen_1000")
            if meas is None:
                continue
            obj["measured_distance_m"] = meas
            est = obj.get("approx_distance_m")
            if isinstance(est, (int, float)) and meas > 0:
                # 'disputed', not 'measured wins': with weaker-grounding models a
                # misplaced box can make the measurement the wrong one — publish
                # both and let the consumer decide by schema_conformance quality.
                obj["distance_check"] = ("ok" if abs(est - meas) / meas <= tol
                                         else f"disputed (vlm {est} vs measured {meas})")

    def _publish_annotated(self, frame, result: dict):
        img = frame.copy()
        h, w = img.shape[:2]
        for obj in result.get("objects") or []:
            box = obj.get("box_2d") if isinstance(obj, dict) else None
            if not box or len(box) != 4:
                continue
            try:
                x1, y1, x2, y2 = (int(box[0] / 1000 * w), int(box[1] / 1000 * h),
                                  int(box[2] / 1000 * w), int(box[3] / 1000 * h))
            except (TypeError, ValueError):
                continue
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 220, 255), 2)
            dist = obj.get("measured_distance_m", obj.get("approx_distance_m"))
            label = f"{obj.get('name', '?')}" + (f" {dist}m" if dist is not None else "")
            cv2.putText(img, label, (x1 + 3, max(14, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 255), 2, cv2.LINE_AA)
        cap_lines = [("VLM: " + (result.get("description") or result.get("status", ""))[:110])]
        cap_lines += ["TEXT: " + t[:100] for t in (result.get("visible_text") or [])[:3]]
        y = h - 14 * len(cap_lines) - 8
        cv2.rectangle(img, (0, y - 6), (img.shape[1], h), (0, 0, 0), -1)
        for line in cap_lines:
            cv2.putText(img, line, (8, y + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (255, 255, 255), 1, cv2.LINE_AA)
            y += 14
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY,
                               int(self.cfg["output"].get("annotated_jpeg_quality", 80))])
        if ok:
            msg = CompressedImage(format="jpeg", data=buf.tobytes())
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub_annotated.publish(msg)


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("-c", "--config", default=DEFAULT_CONFIG)
    args, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args)
    node = VlmPerceptionNode(load_config(args.config))
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # rclpy's SIGINT handler may have shut the context down already
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
