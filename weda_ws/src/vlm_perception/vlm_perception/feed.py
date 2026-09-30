#!/usr/bin/env python3
"""vlm-feed — human-readable live view of /vlm/perception.

`ros2 topic echo` truncates long string fields for display; this small
subscriber prints the FULL payload, formatted for humans:

    [14:02:11  9.8s  depth:ok]
      A worker at an assembly bench holds a labeled cardboard box near a scanner...
      objects : box 0.34m (center) | scanner 1.32m (center-right)
      text    : WEDA-42 | FRAGILE
"""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from vlm_perception.common import DEFAULT_CONFIG, load_config

CYAN, DIM, YELLOW, RESET = "\033[1;36m", "\033[2m", "\033[1;33m", "\033[0m"


def _fmt_obj(o):
    if not isinstance(o, dict):
        return str(o)
    dist = o.get("measured_distance_m", o.get("approx_distance_m"))
    parts = [str(o.get("name", "?"))]
    if dist is not None:
        parts.append(f"{dist}m")
    if o.get("bearing"):
        parts.append(f"({o['bearing']})")
    if str(o.get("distance_check", "")).startswith("disputed"):
        parts.append("[disputed]")
    return " ".join(parts)


class FeedNode(Node):
    def __init__(self, topic: str):
        super().__init__("vlm_feed")
        self.create_subscription(String, topic, self._on_msg, 10)
        print(f"{DIM}listening on {topic} — full payloads, no truncation{RESET}")

    def _on_msg(self, msg: String):
        try:
            p = json.loads(msg.data)
        except json.JSONDecodeError:
            print(msg.data)
            return
        stamp = time.strftime("%H:%M:%S")
        lat = p.get("latency_ms", 0) / 1000
        head = f"{CYAN}[{stamp}  {lat:.1f}s  depth:{p.get('depth_context', '?')}]{RESET}"
        if p.get("truncated"):
            head += f" {YELLOW}(long reply — repaired){RESET}"
        if not p.get("parse_ok", True):
            head += f" {YELLOW}({p.get('status', 'unparsed reply')}){RESET}"
        print(head)
        if p.get("description"):
            print(f"  {p['description']}")
        objs = [o for o in (p.get("objects") or []) if isinstance(o, dict)]
        if objs:
            print(f"  {DIM}objects :{RESET} " + " | ".join(_fmt_obj(o) for o in objs))
        if p.get("visible_text"):
            print(f"  {DIM}text    :{RESET} " + " | ".join(str(t) for t in p["visible_text"]))
        if p.get("actions"):
            print(f"  {DIM}actions :{RESET} " + " | ".join(str(a) for a in p["actions"]))
        print(flush=True)


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("-c", "--config", default=DEFAULT_CONFIG)
    args, ros_args = ap.parse_known_args()
    topic = load_config(args.config)["output"]["perception_topic"]

    rclpy.init(args=ros_args)
    node = FeedNode(topic)
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
