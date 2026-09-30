#!/usr/bin/env python3
"""vlm-serve — launch the vLLM OpenAI server from config/vlm_app.yaml.

ROS-free on purpose: the vlm-server compose service runs this file by path
BEFORE anyone has built the colcon workspace. One YAML drives both the server
and the ROS node — single source of truth.

The assembled `vllm serve` command line is printed before exec so users can
see exactly which Thor optimizations are applied (teach-by-example).
"""

import argparse
import json
import os
import shutil
import string
import sys

import yaml

DEFAULT_CONFIG = "/opt/weda-release/config/vlm_app.yaml"


def load_server_config(path: str) -> dict:
    try:
        with open(path) as f:
            raw = f.read()
        # Same ${VAR} contract as common.load_config — the config header
        # promises env interpolation for the WHOLE file, and this launcher
        # must read it exactly like the ROS nodes do. (Inlined, not imported:
        # this file runs by path before the workspace is built.)
        cfg = yaml.safe_load(string.Template(raw).safe_substitute(os.environ))
    except FileNotFoundError:
        sys.exit(f"[vlm-serve] FATAL: config not found: {path}")
    except yaml.YAMLError as e:
        sys.exit(f"[vlm-serve] FATAL: invalid YAML in {path}: {e}")
    server = (cfg or {}).get("server")
    if not isinstance(server, dict) or "model" not in server:
        sys.exit(f"[vlm-serve] FATAL: {path} needs a 'server:' section with 'model:'")
    return server


def hf_cache_status(model: str) -> str:
    cache = os.environ.get("HUGGINGFACE_HUB_CACHE") or os.path.expanduser(
        os.path.join(os.environ.get("HF_HOME", "~/.cache/huggingface"), "hub")
    )
    model_dir = os.path.join(cache, "models--" + model.replace("/", "--"))
    if os.path.isdir(model_dir):
        return f"weights CACHED at {model_dir}"
    return f"weights NOT cached (will download to {cache} on first start — set HF_TOKEN in .env if gated)"


def build_argv(server: dict) -> list:
    argv = ["vllm", "serve", server["model"],
            "--host", str(server.get("host", "0.0.0.0")),
            "--port", str(server.get("port", 8000))]

    def add(flag, key, transform=str):
        if server.get(key) is not None:
            argv.extend([flag, transform(server[key])])

    add("--served-model-name", "served_model_name")
    add("--revision", "revision")
    add("--gpu-memory-utilization", "gpu_memory_utilization")
    add("--max-model-len", "max_model_len")
    add("--max-num-seqs", "max_num_seqs")
    add("--max-num-batched-tokens", "max_num_batched_tokens")
    add("--kv-cache-dtype", "kv_cache_dtype")
    add("--quantization", "quantization")
    add("--attention-backend", "attention_backend")

    if server.get("enable_chunked_prefill"):
        argv.append("--enable-chunked-prefill")
    if server.get("enable_prefix_caching"):
        argv.append("--enable-prefix-caching")
    if server.get("trust_remote_code"):
        argv.append("--trust-remote-code")
    if server.get("enforce_eager"):
        # Debug escape hatch only: costs ~7x decode throughput on Jetson.
        argv.append("--enforce-eager")

    if server.get("speculative_config"):
        # ngram speculative decoding: drafts token spans by looking them up in
        # the prompt — free speedup for structured JSON output whose keys
        # repeat from the schema. No draft model, no extra memory.
        argv.extend(["--speculative-config", json.dumps(server["speculative_config"])])

    mm = server.get("limit_mm_per_prompt") or {}
    if mm:
        argv.extend(["--limit-mm-per-prompt", json.dumps(mm)])
    if server.get("mm_processor_cache_gb") is not None:
        argv.extend(["--mm-processor-cache-gb", str(server["mm_processor_cache_gb"])])
    if server.get("max_pixels"):
        # max_pixels is a Qwen/GLM-processor kwarg; other families silently
        # ignore it (frame token cap NOT in effect) — warn instead of lying.
        fam = server["model"].lower()
        if any(k in fam for k in ("qwen", "glm", "ernie")):
            argv.extend(["--mm-processor-kwargs",
                         json.dumps({"max_pixels": int(server["max_pixels"])})])
        else:
            print(f"[vlm-serve] WARNING: max_pixels set but '{server['model']}' is not a "
                  "Qwen/GLM-family model — its processor will ignore the cap; per-frame "
                  "token cost follows the model's own tiling logic.", flush=True)

    argv.extend(str(a) for a in server.get("extra_args") or [])
    return argv


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-c", "--config", default=os.environ.get("VLM_APP_CONFIG", DEFAULT_CONFIG))
    ap.add_argument("--dry-run", action="store_true", help="print the command and exit")
    args = ap.parse_args()

    server = load_server_config(args.config)
    argv = build_argv(server)

    print("=" * 78)
    print("WEDA VLM server — assembled from", args.config)
    print(f"  model : {server['model']}  ({hf_cache_status(server['model'])})")
    print(f"  cuda graphs: {'OFF (enforce_eager!)' if server.get('enforce_eager') else 'ON'}"
          f" | kv cache: {server.get('kv_cache_dtype', 'auto')}"
          f" | attention: {server.get('attention_backend', 'default')}")
    print("  exec:", " ".join(argv))
    print("=" * 78, flush=True)

    if args.dry_run:
        return
    if shutil.which("vllm") is None:
        sys.exit("[vlm-serve] FATAL: `vllm` not on PATH — run inside the weda-ros-vlm container.")
    os.execvp(argv[0], argv)


if __name__ == "__main__":
    main()
