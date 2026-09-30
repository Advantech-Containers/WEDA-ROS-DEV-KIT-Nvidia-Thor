"""Shared, dependency-light helpers (safe to import under ANY python)."""

import os
import string

import yaml

DEFAULT_CONFIG = os.environ.get("VLM_APP_CONFIG", "/opt/weda-release/config/vlm_app.yaml")


def load_config(path: str) -> dict:
    try:
        with open(path) as f:
            raw = f.read()
        # ${VAR} env interpolation — secrets stay in .env, never in this file.
        cfg = yaml.safe_load(string.Template(raw).safe_substitute(os.environ))
    except FileNotFoundError:
        raise SystemExit(f"[vlm_perception] FATAL: config not found: {path}")
    except yaml.YAMLError as e:
        raise SystemExit(f"[vlm_perception] FATAL: invalid YAML in {path}: {e}")
    if not isinstance(cfg, dict):
        raise SystemExit(f"[vlm_perception] FATAL: {path} is empty or not a YAML mapping")
    for section in ("server", "client", "stream", "perception", "depth", "output"):
        if section not in cfg:
            raise SystemExit(f"[vlm_perception] config error: missing '{section}:' section in {path}")
    return cfg
