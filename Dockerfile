# ==============================================================================
# WEDA ROS VLM — Custom Image Template (OPTIONAL)
# ------------------------------------------------------------------------------
# This bundle runs on the SHARED WEDA vision base image by default; the
# bundle-specific runtime pieces hydrate at container start via entrypoint.sh.
# You do NOT need this file to use the product.
#
# Use it when you want a self-contained image (e.g. air-gapped fleets):
#   docker build -t weda-ros-vlm:custom .
#   then set  WEDA_ROS_VLM_IMAGE=weda-ros-vlm:custom  in .env
# ==============================================================================

ARG BASE_IMAGE=harbor.edgesync.cloud/weda-ai/weda-ros-cv:1.0.0-thor
FROM ${BASE_IMAGE}

WORKDIR /opt/weda_ws

# ── RECOMMENDED BAKE (uncomment for offline/air-gapped fleets) ─────────────────
# The exact set the entrypoint otherwise hydrates once per fresh container:
#
# RUN uv pip install --system --no-cache-dir "openai==2.38.0" \
#     && uv pip install --system --no-cache-dir --no-deps "pyorbbecsdk2==2.1.1"
# RUN apt-get update && apt-get install -y --no-install-recommends \
#         ros-jazzy-rviz2 ros-jazzy-image-transport-plugins strace \
#     && rm -rf /var/lib/apt/lists/*
#
# LICENSING NOTE: commercially licensed packages (Ultralytics detector) and
# model weights must NOT be baked — they hydrate at container start only
# after EULA acceptance. Keep it that way to stay license-compliant.

# ── YOUR CUSTOMIZATIONS ────────────────────────────────────────────────────────
# COPY my_ros_pkg /opt/weda_ws/src/my_ros_pkg
# RUN . /opt/ros/${ROS_DISTRO:-jazzy}/setup.sh \
#     && cd /opt/weda_ws && colcon build --symlink-install

# The compose file supplies the runtime entrypoint, GPU access, devices and
# mounts exactly as with the shared base image.
