# WEDA ROS VLM — Troubleshooting

One guide for every problem — installation, the demo, the model server, the depth camera (including camera swaps from `DEPTH_CAMERA.md`), and your own development on the application (from the developer guide). For any problem, the fastest first move is always:

```bash
./build.sh --bench
```

Every FAIL/WARN prints one line in the terminal; **full verbose diagnostics (tracebacks, command output, fix hints) are written to `weda_ws/reports/*.log`**.

**Pick your entry point:**

| Your situation | Go to |
|---|---|
| Setting up / first run of a quick-start step | Part 1 |
| WISE-Bench shows a FAIL or WARN | Part 2 |
| The app runs but behaves wrong | Part 3 |
| Camera, depth, distances, or a camera swap | Part 4 |
| You are writing / extending code and something broke | Part 5 |
| Still stuck — need raw logs and tracing | Part 6 |

## Table of contents

- [Part 1 — Setup and quick-start issues](#part-1--setup-and-quick-start-issues)
  - [Step 0 — Permission denied running the scripts](#step-0--permission-denied-running-the-scripts)
  - [Step 0 — docker not found or docker.sock permission denied](#step-0--docker-not-found-or-dockersock-permission-denied)
  - [Step 0 — unknown runtime: nvidia](#step-0--unknown-runtime-nvidia)
  - [Step 1 — detector tier idle, EULA not accepted](#step-1--detector-tier-idle-eula-not-accepted)
  - [Step 2 — server takes forever on first start](#step-2--server-takes-forever-on-first-start)
  - [Step 2 — server crash-loops with Free memory less than desired](#step-2--server-crash-loops-with-free-memory-less-than-desired)
  - [Step 3 — container start is slow the first time](#step-3--container-start-is-slow-the-first-time)
  - [Step 3 — container restart-loops right after creation](#step-3--container-restart-loops-right-after-creation)
  - [Step 4 — demo says the perception pipeline is already running](#step-4--demo-says-the-perception-pipeline-is-already-running)
  - [Step 4 — GUI windows don't open, qt.qpa.xcb errors](#step-4--gui-windows-dont-open-qtqpaxcb-errors)
  - [Closing the demo — q or Esc doesn't stop everything](#closing-the-demo--q-or-esc-doesnt-stop-everything)
- [Part 2 — WISE-Bench failures, check by check](#part-2--wise-bench-failures-check-by-check)
  - [T1 — GPU and CUDA foundation (ROS container)](#t1--gpu-and-cuda-foundation-ros-container)
  - [T2 — ROS 2 core](#t2--ros-2-core)
  - [T3 — VLM application stack](#t3--vlm-application-stack)
  - [T4 — Integration and BYOL](#t4--integration-and-byol)
  - [T5 — Environment hygiene](#t5--environment-hygiene)
  - [Server bench S1 to S5 (runs when vlm-server is up)](#server-bench-s1-to-s5-runs-when-vlm-server-is-up)
- [Part 3 — Application runtime issues](#part-3--application-runtime-issues)
  - [Perception publishes status server_unreachable](#perception-publishes-status-server_unreachable)
  - [parse_ok false payloads](#parse_ok-false-payloads)
  - [Response text looks cut off](#response-text-looks-cut-off)
  - [A swapped-in model fails to serve](#a-swapped-in-model-fails-to-serve)
  - [Detector boxes missing while VLM boxes still appear](#detector-boxes-missing-while-vlm-boxes-still-appear)
  - [Topics listed but no images in rviz, panels black](#topics-listed-but-no-images-in-rviz-panels-black)
  - [rqt_graph shows nodes with no connections](#rqt_graph-shows-nodes-with-no-connections)
  - [Display windows didn't open although a display exists](#display-windows-didnt-open-although-a-display-exists)
- [Part 4 — Camera and depth issues](#part-4--camera-and-depth-issues)
  - [Camera gives no frames, no frame yet warnings](#camera-gives-no-frames-no-frame-yet-warnings)
  - [Distances wrong by a constant factor, 10x or 1000x](#distances-wrong-by-a-constant-factor-10x-or-1000x)
  - [Measured distances mostly absent from payloads](#measured-distances-mostly-absent-from-payloads)
  - [depth_context unavailable in payloads](#depth_context-unavailable-in-payloads)
  - [depth_context stale in payloads](#depth_context-stale-in-payloads)
  - [After a camera swap topics exist but nothing arrives](#after-a-camera-swap-topics-exist-but-nothing-arrives)
  - [rviz depth colormap panel stays black](#rviz-depth-colormap-panel-stays-black)
- [Part 5 — Development and extension issues](#part-5--development-and-extension-issues)
  - [My code edit has no effect](#my-code-edit-has-no-effect)
  - [ros2 run says no executable found](#ros2-run-says-no-executable-found)
  - [Config error, missing section](#config-error-missing-section)
  - [Config edits seem ignored](#config-edits-seem-ignored)
  - [My new subscriber receives nothing](#my-new-subscriber-receives-nothing)
  - [Boxes render in the wrong place or clustered top-left](#boxes-render-in-the-wrong-place-or-clustered-top-left)
  - [ros2 topic echo shows truncated text](#ros2-topic-echo-shows-truncated-text)
  - [colcon build fails with --uninstall not recognized](#colcon-build-fails-with---uninstall-not-recognized)
  - [Where do node logs go](#where-do-node-logs-go)
- [Part 6 — Deep debugging](#part-6--deep-debugging)

---

## Part 1 — Setup and quick-start issues

### Step 0 — Permission denied running the scripts
The execute bit was lost (common after archive extraction):
```bash
chmod +x build.sh entrypoint.sh launch_demo.sh
```

### Step 0 — docker not found or docker.sock permission denied
Install Docker Engine, and add your user to the docker group:
```bash
sudo usermod -aG docker $USER   # then log out and back in
```

### Step 0 — unknown runtime: nvidia
```bash
sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker
```

### Step 1 — detector tier idle, EULA not accepted
By design: the licensed detector installs only after acceptance. Accept at the next `./build.sh` prompt (session-scoped), or set
`ACCEPT_COMMERCIAL_EULA=true` in `.env` for persistence, then restart the
container. Without it everything else (VLM description, OCR, depth) still runs.

### Step 2 — server takes forever on first start
Normal: first boot downloads ~17 GB of model weights and captures CUDA graphs (up to an hour). Watch progress: `docker logs weda-vlm-server -f`. Later starts: a few minutes.

### Step 2 — server crash-loops with Free memory less than desired
Jetson unified memory: `gpu_memory_utilization` in the config is a fraction of TOTAL RAM that must actually be **free at engine start**. Default 0.30 boots on a busy box. If a previous engine was killed hard, orphaned memory may be held until the container fully stops — or worst case a host reboot.

### Step 3 — container start is slow the first time
The entrypoint hydrates bundle pieces once per container creation (detector package post-EULA, VLM client, camera SDK, display tools — network needed once), builds the workspace if missing, and the detector exports its TensorRT engine on first run (~3 min, loud). All later starts skip everything.

### Step 3 — container restart-loops right after creation
`./build.sh --logs`. Usual cause: first-start hydration needs network once; offline devices come up in degraded mode with clear WARNs — a crash-loop suggests something else (read the log tail; the workspace self-heal and every hydration step are individually non-fatal by design).

### Step 4 — demo says the perception pipeline is already running
The camera is exclusive; a previous session is still up. Press `q` in its terminal, or: `docker exec weda-ros-vlm pkill -INT -f "ros2 launch"`.

### Step 4 — GUI windows don't open, qt.qpa.xcb errors
Run from a desktop session so `build.sh` detects `DISPLAY`, and allow local containers on X: `xhost +local:docker`. Headless is automatic and healthy — topics only. **If the container was created from a shell without a display** (e.g. over plain SSH), GUIs will fail even later: recreate it via `./build.sh` from a desktop session — creation-time environment is what counts.

### Closing the demo — q or Esc doesn't stop everything
All close paths (terminal q/Esc, viewer window close, Ctrl+C) end with `Demo stopped — camera released.` If a window ever survives a hard kill of the terminal, run the demo once and quit normally — cleanup sweeps leaked helpers from previous sessions.

---

## Part 2 — WISE-Bench failures, check by check

Run: `./build.sh --bench`. The `.log` in `weda_ws/reports/` carries the full diagnostics for anything flagged. Below: every check, its meaning on failure, and the remedy.

### T1 — GPU and CUDA foundation (ROS container)

| Check | If it fails | Remedy |
|---|---|---|
| PyTorch CUDA available | container has no GPU access | verify `runtime: nvidia` + `NVIDIA_VISIBLE_DEVICES` in compose; `nvidia-ctk runtime configure`; recreate |
| CUDA GEMM correctness | GPU computes wrong numbers | driver/toolkit mismatch — check JetPack vs image generation; reboot the device |
| OpenCV CUDA build | cv2 is CPU-only | image corruption or a pip install replaced cv2 — never install packages that depend on `opencv-python`; recreate the container |
| TensorRT functional (builder/network/config) | TRT runtime broken | image issue — re-pull the base image |
| torchvision CUDA ops | compiled vision ops broken | image issue — re-pull |
| OpenCV CUDA compute | GPU image ops fail despite CUDA build | image issue; check no stray pip install replaced cv2 |
| onnxruntime | ONNX inference/providers broken | image issue — re-pull; providers should list Tensorrt/CUDA/CPU |

### T2 — ROS 2 core

| Check | If it fails | Remedy |
|---|---|---|
| ROS distro environment | `ROS_DISTRO` unset | start via `./build.sh` (compose sets it), not bare `docker run` |
| rclpy init/shutdown | ROS python runtime broken | re-pull image |
| DDS pub/sub round-trip | messages don't deliver | stale shared-memory segments — with ALL ROS stopped: `rm -f /dev/shm/fastrtps_* /dev/shm/fast_datasharing* /dev/shm/sem.fastrtps_*` (or restart the container) |
| MultiThreadedExecutor + timer | executor machinery broken | image issue; re-pull |
| TF static broadcast round-trip | tf2 stack broken | image issue; re-pull |
| cv_bridge numpy round-trip | image conversion corrupts data | conflicting numpy/cv2 install — check for stray pip installs |
| QoS profiles | rclpy API mismatch | image issue |
| message_filters / sensor_msgs | ROS packages missing | re-pull image |
| image-transport-plugins | rviz image panels will error | hydrated at start — rerun with network once, or `apt-get install ros-jazzy-image-transport-plugins` in the container |
| ros2 CLI | CLI daemon broken | `ros2 daemon stop` then retry; check `RMW_IMPLEMENTATION` |
| colcon present | workspace can't build | image issue |
| rviz2 / rqt_graph present | display tools missing | hydrated at start (network once); harmless on headless devices |

### T3 — VLM application stack

| Check | If it fails | Remedy |
|---|---|---|
| JPEG encode/decode | frame→VLM path broken | cv2 build issue — re-pull |
| GStreamer pipeline grab | RGB-only mode can't capture | gst plugins missing from image — re-pull; check `stream.gst_pipeline` syntax if you edited it |
| openai | VLM client missing | hydration needs network once; or `uv pip install --system openai==2.38.0` in the container |
| yaml | config loader missing | image issue |
| vlm_app.yaml loads + sections | config file broken | restore from git/backup; every required section must exist (error lists what's missing) |
| unified-memory budget (INFO) | informational: required = config util × total RAM (computed, never hardcoded); compare with the shown available | if required > available, free RAM before `--server` or lower `gpu_memory_utilization` |
| ultralytics licensed detector (SKIP/WARN) | SKIP: license not accepted — the licensed tier is untested by design. WARN: EULA accepted but the package is missing | SKIP: accept at the `./build.sh` prompt (or set `ACCEPT_COMMERCIAL_EULA=true` in `.env` for persistence) and re-run — the report log carries the full guidance. WARN: restart the container with network (hydration installs it) |
| Qt binding | viewer can't run | image issue; viewer optional on headless |
| vlm_perception modules import | app code broken | read the `.log` traceback — usually an edit introduced an import error; `colcon build` output shows details |
| units: box canonicalize + depth | core math regression | you edited `depth_processor.py` — the `.log` shows which case; revert or fix |
| units: JSON parser battery | parser regression | same — the parse contract (strict/loose/repair/never-silent) must hold |
| units: streaming/reasoning battery | early-publish regression | same — see the failing case in the `.log` |
| vlm_perception registration | workspace not built correctly | `cd /opt/weda_ws && rm -rf build install log && colcon build --symlink-install` |
| launch file syntax | launch edit broke it | the `.log` has the compile error |
| vlm-serve --dry-run | server launcher broken | run it manually to see the full error: `python3 .../serve_vlm.py --dry-run` |

### T4 — Integration and BYOL

| Check | If it fails | Remedy |
|---|---|---|
| vlm-server link + live completion (WARN) | server down or still loading | `./build.sh --server`; first boot up to an hour; `docker logs weda-vlm-server` |
| BYOL gate (ultralytics) FAIL | EULA accepted but hydration failed | network needed once; `docker logs weda-ros-vlm \| grep -i ultralytics` |
| detector TensorRT engine (SKIP) | not built yet | normal — first detector run exports it (~3 min) |

### T5 — Environment hygiene

| Check | If it fails | Remedy |
|---|---|---|
| bundle mount / workspace mount | compose volumes changed | restore the volume lines in `docker-compose.yml` |
| workspace writable | builds/engine can't persist | check host permissions on `weda_ws/` |
| uv / strace present (WARN) | tooling missing | hydrated at start with network |
| DDS shared-memory segments (WARN) | stale segments may break delivery | with all ROS stopped: `rm -f /dev/shm/fastrtps_* /dev/shm/fast_datasharing* /dev/shm/sem.fastrtps_*` |
| DISPLAY (SKIP) | headless | expected; not a defect |

### Server bench S1 to S5 (runs when vlm-server is up)

| Check | If it fails | Remedy |
|---|---|---|
| S1 CUDA / GEMM | server container lost GPU | same GPU remedies as T1, for the `vlm-server` service |
| S2 vllm + CUDA ops | serving engine broken | re-pull the server image |
| S2 flashinfer / transformers / tokenizer / Triton JIT | serving stack component broken | re-pull server image; the `.log` names the component |
| S2 nvcc (PASS with note) | — | informational: resolved via CUDA_HOME, not PATH |
| S3 engine process / health / models (WARN) | engine loading or crashed | wait for first-boot load; `docker logs weda-vlm-server --tail 50` |
| S3 LIVE text / image completion (WARN) | API up but inference failing | check model finished loading; the `.log` carries the exception |
| S4 config / launcher | `server:` section broken | fix `vlm_app.yaml`; `vlm-serve --dry-run` shows the assembled command |
| S4 HF weights cache | cache mount missing/unwritable | check the compose volume for `/models/huggingface`; without it every restart re-downloads |
| S4 memory budget (INFO) | informational: required is computed from the live config for any model | if required > available, free RAM before starting the engine |
| S4 model-swap readiness (WARN) | Nemotron-family deps missing | these ship in the serving image — a WARN means the container runs an outdated image: update/re-pull it (stopgap: `docker exec weda-vlm-server pip install timm open-clip-torch`) |
| S5 HF_HOME / EULA doc / uv | environment drift | compare against a fresh `--server` start |

---

## Part 3 — Application runtime issues

### Perception publishes status server_unreachable
The VLM server isn't up or is still loading. The ROS side keeps running and recovers automatically once `curl -sf http://127.0.0.1:8000/health` returns 200.

### parse_ok false payloads
The model reply wasn't valid JSON. The raw text is preserved in the payload — failures are loud, never silent. Persistent with a swapped model? Reasoning models need `perception.max_tokens: 2048+`; check `model_profile:` settings.

### Response text looks cut off
In the annotated window the full description and OCR are word-wrapped — extremely long replies are capped at 8 + 4 banner lines to protect the video area. The complete text is always in the `/vlm/perception` payload.

### A swapped-in model fails to serve
Follow the 3-line swap recipe in the config (`SWAP NOTE`s): new `model:`, `kv_cache_dtype: auto`, `attention_backend: null`, then `docker restart weda-vlm-server` (the YAML is read once at engine start). `ImportError` in server logs → the checkpoint needs a package beyond the serving stack (Nemotron-family vision-tower deps already ship in the image). Gated repos need `HF_TOKEN` in `.env`.

### Detector boxes missing while VLM boxes still appear
Tier 1 is idle: EULA not accepted (see Part 1, Step 1), engine still exporting on first run, or `detector.enabled: false`. Check `docker logs weda-ros-vlm | grep -i detector`.

### Topics listed but no images in rviz, panels black
Stale DDS shared-memory segments from hard-killed processes. With all ROS stopped: `rm -f /dev/shm/fastrtps_* /dev/shm/fast_datasharing* /dev/shm/sem.fastrtps_*` (inside the container), or restart the container. WISE-Bench detects this state.

### rqt_graph shows nodes with no connections
`/tf_camera_color` and `/tf_camera_depth` publish one latched `/tf_static` message each — rqt_graph's default filters hide tf topics, so the nodes render isolated. Working as intended; untick "Hide → tf" to see the edges.

### Display windows didn't open although a display exists
Check the announced decision line: `windows per config: rviz=? rqt_graph=? viewer=?`
— the `display:` section of the config controls each window. Also see the
Step 4 X11 entry in Part 1 (creation-time environment).

---

## Part 4 — Camera and depth issues

Covers the shipped Orbbec camera **and** camera swaps. The architecture, swap options (A/B/C), and full decision + symptom trees live in `DEPTH_CAMERA.md` — this section is the symptom-first index into all of it.

### Camera gives no frames, no frame yet warnings
- The depth camera must be on USB3 and not held by another process — only one pipeline may own it (see the already-running entry in Part 1, Step 4).
- Re-plug the camera if it stopped enumerating; then restart the demo.
- No camera at all? Set `depth.enabled: false` — the app runs RGB-only from the configured `stream:` source (test pattern by default; USB/RTSP/file variants are commented in the config).

### Distances wrong by a constant factor, 10x or 1000x
`depth_scale` mismatch (Contract 2 in `DEPTH_CAMERA.md`): the wire format and the multiplier must agree. `16UC1` uint16 millimeters → `depth_scale: 0.001`; `32FC1` float32 meters → `depth_scale: 1.0` (the float path ignores scale automatically). Set it in the `depth:` section of `config/vlm_app.yaml`.

### Measured distances mostly absent from payloads
`measured_distance_m` missing per object means one of:
- **Depth not aligned to color** (Contract 1): the depth topic must be the *aligned/registered* one — enable your vendor driver's "align depth to color" mode and point `depth.depth_topic` at that topic.
- `depth.min_m` / `depth.max_m` don't match the camera's rated range — everything falls outside the valid window.
- The quality gate is doing its job: a box needs ≥30 valid depth pixels AND ≥5% of its region valid, otherwise the value is deliberately absent — no value beats a confidently wrong value.

### depth_context unavailable in payloads
- With the Orbbec: `vlm-depth` isn't running, the camera is unplugged, or another process holds it.
- After an Option A camera swap: **expected** — the vendor driver doesn't publish `/vlm/depth_context`; only the prompt enrichment is lost. Option B (relay, ~30 lines) restores it.
- Under Option B: the relay node isn't running.

### depth_context stale in payloads
The depth stream stopped or slowed (camera fps drop, driver stall). By design, data older than `depth.max_context_age_s` (default 2 s) is never fed to the model and per-object measurement is skipped — fix the stream, not the guard.

### After a camera swap topics exist but nothing arrives
- Wrong topic names: compare `ros2 topic list` against the `depth:` section; verify flow with `ros2 topic hz <topic>`.
- The pipeline's consumers subscribe best-effort sensor-data QoS, which matches both reliable and best-effort vendor drivers — but **your own** default-QoS subscriber will silently receive nothing from a best-effort publisher (see the QoS entry in Part 5).

### rviz depth colormap panel stays black
The colormap topic is published best-effort — rviz must subscribe with the sensor-data profile (the shipped preset does). Also check the stale-DDS remedy in Part 3 if all panels are black.

---

## Part 5 — Development and extension issues

You are editing the application or adding your own nodes. The full context — architecture, contracts, a worked add-a-node example, extension recipes — is the developer guide at `weda_ws/src/vlm_perception/README.md`; below are the failures you'll actually hit, symptom-first.

### My code edit has no effect
This colcon setup **copies** Python modules into `install/` — `ros2 run` executes the copy, not your edited file. Rebuild and re-source:
```bash
cd /opt/weda_ws && colcon build && source install/setup.bash
```
`./build.sh --demo` detects sources newer than the install stamp and rebuilds
automatically.

### ros2 run says no executable found
- Executable names are **hyphenated**: `vlm-depth`, not `vlm_depth`.
- The overlay isn't sourced: use `./build.sh --shell` (sources everything), or `source /opt/weda_ws/install/setup.bash` in your current shell.
- For a brand-new node: is the `console_scripts` line added in `setup.py`? Did you rebuild *and* re-source afterwards?
- Don't delete `setup.cfg` — its `script_dir`/`install_scripts` lines are what place executables where `ros2 run`/`ros2 launch` look.

### Config error, missing section
`load_config` fails loudly if any of the six required sections is absent:
`server, client, stream, perception, depth, output`. Restore the missing one.
Your own feature sections must stay **optional** — read them with
`cfg.get("yoursection") or {}` so the config never grows new hard requirements.

### Config edits seem ignored
- Edit on the **host** (`config/vlm_app.yaml` in the bundle) — the container mounts the bundle read-only at `/opt/weda-release`.
- ROS side: restart the pipeline (`q`, then `./build.sh --demo`).
- Server side: `docker restart weda-vlm-server` — the YAML is read once at engine start; a plain `./build.sh --server` on a running server is a no-op.

### My new subscriber receives nothing
- `/vlm/annotated/compressed` and the depth colormap are **best-effort** publishers: subscribe with `qos_profile_sensor_data`, not default QoS.
- All `/vlm/*` JSON topics are `std_msgs/String` at plain depth-10 — check you subscribed with the matching type.
- Everything correct and still silent? Stale DDS shared memory — see the rviz panels entry in Part 3.

### Boxes render in the wrong place or clustered top-left
`box_2d` is **canonical 0–1000 normalized**, not pixels. Scale at draw time:
`x_px = x / 1000 * frame_width` (same for y/height). Forgetting the scaling puts every box near the origin.

### ros2 topic echo shows truncated text
`ros2 topic echo` truncates long string fields — that's the tool, not the payload. Use the bundled printer: `ros2 run vlm_perception vlm-feed`.

### colcon build fails with --uninstall not recognized
Stale state from an older image generation (the entrypoint self-heals this on restart): `cd /opt/weda_ws && rm -rf build install log && colcon build --symlink-install`.

### Where do node logs go
Launch-file nodes print to the demo terminal (`output=screen`); file logs land under `/opt/weda_ws/log` (colcon) and `~/.ros/log` (nodes); from the host,`./build.sh --logs` tails the container.

---

## Part 6 — Deep debugging

- **Reports**: `weda_ws/reports/*.log` — verbose diagnostics for every bench FAIL/WARN.
- **Demo log**: `/tmp/vlm_demo_launch.log` (inside the container) — full node output.
- **Server log**: `docker logs weda-vlm-server`.
- **Syscall tracing**: `strace` is available in the ROS container (`docker exec weda-ros-vlm strace -f -e trace=openat -p <PID>`) — the fastest way to answer "what file is this process actually trying to open?"
