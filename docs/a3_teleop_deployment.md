# A3 teleop deployment (Orin + onboard)

This document is the delivery entry point for the teleoperation work that lives on
the `feat/a3-streaming-reference` branch of this repository.

It answers three questions:

1. what this branch adds to `sonic_for_a3`,
2. how to get the code onto GitHub / the robot (including the **deploy public key**),
3. how to deploy and run PICO teleoperation on Jetson Orin + the A3 onboard computer.

The full, copy-paste runbooks live in the companion repository
`a3_teleop_bridge`:

| Document | Content |
| --- | --- |
| `a3_teleop_bridge/docs/DEPLOY_ORIN.md` | Orin: preflight, aarch64 environment rebuild, licensed SMPL-X asset, 16-item readiness gate, network, PICO + online UMR live start, MuJoCo consumer check, fallback |
| `a3_teleop_bridge/docs/A3_ONBOARD.md` | A3 onboard: rockchip package build, MDU service configuration, receive-only probe, streaming reference integration, gantry bring-up order, safety thresholds |
| `a3_teleop_bridge/docs/DELIVERY.md` | deliverables, target repositories, deploy key, push commands, offline bundle path |
| `a3_teleop_bridge/docs/progress.md` | every stage with commit SHA, commands, inputs, outputs, results |

---

## 1. What this branch adds (no upstream change)

```text
gear_sonic/utils/reference_provider.py                     ReferenceProvider abstraction
gear_sonic/scripts/sim2sim_a3_mujoco.py                    --reference-source stream, --realtime
gear_sonic_deploy/.../include/a3_deploy/a3_reference_limits.hpp
gear_sonic_deploy/.../include/a3_deploy/a3_reference_stream.hpp
gear_sonic_deploy/.../src/a3_deploy/a3_reference_stream.cpp
gear_sonic_deploy/.../unit_tests/test_a3_reference_stream_standalone.cpp
gear_sonic_deploy/.../CMakeLists.txt
```

`main` is untouched; the CSV playback path is byte-for-byte unchanged (regression
tested: `reference_provider` keeps the legacy observations identical, max diff 0.0).

The streaming protocol is frozen:

```text
A3_REFERENCE_V1 = b"A3R1" | uint32 header_len | msgpack header | float32 payload
payload = root_pos[10,3] | root_quat_wxyz[10,4] | joint_pos_rad[10,29] | joint_vel_rad_s[10,29]
```

**Joint order matters**: the wire payload is the *encoder / IsaacLab* order
(`joint_order: a3_il_v1`), which is a real permutation of the policy order used by
the A3 runtime and of the 31-column CSV order. Map by name using
`a3_teleop_bridge/generated/a3_contract.json`; never index by position.

---

## 2. Deploy public key

The packaging machine holds a dedicated key pair for pushing these repositories:

```text
private key : ~/.ssh/id_ed25519_github_a3      (machine-local, mode 600, never commit)
public key  : ~/.ssh/id_ed25519_github_a3.pub
fingerprint : SHA256:WJ5l9QV0feolYc1LTRAOn0Tjd4G44CBCi/M0E2NQn0o
comment     : a3-teleop-orin-deploy
```

Public key (copy this whole line):

```text
ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIDrJ4lUXNPn69rNCjgRhkAieGpu3AMAnOzjeAaeTCXPi a3-teleop-orin-deploy
```

Add it on GitHub: **avatar → Settings → SSH and GPG keys → New SSH key → paste → Add**.
Use an *account-level* SSH key (a single public key can only be a deploy key for one
repository, and we push more than one).

Verify from the packaging machine:

```bash
ssh -T git@github.com      # expect: Hi <account>! You've successfully authenticated...
```

---

## 3. Push

```bash
# this repository: feature branch only
git push git@github.com:Restar7/sonic_for_a3.git feat/a3-streaming-reference

# the companion repository (create an EMPTY repo Restar7/a3_teleop_bridge first)
cd ../a3_teleop_bridge
git remote add origin git@github.com:Restar7/a3_teleop_bridge.git
git push -u origin main
```

Offline alternatives (git bundles) and the 14 MB Orin package are described in
`a3_teleop_bridge/docs/DELIVERY.md` §1/§5.

---

## 4. Deployment in short

### 4.1 Package (on the development machine)

```bash
cd ~/a3_teleop_ws/a3_teleop_bridge
bash scripts/package_bundle.sh --with-umr
bash scripts/sync_to_orin.sh --bundle ~/a3_teleop_ws/dist/a3_teleop_orin_<stamp>.tar.gz --host <ORIN_IP>
```

The bundle contains this repository's code and A3 assets (MJCF/URDF/meshes, the
`a3_loop` solver library, the RKNN runtime, the C++ reference stream) but **no
weights and no SMPL-X body model** — SMPL-X is a licensed asset and must be copied
by hand.

### 4.2 On the Orin

```bash
cd ~/a3_teleop_ws/a3_teleop_bridge
source scripts/env_orin.sh              # exports SONIC_A3_ROOT, UMR_ROOT, PY_*
bash scripts/orin_preflight.sh          # system / deps / zmq report
bash scripts/orin_bootstrap.sh --with-umr
bash scripts/check_orin_ready.sh        # 16-item gate, includes solving one online UMR frame
```

Run the live stack (PICO → SMPL → online UMR → predictor → publisher):

```bash
# terminal 1: PICO sender (XRoboToolkit PC service must be connected to the headset)
cd ~/a3_teleop_ws/sonic_for_a3
.venv_sim/bin/python gear_sonic/scripts/pico_pose_zmq_minimal.py --port 5556

# terminal 2: reference generation, published for the A3 / MuJoCo consumer
cd ~/a3_teleop_ws/a3_teleop_bridge && source scripts/env_orin.sh
bash scripts/run_orin_live.sh --duration 600 --endpoint tcp://0.0.0.0:5560 \
     --save-calibration calibration.json
```

> The PICO pose port is **5556** — the default of both
> `gear_sonic/scripts/pico_pose_zmq_minimal.py` (this repo) and GR00T's
> `pico_manager_thread_server.py`. Earlier bridge configs used 5561, which would
> have silently dropped every PICO frame; it is fixed and aligned on 5556.

Verify without the robot (workstation consuming the Orin stream in MuJoCo):

```bash
bash scripts/run_mujoco_consumer.sh --endpoint tcp://<ORIN_IP>:5560 \
     --motion ~/a3_teleop_ws/logs/a3_validation/endurance_loop.csv --steps 3000
```

### 4.3 On the A3 (MDU / RK3588)

```bash
# build the deploy package on the x86 development machine
python download_from_hf.py --component sysroot
export A3_ONNXRUNTIME_AARCH64_TARBALL='/absolute/path/onnxruntime-aarch64-1.19.2.tar.gz'
gear_sonic_deploy/scripts/build_a3_deploy_pkg.sh --arch rockchip --jobs 20 \
  --onnxruntime-aarch64-tarball "$A3_ONNXRUNTIME_AARCH64_TARBALL" \
  --runtime-cfg gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/config/a3_runtime_config.yaml
```

Then, on the MDU: make the Motion function group start only the agent, restart
`agibot_pm`, confirm `motion_control` is not running, run the **no-command-publish
probe** first, and only then the formal launch:

```bash
export A3_ROBOT_ENV=/agibot/software/v0/entry/env/env.sh
A3_TRANSPORT=iceoryx A3_PROBE_SOURCE=g1 A3_LATENCY_LOG=verbose taskset -c 4-5 ./run_a3_probe.sh
taskset -c 4-5 ./run_a3.sh
```

The onboard runtime already consumes a teleop reference through
`A3TeleopReference` from `/ta/whole_body_command` (`q_mujoco[29]`, `dq_mujoco[29]`,
head). The recommended integration therefore adds **no vendor C++ change**: a thin
adapter on the MDU decodes `A3_REFERENCE_V1` with `A3ReferenceStream` (already
implemented and unit-tested here), permutes the il-order joints into policy order,
and publishes that channel. Full detail, including the alternative in-runtime route
and the safety thresholds that must not be relaxed, is in
`a3_teleop_bridge/docs/A3_ONBOARD.md`.

Gantry bring-up order (stop at the first failure): stand → shoulder → elbow →
wrist → torso small rotation → knee small bend → weight shift → foot unload →
foot lift → slow single step. Safety gantry, fall arrest, physical E-stop and an
on-site supervisor are mandatory; the reference watchdog (50 ms frame age) must
keep the official values.
