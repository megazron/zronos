#!/usr/bin/env python3
"""zron <-> MuJoCo bridge.

A tiny, dependency-light physics backend for zronOS. It loads any MuJoCo MJCF
model and speaks newline-delimited JSON on stdin/stdout, so a zronOS HAL (or the
compatibility harness) can drive real physics for *any* robot model:

    {"cmd":"info"}                         -> model dims, actuator names, ctrl ranges
    {"cmd":"step","ctrl":[...],"n":100}    -> step n times with a control vector
    {"cmd":"reset"}                        -> reset to the initial (or keyframe) state
    {"cmd":"quit"}

Every reply is one JSON object. `finite` is false if the sim diverged (NaN/Inf),
which is how the harness detects an incompatible or unstable model.

Usage:  python3 zron_mujoco.py <model.xml>
Requires: mujoco (pip install mujoco).  Tested with MuJoCo 3.13.
"""
import json, os, sys
import numpy as np
import mujoco


def load(path):
    # MuJoCo resolves includes/assets relative to the XML file; preserve the
    # caller's working directory so repeated loads and relative paths work.
    path = os.path.abspath(path)
    model = mujoco.MjModel.from_xml_path(path)
    data = mujoco.MjData(model)
    # start from keyframe 0 if the model ships one (a valid standing/home pose)
    if model.nkey > 0:
        mujoco.mj_resetDataKeyframe(model, data, 0)
    else:
        mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    return model, data


def actuator_names(model):
    names = []
    for i in range(model.nu):
        n = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        names.append(n if n else f"act{i}")
    return names


def info(model):
    cr = model.actuator_ctrlrange.copy()
    lim = model.actuator_ctrllimited.copy()
    ranges = []
    for i in range(model.nu):
        if lim[i]:
            ranges.append([float(cr[i, 0]), float(cr[i, 1])])
        else:
            ranges.append([0.0, 0.0])   # unlimited -> neutral 0
    return {
        "ok": True, "nu": int(model.nu), "nq": int(model.nq), "nv": int(model.nv),
        "nsensor": int(model.nsensor), "nbody": int(model.nbody),
        "nkey": int(model.nkey), "dof": int(model.nv),
        "ctrlrange": ranges, "actuators": actuator_names(model),
    }


def step(model, data, ctrl, n):
    # Validate the entire request before mutating controls or advancing time.
    if isinstance(n, bool) or not isinstance(n, int) or not 1 <= n <= 10000:
        raise ValueError("n must be an integer between 1 and 10000")
    controls = None
    if ctrl is not None:
        if not isinstance(ctrl, list) or len(ctrl) != model.nu:
            raise ValueError(f"ctrl must contain exactly {model.nu} numbers")
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in ctrl):
            raise ValueError("ctrl must contain finite numbers")
        controls = np.asarray(ctrl, dtype=float)
        if not np.isfinite(controls).all():
            raise ValueError("ctrl must contain finite numbers")
    if controls is not None:
        data.ctrl[:] = controls
    for _ in range(n):
        mujoco.mj_step(model, data)
    q = np.asarray(data.qpos, dtype=float)
    qv = np.asarray(data.qvel, dtype=float)
    finite = bool(np.all(np.isfinite(q))) and bool(np.all(np.isfinite(qv)))
    finite = finite and bool(np.isfinite(data.sensordata).all()) and bool(np.isfinite(data.time))
    qvel_max = float(np.max(np.abs(qv))) if qv.size and finite else (0.0 if finite else None)
    clean = lambda values: [float(v) if np.isfinite(v) else None for v in values]
    return {
        "ok": True, "time": float(data.time) if np.isfinite(data.time) else None, "finite": finite,
        "stable": finite, "qvel_max": qvel_max,
        "qpos": clean(q[: min(len(q), 32)]),
        "sensordata": clean(np.asarray(data.sensordata, dtype=float)[:16]),
    }


def main():
    if len(sys.argv) < 2:
        print(json.dumps({"ok": False, "error": "usage: zron_mujoco.py <model.xml>"}))
        return 2
    model_path = os.path.abspath(sys.argv[1])
    try:
        model, data = load(model_path)
    except Exception as e:
        print(json.dumps({"ok": False, "error": f"load failed: {e}"}), flush=True)
        return 1
    print(json.dumps({"ok": True, "loaded": sys.argv[1]}), flush=True)
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            req = json.loads(line)
        except Exception as e:
            print(json.dumps({"ok": False, "error": f"bad json: {e}"}), flush=True); continue
        if not isinstance(req, dict):
            print(json.dumps({"ok": False, "error": "request must be a JSON object"}), flush=True)
            continue
        cmd = req.get("cmd")
        if cmd == "quit":
            break
        elif cmd == "info":
            print(json.dumps(info(model)), flush=True)
        elif cmd == "reset":
            if model.nkey > 0:
                mujoco.mj_resetDataKeyframe(model, data, 0)
            else:
                mujoco.mj_resetData(model, data)
            mujoco.mj_forward(model, data)
            print(json.dumps({"ok": True}), flush=True)
        elif cmd == "step":
            try:
                print(json.dumps(step(model, data, req.get("ctrl"), req.get("n", 1))), flush=True)
            except Exception as e:
                print(json.dumps({"ok": False, "error": f"step failed: {e}"}), flush=True)
        else:
            print(json.dumps({"ok": False, "error": f"unknown cmd {cmd}"}), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
