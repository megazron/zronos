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
    # Load from inside the model's own directory so relative <include>/asset
    # paths resolve for every Menagerie model (some nest includes).
    path = os.path.abspath(path)
    d = os.path.dirname(path)
    if d:
        os.chdir(d)
    model = mujoco.MjModel.from_xml_path(os.path.basename(path))
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
    if ctrl is not None and len(ctrl) == model.nu:
        data.ctrl[:] = np.asarray(ctrl, dtype=float)
    for _ in range(n):
        mujoco.mj_step(model, data)
    q = np.asarray(data.qpos, dtype=float)
    qv = np.asarray(data.qvel, dtype=float)
    finite = bool(np.all(np.isfinite(q))) and bool(np.all(np.isfinite(qv)))
    qvel_max = float(np.max(np.abs(qv))) if qv.size and finite else (0.0 if finite else float("inf"))
    return {
        "ok": True, "time": float(data.time), "finite": finite,
        "stable": finite, "qvel_max": qvel_max,
        "qpos": q[: min(len(q), 32)].tolist(),
        "sensordata": np.asarray(data.sensordata, dtype=float)[:16].tolist(),
    }


def main():
    if len(sys.argv) < 2:
        print(json.dumps({"ok": False, "error": "usage: zron_mujoco.py <model.xml>"}))
        return 2
    try:
        model, data = load(sys.argv[1])
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
        cmd = req.get("cmd")
        if cmd == "quit":
            break
        elif cmd == "info":
            print(json.dumps(info(model)), flush=True)
        elif cmd == "reset":
            model, data = load(sys.argv[1]); print(json.dumps({"ok": True}), flush=True)
        elif cmd == "step":
            try:
                print(json.dumps(step(model, data, req.get("ctrl"), int(req.get("n", 1)))), flush=True)
            except Exception as e:
                print(json.dumps({"ok": False, "error": f"step failed: {e}"}), flush=True)
        else:
            print(json.dumps({"ok": False, "error": f"unknown cmd {cmd}"}), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
