"""Assemble the GitHub Pages playground into _site/.

Needs em++ (Emscripten) on PATH and a native ./zron (for the scaffold preset):
  make && python3 site/build.py
"""
import json
import os
import re
import shutil
import subprocess
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "_site")


def folder(manifest_path):
    """A manifest plus every script it names, as {name: text}."""
    d = os.path.dirname(manifest_path)
    text = open(manifest_path).read()
    files = {os.path.basename(manifest_path): text}
    for script in re.findall(r"^\s*script\s*=\s*(\S+)", text, re.M):
        files[script] = open(os.path.join(d, script)).read()
    return files


def main():
    shutil.rmtree(OUT, ignore_errors=True)
    os.makedirs(OUT)
    subprocess.run(
        ["em++", "-std=c++17", "-O2", "-Ivendor/chalao", "-Isrc", "src/zron.cpp",
         "-o", os.path.join(OUT, "zron.js"),
         "-sMODULARIZE=1", "-sEXPORT_NAME=createZron", "-sINVOKE_RUN=0",
         "-sEXIT_RUNTIME=1", "-sALLOW_MEMORY_GROWTH=1", "-sFORCE_FILESYSTEM=1",
         "-sEXPORTED_RUNTIME_METHODS=callMain,FS", "-sENVIRONMENT=web,worker",
         "-fexceptions"],   # the interpreter uses C++ exceptions (return, errors)
        cwd=ROOT, check=True)
    for f in ("index.html", "app.js"):
        shutil.copy(os.path.join(ROOT, "site", f), OUT)
    shutil.copy(os.path.join(ROOT, "docs", "img", "driven.gif"), OUT)

    with tempfile.TemporaryDirectory() as tmp:
        subprocess.run([os.path.join(ROOT, "zron"), "new", "mybot"], cwd=tmp, check=True,
                       stdout=subprocess.DEVNULL)
        scaffold = folder(os.path.join(tmp, "mybot", "robot.manifest"))

    presets = [
        ("patrol", "Patrol robot (battery drains, watchdog trips the e-stop)",
         folder(os.path.join(ROOT, "system", "system.manifest"))),
        ("capabilities", "Runtime tour: params, TF tree, RPC, health",
         folder(os.path.join(ROOT, "examples", "capabilities.manifest"))),
        ("restart", "Crashing service, supervisor restarts it",
         folder(os.path.join(ROOT, "tests", "restart.manifest"))),
        ("new", "Blank robot (zron new mybot)", scaffold),
    ]
    data = [{"id": i, "title": t, "files": f,
             "manifest": next(n for n in f if n.endswith(".manifest"))} for i, t, f in presets]
    with open(os.path.join(OUT, "presets.json"), "w") as fh:
        json.dump(data, fh)
    open(os.path.join(OUT, ".nojekyll"), "w").close()
    print("built _site/ with", len(data), "presets")


if __name__ == "__main__":
    main()
