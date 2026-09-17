#!/usr/bin/env python3
"""Render every MuJoCo Menagerie robot to a labelled tile and assemble a
contact-sheet gallery: visual proof of every robot zronOS was validated on."""
import os, glob, sys, json
os.environ.setdefault("MUJOCO_GL", "osmesa")
import numpy as np, mujoco
from PIL import Image, ImageDraw, ImageFont

ROOT="mujoco_menagerie"
OUT=sys.argv[1] if len(sys.argv)>1 else "/tmp/imgcheck/gallery.png"
ONLY=set(sys.argv[2].split(",")) if len(sys.argv)>2 else None
TILE_W, TILE_H = 300, 240
COLS = 8

def font(sz, bold=True):
    p="/usr/share/fonts/truetype/dejavu/DejaVuSansMono%s.ttf"%("-Bold" if bold else "")
    try: return ImageFont.truetype(p, sz)
    except: return ImageFont.load_default()
FL=font(15)

def pick(d):
    xs=[os.path.basename(x) for x in glob.glob(os.path.join(d,"*.xml"))]
    for p in ("scene.xml",): 
        if p in xs: return os.path.join(d,p)
    sc=sorted(x for x in xs if x.startswith("scene"))
    if sc: return os.path.join(d, sc[0])
    named=[x for x in xs if os.path.basename(d).split("_")[-1] in x]
    c=named or [x for x in xs if x!="assets.xml"]
    return os.path.join(d, sorted(c)[0]) if c else None

def render_one(path):
    cwd=os.getcwd()
    try:
        os.chdir(os.path.dirname(path))   # dir-relative includes
        m=mujoco.MjModel.from_xml_path(os.path.basename(path))
    except Exception:
        try:
            m=mujoco.MjModel.from_xml_path(os.path.basename(path))
        except Exception as e:
            os.chdir(cwd); return None
    finally:
        os.chdir(cwd)
    d=mujoco.MjData(m)
    if m.nkey>0:
        mujoco.mj_resetDataKeyframe(m,d,0); mujoco.mj_forward(m,d)   # designed home pose
    else:
        mujoco.mj_forward(m,d)
        for _ in range(40):                                          # let it rest on the floor
            mujoco.mj_step(m,d)
            if not np.all(np.isfinite(d.qpos)): break
    try:
        r=mujoco.Renderer(m, TILE_H, TILE_W)
    except Exception:
        return None
    cam=mujoco.MjvCamera(); mujoco.mjv_defaultFreeCamera(m,cam)
    cam.distance*=1.15; cam.azimuth=130; cam.elevation=-18
    r.update_scene(d, cam)
    px=r.render(); r.close()
    return Image.fromarray(px)

dirs=sorted(d for d in glob.glob(ROOT+"/*") if os.path.isdir(d)
            and os.path.basename(d) not in ("assets","python",".github","test"))
if ONLY: dirs=[d for d in dirs if os.path.basename(d) in ONLY]

tiles=[]
for d in dirs:
    name=os.path.basename(d); p=pick(d)
    im=render_one(p) if p else None
    tile=Image.new("RGB",(TILE_W,TILE_H),(18,20,26))
    if im is not None:
        tile.paste(im,(0,0))
        dr=ImageDraw.Draw(tile)
        label=name.replace("_"," ")
        dr.rectangle([0,TILE_H-22,TILE_W,TILE_H], fill=(0,0,0))
        dr.text((6,TILE_H-20), label[:34], font=FL, fill=(180,230,200))
        print("OK ",name)
    else:
        print("MISS",name)
    tiles.append(tile)

rows=(len(tiles)+COLS-1)//COLS
sheet=Image.new("RGB",(COLS*TILE_W, rows*TILE_H),(10,11,14))
for i,t in enumerate(tiles):
    sheet.paste(t,((i%COLS)*TILE_W,(i//COLS)*TILE_H))
sheet.save(OUT)
print("WROTE",OUT, sheet.size, len(tiles),"tiles")
