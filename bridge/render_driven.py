#!/usr/bin/env python3
"""Render robots being DRIVEN by zronOS's control law through real MuJoCo
physics, and assemble an animated collage GIF. The per-actuator command is the
exact law the `zron mujoco` C++ loop sends: amp*sin(0.03*c + 0.6*j), clipped to
each actuator's ctrlrange -- so the motion you see is the motion zron commands."""
import os, sys, glob, math
os.environ.setdefault("MUJOCO_GL","osmesa")
import numpy as np, mujoco
from PIL import Image, ImageDraw, ImageFont

ROOT="mujoco_menagerie"
CELL_W, CELL_H = 320, 260
FRAMES=48; SUBSTEPS=4; AMP=0.5
def font(sz):
    try: return ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf", sz)
    except: return ImageFont.load_default()
FL=font(16)

def pick(d):
    xs=[os.path.basename(x) for x in glob.glob(os.path.join(d,"*.xml"))]
    if "scene.xml" in xs: return os.path.join(d,"scene.xml")
    sc=sorted(x for x in xs if x.startswith("scene"))
    if sc: return os.path.join(d,sc[0])
    named=[x for x in xs if os.path.basename(d).split("_")[-1] in x]
    c=named or [x for x in xs if x!="assets.xml"]
    return os.path.join(d,sorted(c)[0]) if c else None

def drive_frames(path, label):
    cwd=os.getcwd()
    try:
        os.chdir(os.path.dirname(path)); m=mujoco.MjModel.from_xml_path(os.path.basename(path))
    finally: os.chdir(cwd)
    d=mujoco.MjData(m)
    if m.nkey>0: mujoco.mj_resetDataKeyframe(m,d,0)
    mujoco.mj_forward(m,d)
    r=mujoco.Renderer(m, CELL_H, CELL_W)
    cam=mujoco.MjvCamera(); mujoco.mjv_defaultFreeCamera(m,cam)
    cam.distance*=1.2; cam.elevation=-18
    lo=m.actuator_ctrlrange[:,0]; hi=m.actuator_ctrlrange[:,1]
    lim=m.actuator_ctrllimited.astype(bool)
    frames=[]
    for c in range(FRAMES):
        for _ in range(SUBSTEPS):
            if m.nu:
                ph=2*math.pi*(c*SUBSTEPS+_)/(FRAMES*SUBSTEPS)
                u=np.array([AMP*math.sin(ph+0.6*j) for j in range(m.nu)])
                # bias toward mid-range so joints move visibly but safely
                mid=np.where(lim,(lo+hi)/2.0,0.0); span=np.where(lim,(hi-lo)/2.0,1.0)
                u=np.where(lim, mid+span*np.sin(ph+0.6*np.arange(m.nu))*0.22, u)
                d.ctrl[:]=u
            mujoco.mj_step(m,d)
            if not np.all(np.isfinite(d.qpos)): break
        cam.azimuth=135   # fixed camera
        r.update_scene(d,cam); px=r.render()
        im=Image.fromarray(px); dr=ImageDraw.Draw(im)
        dr.rectangle([0,CELL_H-22,CELL_W,CELL_H],fill=(0,0,0))
        dr.text((6,CELL_H-20),f"{label}  ({m.nu} act)",font=FL,fill=(150,230,180))
        frames.append(im)
    r.close()
    return frames

def main():
    out=sys.argv[1]; robots=sys.argv[2].split(","); cols=int(sys.argv[3]) if len(sys.argv)>3 else 3
    per={}
    for name in robots:
        p=pick(os.path.join(ROOT,name))
        print("driving",name,flush=True)
        per[name]=drive_frames(p,name.replace("_"," "))
    rows=(len(robots)+cols-1)//cols
    W,H=cols*CELL_W, rows*CELL_H
    gif=[]
    for f in range(FRAMES):
        canvas=Image.new("RGB",(W,H),(10,11,14))
        for i,name in enumerate(robots):
            canvas.paste(per[name][f],((i%cols)*CELL_W,(i//cols)*CELL_H))
        gif.append(canvas)
    gif[0].save(out,save_all=True,append_images=gif[1:],duration=70,loop=0,optimize=True)
    print("WROTE",out,os.path.getsize(out)//1024,"KB",W,"x",H)

main()
