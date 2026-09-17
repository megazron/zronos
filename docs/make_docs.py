#!/usr/bin/env python3
"""Regenerate docs/img/ for ChalaoOS: an architecture diagram (SVG) and an
animated boot-demo terminal GIF (from the real ./chalaoos --demo output)."""
import os, subprocess, html
HERE=os.path.dirname(os.path.abspath(__file__)); IMG=os.path.join(HERE,"img"); os.makedirs(IMG,exist_ok=True)
ROOT=os.path.dirname(HERE)
INK="#e6edf3"; MUTE="#8b949e"; BG="#0d1117"; CARD="#161b22"; LINE="#30363d"
ACC="#3fb950"; ACC2="#58a6ff"; WARN="#d29922"; FONT="-apple-system,BlinkMacSystemFont,'Segoe UI',Helvetica,Arial,sans-serif"
def esc(s): return html.escape(str(s))

def architecture():
    W,H=920,600; b=[f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}" font-family="{FONT}">']
    b.append(f'<rect width="{W}" height="{H}" fill="{BG}"/>')
    b.append(f'<text x="30" y="40" font-size="22" font-weight="700" fill="{INK}">ChalaoOS architecture</text>')
    b.append(f'<text x="30" y="62" font-size="13" fill="{MUTE}">A robot operating environment. Every service is a Robot Chalao (.rc) program.</text>')
    def layer(y,h,title,sub,items,col):
        b.append(f'<rect x="30" y="{y}" width="{W-60}" height="{h}" rx="10" fill="{CARD}" stroke="{col}" stroke-width="1.6"/>')
        b.append(f'<text x="48" y="{y+26}" font-size="15" font-weight="700" fill="{col}">{esc(title)}</text>')
        b.append(f'<text x="48" y="{y+44}" font-size="11.5" fill="{MUTE}">{esc(sub)}</text>')
        x=48
        for it in items:
            wpx=14+len(it)*7.4
            b.append(f'<rect x="{x}" y="{y+h-40}" width="{wpx:.0f}" height="26" rx="5" fill="#0d1117" stroke="{LINE}"/>')
            b.append(f'<text x="{x+wpx/2:.0f}" y="{y+h-22}" font-size="11.5" fill="{INK}" text-anchor="middle" font-family="monospace">{esc(it)}</text>')
            x+=wpx+10
    layer(84,86,"Services (Robot Chalao .rc)","what the robot does -- one .rc per service, hot-loaded from the manifest",
          ["bringup","heartbeat","perception","drive","watchdog"],ACC)
    layer(186,86,"Shell + IPC blackboard","operate + talk between services",
          ["ps","start/stop","log","topics","echo","bolo -> topic -> sun"],ACC2)
    layer(288,86,"Kernel: init - supervisor - scheduler","boot from manifest, dependency order, restart policy, virtual-clock ticks",
          ["init","supervisor","restart on-failure/always","scheduler @rate_hz"],WARN)
    layer(390,86,"HAL (hardware abstraction) + e-stop","devices behind one interface; band karo cuts actuators",
          ["battery","wheels","imu","camera","gripper","E-STOP"],"#f778ba")
    layer(492,86,"Robot Chalao native core (C++17)","the compiled language the whole OS is built on",
          ["lexer","parser","interpreter"],MUTE)
    # arrows down
    for y in (170,272,374,476): b.append(f'<path d="M{W/2} {y} l0 14 m-5 -6 l5 6 l5 -6" stroke="{MUTE}" stroke-width="1.4" fill="none"/>')
    b.append("</svg>"); open(os.path.join(IMG,"architecture.svg"),"w").write("\n".join(b)); print("wrote architecture.svg")

def boot_gif():
    try:
        from PIL import Image, ImageDraw, ImageFont
    except Exception as e:
        print("PIL missing, skip boot.gif:", e); return
    # build the binary and capture the real demo
    subprocess.run(["make"], cwd=ROOT, capture_output=True)
    out=subprocess.run([os.path.join(ROOT,"chalaoos"),"--demo","--seconds","8"], cwd=ROOT, capture_output=True, text=True).stdout
    lines=[l for l in out.splitlines()]
    def font(sz,bold=False):
        p="/usr/share/fonts/truetype/dejavu/DejaVuSansMono%s.ttf"%("-Bold" if bold else "")
        try: return ImageFont.truetype(p,sz)
        except Exception: return ImageFont.load_default()
    F=font(13); FB=font(13,True)
    W,H=860,470; VIS=28; pad=14; lh=15
    def colour(l):
        if "E-STOP" in l or "BLOCKED" in l: return (235,110,120)
        if "watchdog" in l: return (240,200,120)
        if "init" in l or "boot" in l or "RUNNING" in l: return (120,210,150)
        if "[nakli]" in l or "kernel" in l: return (140,190,240)
        return (205,214,224)
    frames=[]; shown=[]
    def render():
        im=Image.new("RGB",(W,H),(9,11,15)); d=ImageDraw.Draw(im)
        d.rectangle([0,0,W,26],fill=(30,36,46))
        for i,c in enumerate([(255,95,86),(255,189,46),(39,201,63)]): d.ellipse([12+i*16,8,22+i*16,18],fill=c)
        d.text((W/2-70,6),"chalaoos --demo",font=F,fill=(150,160,170))
        vis=shown[-VIS:]; y=34
        for l in vis:
            d.text((pad,y),l[:120],font=(FB if ("E-STOP" in l or "boot complete" in l) else F),fill=colour(l)); y+=lh
        return im
    step=max(1,len(lines)//60)
    for i,l in enumerate(lines):
        shown.append(l)
        if i%1==0: frames.append(render())
    for _ in range(14): frames.append(frames[-1])
    # keep it light: every other frame
    frames=frames[::1]
    frames[0].save(os.path.join(IMG,"boot.gif"),save_all=True,append_images=frames[1:],duration=90,loop=0,optimize=True)
    print("wrote boot.gif",os.path.getsize(os.path.join(IMG,"boot.gif"))//1024,"KB",len(frames),"frames")

if __name__=="__main__":
    architecture(); boot_gif()
