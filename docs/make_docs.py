#!/usr/bin/env python3
"""Regenerate docs/img/ for zronOS: an architecture diagram (SVG), an animated
boot-demo terminal GIF (from the real ./zron demo output), and a MuJoCo
compatibility chart (SVG, from docs/compat.json)."""
import os, subprocess, html, json
HERE=os.path.dirname(os.path.abspath(__file__)); IMG=os.path.join(HERE,"img"); os.makedirs(IMG,exist_ok=True)
ROOT=os.path.dirname(HERE)
INK="#e6edf3"; MUTE="#8b949e"; BG="#0d1117"; CARD="#161b22"; LINE="#30363d"
ACC="#3fb950"; ACC2="#58a6ff"; WARN="#d29922"; PINK="#f778ba"; FONT="-apple-system,BlinkMacSystemFont,'Segoe UI',Helvetica,Arial,sans-serif"
def esc(s): return html.escape(str(s))

def architecture():
    W,H=940,640; b=[f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}" font-family="{FONT}">']
    b.append(f'<rect width="{W}" height="{H}" fill="{BG}"/>')
    b.append(f'<text x="30" y="40" font-size="22" font-weight="700" fill="{INK}">zronOS architecture</text>')
    b.append(f'<text x="30" y="62" font-size="13" fill="{MUTE}">A robot operating environment. Every service is a Robot Chalao (.rc) program.</text>')
    def layer(y,h,title,sub,items,col):
        b.append(f'<rect x="30" y="{y}" width="{W-60}" height="{h}" rx="10" fill="{CARD}" stroke="{col}" stroke-width="1.6"/>')
        b.append(f'<text x="48" y="{y+26}" font-size="15" font-weight="700" fill="{col}">{esc(title)}</text>')
        b.append(f'<text x="48" y="{y+44}" font-size="11.5" fill="{MUTE}">{esc(sub)}</text>')
        x=48
        for it in items:
            wpx=14+len(it)*7.2
            b.append(f'<rect x="{x}" y="{y+h-40}" width="{wpx:.0f}" height="26" rx="5" fill="#0d1117" stroke="{LINE}"/>')
            b.append(f'<text x="{x+wpx/2:.0f}" y="{y+h-22}" font-size="11.5" fill="{INK}" text-anchor="middle" font-family="monospace">{esc(it)}</text>')
            x+=wpx+10
    layer(84,86,"Services (Robot Chalao .rc)","what the robot does -- one .rc per service, booted from the manifest",
          ["bringup","heartbeat","perception","drive","watchdog"],ACC)
    layer(186,90,"Runtime services","the middleware a robot needs",
          ["param server","TF tree","IPC blackboard","RPC (sewa)","record / replay","health / doctor"],ACC2)
    layer(292,86,"Kernel: init - supervisor - scheduler","boot from manifest, dependency order, restart policy, liveness watchdog",
          ["init","supervisor","restart on-failure/always","scheduler @rate_hz"],WARN)
    layer(394,86,"HAL (hardware abstraction) + e-stop","devices behind one interface; band karo cuts actuators; fault injection",
          ["battery","wheels","imu","camera","gripper","E-STOP","MuJoCo bridge"],PINK)
    layer(496,86,"Robot Chalao native core (C++17)","the compiled language the whole OS is built on",
          ["lexer","parser","interpreter","callFunction (RPC)"],MUTE)
    for y in (170,276,378,480,582): b.append(f'<path d="M{W/2} {y} l0 12 m-5 -5 l5 5 l5 -5" stroke="{MUTE}" stroke-width="1.4" fill="none"/>')
    b.append("</svg>"); open(os.path.join(IMG,"architecture.svg"),"w").write("\n".join(b)); print("wrote architecture.svg")

def compat_svg():
    jp=os.path.join(HERE,"compat.json")
    if not os.path.exists(jp): print("no compat.json, skip compat.svg"); return
    rows=json.load(open(jp))
    npass=sum(1 for r in rows if r["pass"]); total=len(rows)
    cats={}
    for r in rows:
        c=r["category"]; cats.setdefault(c,[0,0]); cats[c][1]+=1
        if r["pass"]: cats[c][0]+=1
    order=sorted(cats, key=lambda c:-cats[c][1])
    W,H=880,120+len(order)*38
    b=[f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}" font-family="{FONT}">']
    b.append(f'<rect width="{W}" height="{H}" fill="{BG}"/>')
    b.append(f'<text x="30" y="42" font-size="21" font-weight="700" fill="{INK}">zronOS × MuJoCo compatibility</text>')
    b.append(f'<text x="{W-30}" y="42" font-size="21" font-weight="800" fill="{ACC}" text-anchor="end">{npass} / {total} models</text>')
    b.append(f'<text x="30" y="64" font-size="12.5" fill="{MUTE}">Every open-source MuJoCo Menagerie robot: loaded in real physics, driven by an auto-generated Robot Chalao service under zron, stepped 200x.</text>')
    y=92; barx=200; barw=W-260
    for c in order:
        p,t=cats[c]; frac=p/t if t else 0
        b.append(f'<text x="30" y="{y+15}" font-size="13" fill="{INK}">{esc(c)}</text>')
        b.append(f'<rect x="{barx}" y="{y}" width="{barw}" height="20" rx="4" fill="#0d1117" stroke="{LINE}"/>')
        b.append(f'<rect x="{barx}" y="{y}" width="{barw*frac:.0f}" height="20" rx="4" fill="{ACC if p==t else WARN}"/>')
        b.append(f'<text x="{barx+barw+8}" y="{y+15}" font-size="12.5" fill="{MUTE}">{p}/{t}</text>')
        y+=38
    b.append("</svg>"); open(os.path.join(IMG,"compat.svg"),"w").write("\n".join(b)); print("wrote compat.svg")

def boot_gif():
    try:
        from PIL import Image, ImageDraw, ImageFont
    except Exception as e:
        print("PIL missing, skip boot.gif:", e); return
    subprocess.run(["make"], cwd=ROOT, capture_output=True)
    out=subprocess.run([os.path.join(ROOT,"zron"),"demo","--seconds","8"], cwd=ROOT, capture_output=True, text=True).stdout
    lines=[l for l in out.splitlines()]
    def font(sz,bold=False):
        p="/usr/share/fonts/truetype/dejavu/DejaVuSansMono%s.ttf"%("-Bold" if bold else "")
        try: return ImageFont.truetype(p,sz)
        except Exception: return ImageFont.load_default()
    F=font(13); FB=font(13,True)
    W,H=880,470; VIS=28; pad=14; lh=15
    def colour(l):
        if "E-STOP" in l or "BLOCKED" in l or "FAIL" in l: return (235,110,120)
        if "watchdog" in l or "WARN" in l: return (240,200,120)
        if "init" in l or "boot" in l or "RUNNING" in l or "OK" in l: return (120,210,150)
        if "[nakli]" in l or "kernel" in l or "[tf]" in l or "[rpc]" in l or "[param]" in l: return (140,190,240)
        return (205,214,224)
    frames=[]; shown=[]
    def render():
        im=Image.new("RGB",(W,H),(9,11,15)); d=ImageDraw.Draw(im)
        d.rectangle([0,0,W,26],fill=(30,36,46))
        for i,c in enumerate([(255,95,86),(255,189,46),(39,201,63)]): d.ellipse([12+i*16,8,22+i*16,18],fill=c)
        d.text((W/2-56,6),"zron demo",font=F,fill=(150,160,170))
        vis=shown[-VIS:]; y=34
        for l in vis:
            d.text((pad,y),l[:120],font=(FB if ("E-STOP" in l or "boot complete" in l) else F),fill=colour(l)); y+=lh
        return im
    for i,l in enumerate(lines):
        shown.append(l); frames.append(render())
    for _ in range(14): frames.append(frames[-1])
    frames[0].save(os.path.join(IMG,"boot.gif"),save_all=True,append_images=frames[1:],duration=90,loop=0,optimize=True)
    print("wrote boot.gif",os.path.getsize(os.path.join(IMG,"boot.gif"))//1024,"KB",len(frames),"frames")

if __name__=="__main__":
    architecture(); compat_svg(); boot_gif()
