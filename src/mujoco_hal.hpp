// zronOS <-> MuJoCo simulation bridge client.
//
// This backend drives a robot model in MuJoCo physics.
// It launches the Python MuJoCo bridge (zron_mujoco.py) as a child process,
// talks to it over a line-oriented JSON protocol on two pipes, enumerates the
// robot's actuators, then runs a closed control loop: every cycle zronOS
// computes a command for each actuator, sends it to physics, reads the state
// back, and terminates the bridge run if state validation or the velocity
// threshold fails. `zron mujoco <model.xml>` prints a one-line
// JSON verdict used by the compatibility harness.
//
// No external C++ dependencies: raw fork/exec/pipe + a tiny hand JSON reader
// for the handful of fields the protocol uses.
#pragma once
#include <string>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <unistd.h>
#include <sys/wait.h>
#include <sstream>
#include <iostream>
#include <limits>
#include <cctype>
#include <csignal>

namespace zmj {

// ---- minimal JSON field readers (protocol is flat + numeric) ----
static bool jbool(const std::string& s, const char* key, bool defv=false){
    std::string k=std::string("\"")+key+"\"";
    auto p=s.find(k); if(p==std::string::npos) return defv;
    p=s.find(':',p); if(p==std::string::npos) return defv;
    while(p+1<s.size() && (s[p]==':'||s[p]==' ')) ++p;
    return s.compare(p,4,"true")==0;
}
static double jnum(const std::string& s, const char* key, double defv=0){
    std::string k=std::string("\"")+key+"\"";
    auto p=s.find(k); if(p==std::string::npos) return defv;
    p=s.find(':',p); if(p==std::string::npos) return defv;
    const char* start=s.c_str()+p+1;
    char* end=nullptr;
    double value=std::strtod(start,&end);
    if(end==start || !std::isfinite(value)) return defv;
    while(*end && std::isspace(static_cast<unsigned char>(*end))) ++end;
    if(*end!=',' && *end!='}') return defv;
    return value;
}

static std::string jsonString(const std::string& value){
    std::string out="\"";
    const char* hex="0123456789abcdef";
    for(unsigned char c:value){
        if(c=='"' || c=='\\'){ out+='\\'; out+=c; }
        else if(c<32){ out+="\\u00"; out+=hex[c>>4]; out+=hex[c&15]; }
        else out+=c;
    }
    return out+'"';
}

// A live connection to a MuJoCo model, driven by zronOS.
struct Bridge {
    pid_t pid=-1; int toChild=-1, fromChild=-1; FILE* in=nullptr;
    std::string lastLine;

    bool start(const std::string& bridgeCmd, const std::string& model){
        int a[2], b[2];                       // a: parent->child, b: child->parent
        if(pipe(a)||pipe(b)) return false;
        pid=fork();
        if(pid<0) return false;
        if(pid==0){                           // child: exec the python bridge
            dup2(a[0],0); dup2(b[1],1);
            close(a[0]);close(a[1]);close(b[0]);close(b[1]);
            // The model is an opaque positional argument, never shell code.
            std::string full=bridgeCmd+" \"$1\"";
            execl("/bin/sh","sh","-c",full.c_str(),"zron-bridge",model.c_str(),(char*)nullptr);
            _exit(127);
        }
        close(a[0]); close(b[1]);
        toChild=a[1]; fromChild=b[0];
        in=fdopen(fromChild,"r");
        return in!=nullptr;
    }
    void send(const std::string& line){ std::string l=line+"\n"; ssize_t r=write(toChild,l.data(),l.size()); (void)r; }
    bool recv(){
        char buf[65536];
        if(!fgets(buf,sizeof(buf),in)) return false;
        lastLine=buf; return true;
    }
    void stop(){
        if(toChild>=0){ send("{\"cmd\":\"quit\"}"); }
        if(in){ fclose(in); in=nullptr; fromChild=-1; }
        if(toChild>=0){ close(toChild); toChild=-1; }
        if(pid>0){ int st; waitpid(pid,&st,0); pid=-1; }
    }
};

struct Verdict {
    bool ok=false, stable=false;
    int nu=0, nq=0, cycles=0;
    double qvel_max=0, t=0;
    std::string model, err;
    std::string json() const {
        std::ostringstream o; o.setf(std::ios::fixed); o.precision(4);
        o<<"{\"ok\":"<<(ok?"true":"false")
         <<",\"stable\":"<<(stable?"true":"false")
         <<",\"nu\":"<<nu<<",\"nq\":"<<nq<<",\"cycles\":"<<cycles
         <<",\"qvel_max\":"<<qvel_max<<",\"t\":"<<t;
        if(!model.empty()) o<<",\"model\":"<<jsonString(model);
        if(!err.empty())   o<<",\"err\":"<<jsonString(err);
        o<<"}"; return o.str();
    }
};

// Drive `model` for `cycles` control cycles (each = `substeps` physics steps).
// zronOS is the controller: a bounded actuator sweep with a stability governor.
inline Verdict compat(const std::string& bridgeCmd, const std::string& model,
                      int cycles=200, int substeps=5, double amp=0.15){
    Verdict v; v.cycles=0;
    Bridge br;
    if(!br.start(bridgeCmd, model)){ v.err="bridge spawn failed"; return v; }
    // 1) startup handshake: the bridge prints {"ok":true,"loaded":...} or {"ok":false,"error":...}
    if(!br.recv()){ v.err="no handshake"; br.stop(); return v; }
    std::string hs=br.lastLine;
    if(!jbool(hs,"ok",false)){
        v.err="load failed";
        auto ep=hs.find("\"error\"");
        if(ep!=std::string::npos){ auto q=hs.find('\"',ep+7); auto q2=(q==std::string::npos)?std::string::npos:hs.find('\"',q+1);
            if(q!=std::string::npos && q2!=std::string::npos) v.err=hs.substr(q+1, q2-q-1); }
        br.stop(); return v;
    }
    // 2) model info
    br.send("{\"cmd\":\"info\"}");
    if(!br.recv()){ v.err="no info reply"; br.stop(); return v; }
    std::string info=br.lastLine;
    const double invalid=std::numeric_limits<double>::quiet_NaN();
    double nu=jnum(info,"nu",invalid), nq=jnum(info,"nq",invalid);
    if(!jbool(info,"ok") || !std::isfinite(nu) || !std::isfinite(nq) ||
       nu<0 || nq<0 || nu>100000 || nq>100000 || std::floor(nu)!=nu || std::floor(nq)!=nq){
        v.err="invalid model info"; br.stop(); return v;
    }
    v.nu=(int)nu; v.nq=(int)nq;
    v.model=model.substr(model.find_last_of('/')+1);
    for(int c=0; c<cycles; ++c){
        // Open-loop multi-phase sinusoid per actuator, bounded in amplitude.
        // A governor trip terminates the bridge run.
        std::ostringstream cmd; cmd<<"{\"cmd\":\"step\",\"n\":"<<substeps<<",\"ctrl\":[";
        for(int j=0;j<v.nu;++j){
            double u = amp*std::sin(0.03*c + 0.6*j);
            cmd<<(j?",":"")<<u;
        }
        cmd<<"]}";
        br.send(cmd.str());
        if(!br.recv()){ v.err="bridge died mid-run"; break; }
        std::string rep=br.lastLine;
        double qv=jnum(rep,"qvel_max",invalid), t=jnum(rep,"time",invalid);
        if(!jbool(rep,"ok") || !jbool(rep,"finite") || !jbool(rep,"stable") ||
           !std::isfinite(qv) || qv<0 || !std::isfinite(t) || t<=v.t){
            v.err="bridge step rejected or invalid physics state"; break;
        }
        v.qvel_max=qv; v.t=t; v.cycles=c+1;
        // Reject excessive velocity and terminate this simulation run.
        if(qv>1e3){
            v.err="velocity limit exceeded";
            break;
        }
    }
    v.ok = v.err.empty() && v.cycles==cycles && cycles>0;
    v.stable = v.ok;
    br.stop();
    return v;
}

// Default bridge command: $ZRON_BRIDGE, else zron_mujoco.py found next to the
// binary (source tree: ./bridge/, installed: ../share/zronos/bridge/).
// Directory holding the running zron binary ("." if unknown).
inline std::string exeDir(){
    char buf[4096]; ssize_t n=readlink("/proc/self/exe", buf, sizeof buf - 1);
    if(n<=0) return ".";
    buf[n]=0; std::string exe(buf); auto sl=exe.find_last_of('/');
    return sl==std::string::npos ? "." : exe.substr(0,sl);
}

inline std::string defaultBridge(){
    if(const char* env=std::getenv("ZRON_BRIDGE")) if(*env) return env;
    std::string dir=exeDir();
    for(const std::string& rel : {std::string("/bridge/zron_mujoco.py"), std::string("/../share/zronos/bridge/zron_mujoco.py")}){
        std::string p=dir+rel;
        if(access(p.c_str(), R_OK)==0) return "python3 '"+p+"'";
    }
    return "python3 bridge/zron_mujoco.py";
}

// Entry point for `zron mujoco <model.xml> [--bridge CMD] [--cycles N]`.
inline int mujocoCompat(const std::vector<std::string>& a){
    if(a.size()<2){ std::cerr<<"usage: zron mujoco <model.xml> [--bridge \"py bridge.py\"] [--cycles N]\n"; return 2; }
    std::string model=a[1];
    std::string bridgeCmd = defaultBridge();
    int cycles=200;
    for(size_t i=2;i<a.size();++i){
        if(a[i]=="--bridge" && i+1<a.size()) bridgeCmd=a[++i];
        else if(a[i]=="--cycles" && i+1<a.size()) cycles=std::atoi(a[++i].c_str());
    }
    // Report a broken bridge as a failed run, not process death via SIGPIPE.
    std::signal(SIGPIPE, SIG_IGN);
    Verdict v=zmj::compat(bridgeCmd, model, cycles);
    std::cout<<v.json()<<"\n";
    return (v.ok && v.stable)?0:1;
}

} // namespace zmj
