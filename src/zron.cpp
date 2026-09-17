// zronOS -- a robot operating environment whose services are Robot Chalao
// (.rc) programs. It boots a robot from one manifest and gives it the runtime
// a robot actually needs: init + supervisor + scheduler, a parameter server,
// a TF transform tree, an IPC blackboard, inter-service RPC, per-service
// health/diagnostics, bus record & replay, a HAL with an e-stop and fault
// injection, and a shell. Built on the Robot Chalao native C++ core.
//
// It is NOT a bare-metal kernel; it operates at the runtime layer (like "ROS"),
// on top of Linux. One binary, zero dependencies, deterministic.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <deque>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>
#include "value.hpp"
#include "ast.hpp"
#include "lexer.hpp"
#include "parser.hpp"
#include "interp.hpp"
#include "mujoco_hal.hpp"

static const char* OS_VERSION = "1.0.0";

static std::string ts(double t){ char b[32]; std::snprintf(b,sizeof b,"%6.2fs", t); return b; }
static std::string f2(double d){ char b[32]; std::snprintf(b,sizeof b,"%.1f", d); return b; }
static std::string trim(std::string s){ size_t a=s.find_first_not_of(" \t\r\n"); size_t b=s.find_last_not_of(" \t\r\n");
    return a==std::string::npos? std::string() : s.substr(a,b-a+1); }

// ============================================================ 3D transforms
// A tiny 4x4 homogeneous transform, row-major. Enough for a correct TF tree.
struct M4 {
    double m[16];
    static M4 identity(){ M4 r{}; for(int i=0;i<16;++i) r.m[i]=0; r.m[0]=r.m[5]=r.m[10]=r.m[15]=1; return r; }
    static M4 fromXYZRPY(double x,double y,double z,double R,double P,double Y){
        double cr=std::cos(R),sr=std::sin(R),cp=std::cos(P),sp=std::sin(P),cy=std::cos(Y),sy=std::sin(Y);
        M4 t = identity();
        // ZYX (yaw*pitch*roll)
        t.m[0]=cy*cp;            t.m[1]=cy*sp*sr - sy*cr; t.m[2]=cy*sp*cr + sy*sr; t.m[3]=x;
        t.m[4]=sy*cp;            t.m[5]=sy*sp*sr + cy*cr; t.m[6]=sy*sp*cr - cy*sr; t.m[7]=y;
        t.m[8]=-sp;              t.m[9]=cp*sr;            t.m[10]=cp*cr;           t.m[11]=z;
        t.m[12]=0; t.m[13]=0; t.m[14]=0; t.m[15]=1;
        return t;
    }
    M4 mul(const M4& o) const {
        M4 r{}; for(int i=0;i<16;++i) r.m[i]=0;
        for(int i=0;i<4;++i) for(int j=0;j<4;++j){ double s=0; for(int k=0;k<4;++k) s+=m[i*4+k]*o.m[k*4+j]; r.m[i*4+j]=s; }
        return r;
    }
    M4 invRigid() const {
        // inverse of a rigid transform: R^T, -R^T t
        M4 r = identity();
        for(int i=0;i<3;++i) for(int j=0;j<3;++j) r.m[i*4+j]=m[j*4+i];
        double tx=m[3],ty=m[7],tz=m[11];
        r.m[3]  = -(r.m[0]*tx + r.m[1]*ty + r.m[2]*tz);
        r.m[7]  = -(r.m[4]*tx + r.m[5]*ty + r.m[6]*tz);
        r.m[11] = -(r.m[8]*tx + r.m[9]*ty + r.m[10]*tz);
        return r;
    }
    Value toPose() const {
        double yaw   = std::atan2(m[4], m[0]);
        double pitch = std::atan2(-m[8], std::sqrt(m[9]*m[9] + m[10]*m[10]));
        double roll  = std::atan2(m[9], m[10]);
        return makePose(m[3], m[7], m[11], roll, pitch, yaw);
    }
};

// ------------------------------------------------------------------ IPC bus
struct BusRec { double t; std::string topic, val; };
struct Bus {
    std::map<std::string,std::string> latest;   // topic -> last value (as text)
    std::map<std::string,long> count;
    std::vector<BusRec> tape;                    // recording buffer
    bool recording = false;
    void publish(const std::string& topic, const std::string& val, double clock){
        latest[topic]=val; count[topic]++;
        if(recording) tape.push_back({clock, topic, val});
    }
};

// ----------------------------------------------------------- parameter server
struct Params {
    std::map<std::string,Value> kv;
    void set(const std::string& k, const Value& v){ kv[k]=v; }
    bool get(const std::string& k, Value& out){ auto it=kv.find(k); if(it==kv.end()) return false; out=it->second; return true; }
};

// ----------------------------------------------------------------- TF tree
struct TfTree {
    struct Frame { std::string parent; M4 local; };
    std::map<std::string,Frame> frames;
    void set(const std::string& child, const std::string& parent, M4 local){ frames[child]={parent,local}; }
    bool known(const std::string& name) const {
        if(frames.count(name)) return true;
        for(auto& kv:frames) if(kv.second.parent==name) return true;   // implicit root
        return false;
    }
    // world transform of a frame: chase parents to the root, composing.
    bool worldOf(const std::string& name, M4& out, std::string& err){
        if(!known(name)){ err="unknown frame '"+name+"'"; return false; }
        std::vector<std::string> chain; std::set<std::string> seen; std::string f=name;
        while(!f.empty()){
            if(seen.count(f)){ err="TF cycle at frame '"+f+"'"; return false; }
            seen.insert(f);
            auto it=frames.find(f);
            if(it==frames.end()) break;   // implicit root -> identity contribution
            chain.push_back(f); f=it->second.parent;
        }
        M4 acc = M4::identity();
        for(auto rit=chain.rbegin(); rit!=chain.rend(); ++rit) acc = acc.mul(frames[*rit].local);
        out=acc; return true;
    }
    // pose of `from` expressed in `to`.
    bool lookup(const std::string& from, const std::string& to, Value& pose, std::string& err){
        M4 wf, wt;
        if(!worldOf(from,wf,err)) return false;
        if(!worldOf(to,wt,err)) return false;
        pose = wt.invRigid().mul(wf).toPose(); return true;
    }
};

// -------------------------------------------------------------- HAL (devices)
struct HAL {
    double battery = 100.0;      // percent, drains over time
    bool   estopped = false;
    bool   obstacle = false;
    double odom = 0.0;           // metres driven
    double drain_per_s = 4.0;
    std::string fault;           // injected actuator fault ("" = healthy)
    void tick(double dt){ if(!estopped) battery -= drain_per_s*dt; if(battery<0) battery=0; }
    bool healthy() const { return fault.empty() && !estopped; }
    bool drive(double dist){ if(estopped||!fault.empty()) return false; odom += dist; return true; }
    void estop(){ estopped = true; }
};

// ------------------------------------------------------------------ services
enum class St { STARTING, RUNNING, STOPPED, FAILED, RESTARTING };
enum class Rp { NEVER, ON_FAILURE, ALWAYS };
enum class Hz { OK, WARN, ERR };
static const char* stName(St s){ switch(s){case St::STARTING:return "STARTING";case St::RUNNING:return "RUNNING";
    case St::STOPPED:return "STOPPED";case St::FAILED:return "FAILED";default:return "RESTARTING";} }
static const char* rpName(Rp r){ return r==Rp::NEVER?"never":r==Rp::ON_FAILURE?"on-failure":"always"; }
static const char* hzName(Hz h){ return h==Hz::OK?"OK":h==Hz::WARN?"WARN":"ERROR"; }

struct Service {
    std::string name, script;
    Rp restart = Rp::ON_FAILURE;
    std::vector<std::string> after;
    double rate_hz = 0.0;        // 0 = run once at boot
    bool autostart = true;
    // runtime
    St state = St::STOPPED;
    int restarts = 0;
    double startedAt = 0.0, nextRun = 0.0, retryAt = -1.0, lastRun = -1.0;
    long runs = 0;
    NodePtr prog;
    std::unique_ptr<Interpreter> interp;
    std::deque<std::string> log;
    bool parsed = false;
    std::string parseErr;
    // health / diagnostics
    Hz health = Hz::OK; std::string healthMsg;
};

// ------------------------------------------------------------------ the OS
class ZronOS {
    double clock = 0.0;
    const double DT = 0.5;
    HAL hal;
    Bus bus;
    Params params;
    TfTree tf;
    std::vector<std::unique_ptr<Service>> svcs;
    std::map<std::string,std::pair<std::string,std::string>> registry; // svc name -> (provider service, fn)
    bool estopLogged = false;
public:
    Service* find(const std::string& n){ for(auto& s:svcs) if(s->name==n) return s.get(); return nullptr; }

    void kmsg(const std::string& src, const std::string& msg){
        std::cout << "[" << ts(clock) << "] " << src;
        for (int i=(int)src.size(); i<10; ++i) std::cout << ' ';
        std::cout << " | " << msg << "\n";
    }

    // ---- manifest: [service NAME] blocks of key = value ----
    bool loadManifest(const std::string& path, const std::string& baseDir){
        std::ifstream f(path);
        if(!f){ std::cerr << "Bhai, manifest nahi mila: " << path << "\n"; return false; }
        std::string line; Service* cur=nullptr;
        while(std::getline(f,line)){
            std::string t=trim(line);
            if(t.empty()||t[0]=='#') continue;
            if(t.front()=='['&&t.back()==']'){
                std::string head=trim(t.substr(1,t.size()-2));
                std::string nm = head.rfind("service ",0)==0 ? trim(head.substr(8)) : head;
                auto s=std::make_unique<Service>(); s->name=nm; cur=s.get(); svcs.push_back(std::move(s));
                continue;
            }
            size_t eq=t.find('=');
            if(eq==std::string::npos||!cur) continue;
            std::string k=trim(t.substr(0,eq)), v=trim(t.substr(eq+1));
            if(k=="script")   cur->script = baseDir.empty()? v : baseDir+"/"+v;
            else if(k=="rate_hz")  cur->rate_hz = std::atof(v.c_str());
            else if(k=="restart")  cur->restart = v=="never"?Rp::NEVER : v=="always"?Rp::ALWAYS : Rp::ON_FAILURE;
            else if(k=="autostart")cur->autostart = (v=="true"||v=="1"||v=="yes");
            else if(k=="after"){ std::stringstream ss(v); std::string d; while(std::getline(ss,d,',')) { d=trim(d); if(!d.empty()) cur->after.push_back(d); } }
        }
        return !svcs.empty();
    }

    // ---- wire a fresh interpreter for a service (output sink + robot hook) ----
    void wire(Service* s){
        s->interp = std::make_unique<Interpreter>(true);
        { auto sl = s->script.find_last_of('/'); s->interp->baseDir = (sl==std::string::npos)? std::string(".") : s->script.substr(0,sl); }
        s->interp->outSink = [this,s](const std::string& out){
            s->log.push_back(out); if(s->log.size()>60) s->log.pop_front();
            kmsg(s->name, out);
        };
        s->interp->robotHook = [this,s](const std::string& m, std::map<std::string,Value>& A, Value& outv)->bool{
            auto num=[&](const char* k){ auto it=A.find(k); return it!=A.end()?it->second.num:0.0; };
            auto sv =[&](const char* k){ auto it=A.find(k); return it!=A.end()? (it->second.t==VT::Str?it->second.str:fmtValue(it->second)) : std::string(); };
            auto vv =[&](const char* k){ auto it=A.find(k); return it!=A.end()?it->second:Value::Nil(); };
            // ---- sensing / HAL ----
            if(m=="battery"){ outv=Value::Num_(hal.battery); return true; }
            if(m=="obstacle_near"){ outv=Value::Bool_(hal.obstacle); return true; }
            if(m=="base_move"){
                if(hal.drive(num("distance"))) kmsg(s->name, "[hal] wheels drove "+f2(num("distance"))+" m (odom "+f2(hal.odom)+")");
                else kmsg(s->name, std::string("[hal] BLOCKED -- ")+(hal.estopped?"actuators are in e-stop":("fault '"+hal.fault+"'")));
                outv=Value::Nil(); return true;
            }
            if(m=="base_rotate"){ if(!hal.healthy()) kmsg(s->name,"[hal] BLOCKED -- actuators offline"); outv=Value::Nil(); return true; }
            if(m=="estop"){
                bool was=hal.estopped; hal.estop();
                if(!was){ kmsg("kernel","*** E-STOP asserted by service '"+s->name+"' -- actuators cut ***"); estopLogged=true; }
                outv=Value::Nil(); return true;
            }
            // ---- IPC bus ----
            if(m=="publish"){
                bus.publish(sv("topic"), fmtValue(vv("value")), clock); outv=Value::Nil(); return true;
            }
            // ---- parameter server ----
            if(m=="param_set"){ params.set(sv("key"), vv("value")); kmsg(s->name,"[param] "+sv("key")+" = "+reprValue(vv("value"))); outv=Value::Nil(); return true; }
            if(m=="param_get"){ Value g; if(params.get(sv("key"),g)){ outv=g; } else outv=Value::Nil(); return true; }
            // ---- TF tree ----
            if(m=="tf_set"){
                tf.set(sv("child"), sv("parent"), M4::fromXYZRPY(num("x"),num("y"),num("z"),num("roll"),num("pitch"),num("yaw")));
                kmsg(s->name,"[tf] "+sv("parent")+" -> "+sv("child")+" set"); outv=Value::Nil(); return true;
            }
            if(m=="tf"){
                Value pose; std::string err;
                if(tf.lookup(sv("from_frame"), sv("to_frame"), pose, err)){ outv=pose; return true; }
                kmsg(s->name,"[tf] lookup failed: "+err); outv=makePose(0,0,0,0,0,0); return true;
            }
            // ---- inter-service RPC ----
            if(m=="service_advertise"){
                registry[sv("name")] = {s->name, sv("fn")};
                kmsg(s->name,"[rpc] advertised service '"+sv("name")+"' -> "+sv("fn")+"()"); outv=Value::Nil(); return true;
            }
            if(m=="call_service"){
                auto it=registry.find(sv("name"));
                Value d=Value::Dict_();
                if(it==registry.end()){ kmsg(s->name,"[rpc] no such service '"+sv("name")+"'"); d.dict->push_back({"success",Value::Bool_(false)}); outv=d; return true; }
                Service* prov=find(it->second.first);
                if(!prov||!prov->interp){ d.dict->push_back({"success",Value::Bool_(false)}); outv=d; return true; }
                std::vector<Value> args; args.push_back(vv("args"));
                Value res;
                bool ok = prov->interp->callFunction(it->second.second, args, res);
                kmsg(s->name,"[rpc] called '"+sv("name")+"' @"+it->second.first+" -> "+fmtValue(res));
                d.dict->push_back({"success",Value::Bool_(ok)}); d.dict->push_back({"result",res}); outv=d; return true;
            }
            // ---- health / diagnostics ----
            if(m=="diag"){
                std::string lv=sv("level"), ms=sv("msg");
                s->health = (lv=="kharaab"||lv=="error"||lv=="err")?Hz::ERR : (lv=="warning"||lv=="warn")?Hz::WARN : Hz::OK;
                s->healthMsg = ms;
                kmsg(s->name,"[sehat] "+std::string(hzName(s->health))+(ms.empty()?"":(" -- "+ms))); outv=Value::Nil(); return true;
            }
            // ---- record / replay ----
            if(m=="record_start"){ bus.recording=true; kmsg(s->name,"[bag] recording -> "+sv("bag")); outv=Value::Nil(); return true; }
            if(m=="record_stop"){ bus.recording=false; kmsg(s->name,"[bag] recording stopped ("+std::to_string(bus.tape.size())+" msgs)"); outv=Value::Nil(); return true; }
            return false;   // fall through to the built-in [nakli] simulation
        };
    }

    // ---- start one service (parse if needed, run-once now) ----
    void start(Service* s){
        s->state = St::STARTING; s->startedAt = clock;
        kmsg("init", "starting '"+s->name+"'  ("+rpName(s->restart)+", "+
             (s->rate_hz>0? f2(s->rate_hz)+" Hz":"once")+")");
        if(!s->parsed){
            std::ifstream f(s->script); std::stringstream ss; ss<<f.rdbuf();
            if(!f){ s->parseErr="script nahi mila: "+s->script; s->state=St::FAILED; s->health=Hz::ERR; kmsg("init","  FAILED: "+s->parseErr); return; }
            try { s->prog = parseSource(ss.str()); s->parsed=true; }
            catch(RCError& e){ s->parseErr=e.what_hinglish(); s->state=St::FAILED; s->health=Hz::ERR; kmsg("init","  FAILED to parse: "+s->parseErr); return; }
        }
        wire(s);
        s->state = St::RUNNING; s->nextRun = clock; s->health=Hz::OK; s->healthMsg.clear();
        kmsg("init", "  '"+s->name+"' is RUNNING");
        if(s->rate_hz==0.0) runOnce(s);      // run-once service executes at start
    }

    void runOnce(Service* s){
        try { s->interp->run(s->prog); s->lastRun=clock; s->runs++; }
        catch(RCError& e){ fail(s, e.what_hinglish()); }
    }

    void fail(Service* s, const std::string& why){
        s->state = St::FAILED; s->health=Hz::ERR; s->healthMsg=why;
        kmsg("kernel", "service '"+s->name+"' FAILED: "+why);
        if(s->restart==Rp::ALWAYS || s->restart==Rp::ON_FAILURE){
            double backoff = std::min(4.0, 0.5*(s->restarts+1));
            s->state = St::RESTARTING; s->retryAt = clock+backoff; s->restarts++;
            kmsg("kernel", "  restart policy '"+std::string(rpName(s->restart))+"': retry in "+f2(backoff)+"s (#"+std::to_string(s->restarts)+")");
        }
    }

    // ---- dependency-ordered boot ----
    void boot(){
        const int W=52;
        auto boxline=[&](const std::string& in){ std::string t=in; if((int)t.size()>W-2) t=t.substr(0,W-2);
            std::cout << "  |" << t; for(int i=(int)t.size(); i<W-2; ++i) std::cout << ' '; std::cout << "|\n"; };
        std::cout << "\n  +"; for(int i=0;i<W-2;++i) std::cout<<'-'; std::cout << "+\n";
        boxline(std::string("  zronOS ")+OS_VERSION+"  --  robot operating environment");
        boxline("  services ki zubaan: Robot Chalao (.rc)");
        std::cout << "  +"; for(int i=0;i<W-2;++i) std::cout<<'-'; std::cout << "+\n\n";
        kmsg("init", "boot: "+std::to_string(svcs.size())+" service(s) in manifest");
        std::vector<Service*> started;
        auto isStarted=[&](const std::string& n){ for(auto* p:started) if(p->name==n) return true; return false; };
        size_t guard=0, limit=svcs.size()*svcs.size()+4;
        bool progress=true;
        while(started.size()<svcs.size() && progress && guard++<limit){
            progress=false;
            for(auto& s:svcs){
                if(isStarted(s.get()->name)) continue;
                bool depsOk=true;
                for(auto& d:s->after){ Service* dp=find(d); if(!dp||!isStarted(d)){ depsOk=false; break; } }
                if(!depsOk) continue;
                if(s->autostart) start(s.get()); else { s->state=St::STOPPED; kmsg("init","'"+s->name+"' held (autostart=false)"); }
                started.push_back(s.get()); progress=true;
            }
        }
        if(started.size()<svcs.size()) kmsg("kernel","WARNING: dependency cycle or missing dep -- some services not started");
        kmsg("init","boot complete -- entering scheduler\n");
    }

    // ---- liveness watchdog: a periodic service that misses its window degrades ----
    void checkLiveness(){
        for(auto& s:svcs){
            if(s->state!=St::RUNNING || s->rate_hz<=0.0) continue;
            if(s->health==Hz::ERR) continue;           // explicit ERROR already set
            double period=1.0/s->rate_hz, age=clock-s->lastRun;
            if(s->lastRun<0) continue;
            if(age > 3.0*period){ if(s->health!=Hz::WARN){ s->health=Hz::WARN; s->healthMsg="missed liveness window"; } }
            else if(s->healthMsg=="missed liveness window"){ s->health=Hz::OK; s->healthMsg.clear(); }
        }
    }

    // ---- one scheduler tick ----
    void tick(){
        clock += DT;
        hal.tick(DT);
        for(auto& s:svcs){
            if(s->state==St::RESTARTING && s->retryAt>=0 && clock>=s->retryAt){
                kmsg("kernel","restarting '"+s->name+"'"); wire(s.get()); s->state=St::RUNNING; s->health=Hz::OK; s->healthMsg.clear(); s->nextRun=clock;
            }
            if(s->state!=St::RUNNING || s->rate_hz<=0.0) continue;
            if(clock+1e-9 < s->nextRun) continue;
            s->nextRun += 1.0/s->rate_hz;
            try { s->interp->run(s->prog); s->lastRun=clock; s->runs++; }
            catch(RCError& e){ fail(s.get(), e.what_hinglish()); }
        }
        checkLiveness();
    }

    void ps(){
        std::cout << "\n  SERVICE     STATE        HEALTH  RESTARTS  RATE     RUNS   UPTIME\n";
        std::cout <<   "  -------     -----        ------  --------  ----     ----   ------\n";
        for(auto& s:svcs){
            char row[200];
            std::snprintf(row,sizeof row,"  %-10s  %-11s  %-6s  %-8d  %-7s  %-5ld  %.1fs",
                s->name.c_str(), stName(s->state), hzName(s->health), s->restarts,
                s->rate_hz>0? (f2(s->rate_hz)+"Hz").c_str():"once", s->runs,
                s->state==St::RUNNING? (clock-s->startedAt):0.0);
            std::cout << row << "\n";
        }
        std::cout << "\n";
    }

    void topics(){
        std::cout << "\n  TOPIC          LAST VALUE      MSGS\n";
        std::cout <<   "  -----          ----------      ----\n";
        for(auto& kv:bus.latest){
            char row[200]; std::snprintf(row,sizeof row,"  %-13s  %-14s  %ld",
                kv.first.c_str(), kv.second.c_str(), bus.count[kv.first]);
            std::cout << row << "\n";
        }
        std::cout << "\n";
    }

    void showParams(){
        std::cout << "\n  PARAM               VALUE\n  -----               -----\n";
        for(auto& kv:params.kv){ char row[200]; std::snprintf(row,sizeof row,"  %-18s  %s", kv.first.c_str(), reprValue(kv.second).c_str()); std::cout<<row<<"\n"; }
        std::cout << "\n";
    }

    void showTf(){
        std::cout << "\n  TF TREE (child <- parent)\n";
        for(auto& kv:tf.frames){ std::cout << "    " << (kv.second.parent.empty()?"(world)":kv.second.parent) << "  ->  " << kv.first << "\n"; }
        std::cout << "\n";
    }

    // ---- aggregate health report (the `doctor` at runtime) ----
    int health(){
        int bad=0, warn=0;
        std::cout << "\n  zron health\n  -----------\n";
        for(auto& s:svcs){
            const char* mark = s->health==Hz::OK?"[ OK ]":s->health==Hz::WARN?"[WARN]":"[FAIL]";
            if(s->health==Hz::ERR) bad++; else if(s->health==Hz::WARN) warn++;
            std::cout << "    " << mark << "  " << s->name;
            if(!s->healthMsg.empty()) std::cout << "  -- " << s->healthMsg;
            std::cout << "\n";
        }
        std::cout << "    " << (hal.healthy()?"[ OK ]":"[FAIL]") << "  HAL  -- battery "<<f2(hal.battery)<<"%, e-stop="
                  << (hal.estopped?"HAAN":"nahi") << (hal.fault.empty()?"":(", fault '"+hal.fault+"'")) << "\n";
        std::cout << "  summary: " << (bad? "UNHEALTHY" : warn? "DEGRADED" : "HEALTHY")
                  << " ("<<bad<<" failed, "<<warn<<" degraded)\n\n";
        return bad? 1 : 0;
    }

    void replay(const std::string& /*bag*/){
        if(bus.tape.empty()){ std::cout << "  (bag khaali hai -- pehle record karo)\n"; return; }
        std::cout << "  replaying " << bus.tape.size() << " msgs...\n";
        for(auto& r:bus.tape) std::cout << "    [" << ts(r.t) << "] " << r.topic << " <- " << r.val << "\n";
    }

    void halt(){
        kmsg("init","halt: stopping services");
        for(auto& s:svcs) if(s->state==St::RUNNING) s->state=St::STOPPED;
        std::cout << "\n  zronOS band. Alvida.\n";
    }

    // ---- non-interactive demo ----
    int demo(double seconds){
        boot();
        int ticks = (int)(seconds/DT);
        for(int i=0;i<ticks;++i) tick();
        kmsg("kernel","battery ab "+f2(hal.battery)+"%, e-stop="+(hal.estopped?"HAAN":"nahi"));
        ps();
        topics();
        health();
        halt();
        return 0;
    }

    // ---- interactive shell ----
    int shell(){
        boot();
        std::string line;
        std::cout << "zron> " << std::flush;
        while(std::getline(std::cin,line)){
            std::stringstream ss(line); std::string cmd,arg; ss>>cmd; ss>>arg;
            if(cmd=="help"){ std::cout <<
                "  commands:\n"
                "    ps | health | topics | params | tf | echo TOPIC\n"
                "    start N | stop N | restart N | log N\n"
                "    tick [n] | uptime | fault NAME | clear | estop\n"
                "    replay | halt | help\n"; }
            else if(cmd=="ps") ps();
            else if(cmd=="health"||cmd=="doctor") health();
            else if(cmd=="topics") topics();
            else if(cmd=="params") showParams();
            else if(cmd=="tf") showTf();
            else if(cmd=="uptime") std::cout << "  uptime "<<f2(clock)<<"s, battery "<<f2(hal.battery)<<"%\n";
            else if(cmd=="tick"){ int n=arg.empty()?1:std::atoi(arg.c_str()); for(int i=0;i<n;++i) tick(); }
            else if(cmd=="fault"){ hal.fault=arg; std::cout<<"  fault injected: '"<<arg<<"'\n"; }
            else if(cmd=="clear"){ hal.fault.clear(); std::cout<<"  faults cleared\n"; }
            else if(cmd=="estop"){ hal.estop(); std::cout<<"  e-stop asserted.\n"; }
            else if(cmd=="replay") replay(arg);
            else if(cmd=="halt"){ halt(); break; }
            else if(cmd=="start"){ Service* s=find(arg); if(s) start(s); else std::cout<<"  no such service\n"; }
            else if(cmd=="stop"){ Service* s=find(arg); if(s){ s->state=St::STOPPED; std::cout<<"  stopped\n";} else std::cout<<"  no such service\n"; }
            else if(cmd=="restart"){ Service* s=find(arg); if(s){ wire(s); s->state=St::RUNNING; s->health=Hz::OK; s->healthMsg.clear(); s->restarts++; s->nextRun=clock; std::cout<<"  restarted\n";} else std::cout<<"  no such service\n"; }
            else if(cmd=="log"){ Service* s=find(arg); if(s){ for(auto& l:s->log) std::cout<<"    "<<l<<"\n"; } else std::cout<<"  no such service\n"; }
            else if(cmd=="echo"){ auto it=bus.latest.find(arg); std::cout<<"  "<<(it!=bus.latest.end()?it->second:"(no such topic)")<<"\n"; }
            else if(cmd.empty()){}
            else std::cout << "  anjaan hukum: "<<cmd<<" (try 'help')\n";
            std::cout << "zron> " << std::flush;
        }
        return 0;
    }
};

// ===================================================== preflight (zron doctor)
// Static checks WITHOUT booting: manifest parses, scripts exist and parse,
// dependencies resolve, no cycles, rates are sane.
static int doctor(const std::string& manifest){
    std::string baseDir="."; size_t slash=manifest.find_last_of('/');
    if(slash!=std::string::npos) baseDir=manifest.substr(0,slash);
    std::ifstream f(manifest);
    if(!f){ std::cout << "[FAIL] manifest nahi mila: " << manifest << "\n"; return 1; }
    struct S { std::string name, script; std::vector<std::string> after; double rate=0; };
    std::vector<S> ss; S* cur=nullptr; std::string line;
    while(std::getline(f,line)){
        std::string t=trim(line); if(t.empty()||t[0]=='#') continue;
        if(t.front()=='['&&t.back()==']'){ std::string h=trim(t.substr(1,t.size()-2)); std::string nm=h.rfind("service ",0)==0?trim(h.substr(8)):h; ss.push_back({nm,"",{},0}); cur=&ss.back(); continue; }
        size_t eq=t.find('='); if(eq==std::string::npos||!cur) continue;
        std::string k=trim(t.substr(0,eq)), v=trim(t.substr(eq+1));
        if(k=="script") cur->script=baseDir+"/"+v;
        else if(k=="rate_hz") cur->rate=std::atof(v.c_str());
        else if(k=="after"){ std::stringstream a(v); std::string d; while(std::getline(a,d,',')){ d=trim(d); if(!d.empty()) cur->after.push_back(d); } }
    }
    int fails=0, warns=0;
    auto ok  =[&](const std::string& m){ std::cout<<"[ OK ] "<<m<<"\n"; };
    auto fail=[&](const std::string& m){ std::cout<<"[FAIL] "<<m<<"\n"; fails++; };
    auto warn=[&](const std::string& m){ std::cout<<"[WARN] "<<m<<"\n"; warns++; };
    std::cout << "zron doctor -- " << manifest << "\n\n";
    if(ss.empty()){ fail("manifest mein koi service nahi"); return 1; }
    ok(std::to_string(ss.size())+" service(s) defined");
    std::set<std::string> names; for(auto& s:ss) names.insert(s.name);
    for(auto& s:ss){
        std::ifstream sf(s.script);
        if(!sf){ fail("service '"+s.name+"': script missing ("+s.script+")"); continue; }
        std::stringstream b; b<<sf.rdbuf();
        try { parseSource(b.str()); ok("service '"+s.name+"': script parses"); }
        catch(RCError& e){ fail("service '"+s.name+"': parse error -- "+e.what_hinglish()); }
        for(auto& d:s.after) if(!names.count(d)) fail("service '"+s.name+"': depends on unknown '"+d+"'");
        if(s.rate>50) warn("service '"+s.name+"': rate "+f2(s.rate)+"Hz is high for a virtual-clock demo");
        if(s.rate<0) fail("service '"+s.name+"': negative rate_hz");
    }
    // cycle detection over 'after'
    std::map<std::string,std::vector<std::string>> g; for(auto& s:ss) g[s.name]=s.after;
    std::set<std::string> visiting, done; bool cyc=false;
    std::function<void(const std::string&)> dfs=[&](const std::string& n){
        if(done.count(n)) return;
        if(visiting.count(n)){ cyc=true; return; }
        visiting.insert(n);
        for(auto& d:g[n]) if(names.count(d)) dfs(d);
        visiting.erase(n); done.insert(n);
    };
    for(auto& s:ss) dfs(s.name);
    if(cyc) fail("dependency cycle detected in 'after' graph"); else ok("dependency graph is acyclic");
    std::cout << "\nsummary: " << (fails?"NOT READY":"READY TO BOOT") << "  ("<<fails<<" fail, "<<warns<<" warn)\n";
    return fails?1:0;
}

// ========================================================= scaffold (zron new)
static int scaffoldNew(const std::string& name){
    if(name.empty()){ std::cerr<<"usage: zron new <robot-name>\n"; return 2; }
    std::string dir=name;
    std::string mkcmd="mkdir -p '"+dir+"'";
    if(std::system(mkcmd.c_str())!=0){ std::cerr<<"could not create "<<dir<<"\n"; return 1; }
    auto write=[&](const std::string& rel, const std::string& body){
        std::ofstream o(dir+"/"+rel); o<<body; };
    write("robot.manifest",
        "# "+name+" -- zronOS manifest. `zron run "+dir+"/robot.manifest`\n\n"
        "[service bringup]\nscript = bringup.rc\nrestart = never\nrate_hz = 0\nautostart = true\n\n"
        "[service heartbeat]\nscript = heartbeat.rc\nafter = bringup\nrate_hz = 1\nrestart = on-failure\nautostart = true\n");
    write("bringup.rc",
        "# boot par ek baar chalta hai\nrobot jodo \""+name+"\"\ndikhao \""+name+" online\"\n");
    write("heartbeat.rc",
        "# har tick battery bus par bhejo\nmaano b = battery kitni hai\nbolo \"battery\" b\nsehat theek\n");
    write("README.md",
        "# "+name+"\n\nA zronOS robot. Boot it:\n\n```bash\nzron doctor "+dir+"/robot.manifest\nzron run "+dir+"/robot.manifest\n```\n");
    std::cout << "created ./" << dir << "/  (robot.manifest, bringup.rc, heartbeat.rc, README.md)\n";
    std::cout << "next:  zron doctor " << dir << "/robot.manifest  &&  zron run " << dir << "/robot.manifest\n";
    return 0;
}

static int usage(){
    std::cout <<
      "zronOS " << OS_VERSION << " -- robot operating environment (services are Robot Chalao .rc)\n\n"
      "usage:\n"
      "  zron run <manifest>        boot a manifest and drop into the shell\n"
      "  zron demo [manifest]       boot and run a deterministic story, then exit\n"
      "  zron doctor <manifest>     preflight checks (no boot): parse, deps, cycles\n"
      "  zron new <name>            scaffold a new robot (manifest + services)\n"
      "  zron mujoco <model.xml>    drive a MuJoCo model live via the bridge (HITL)\n"
      "  zron --version             print version\n"
      "  zron --help                this help\n\n"
      "flags:  --seconds N (with demo)\n";
    return 0;
}

int main(int argc, char** argv){
    std::vector<std::string> a; for(int i=1;i<argc;++i) a.push_back(argv[i]);
    if(a.empty()) return usage();

    std::string sub=a[0];
    if(sub=="--version"){ std::cout<<"zronOS "<<OS_VERSION<<"\n"; return 0; }
    if(sub=="--help"||sub=="help") return usage();
    if(sub=="doctor"){ if(a.size()<2){ std::cerr<<"usage: zron doctor <manifest>\n"; return 2; } return doctor(a[1]); }
    if(sub=="new"){ return scaffoldNew(a.size()>1?a[1]:""); }
    if(sub=="mujoco"){ return zmj::mujocoCompat(a); }

    // demo / run (+ back-compat: bare manifest path or --demo flag)
    bool demoMode=false; double seconds=8.0; std::string manifest="system/system.manifest";
    size_t start=0;
    if(sub=="demo"){ demoMode=true; start=1; }
    else if(sub=="run"){ demoMode=false; start=1; }
    for(size_t i=start;i<a.size();++i){
        if(a[i]=="--demo") demoMode=true;
        else if(a[i]=="--seconds"&&i+1<a.size()) seconds=std::atof(a[++i].c_str());
        else if(!a[i].empty() && a[i][0]!='-') manifest=a[i];
    }
    std::string baseDir="."; size_t slash=manifest.find_last_of('/');
    if(slash!=std::string::npos) baseDir=manifest.substr(0,slash);
    ZronOS os;
    if(!os.loadManifest(manifest, baseDir)) return 2;
    return demoMode ? os.demo(seconds) : os.shell();
}
