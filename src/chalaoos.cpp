// ChalaoOS -- a robot operating environment whose services are Robot Chalao
// programs. init + supervisor + scheduler + HAL + IPC bus + shell.
// NOT a bare-metal kernel; it operates at the runtime layer (like "ROS").
// Built on the Robot Chalao native C++ core (vendor/chalao).
#include <algorithm>
#include <cstdio>
#include <deque>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include "value.hpp"
#include "ast.hpp"
#include "lexer.hpp"
#include "parser.hpp"
#include "interp.hpp"

static const char* OS_VERSION = "0.1.0";

static std::string ts(double t){ char b[32]; std::snprintf(b,sizeof b,"%6.2fs", t); return b; }
static std::string f2(double d){ char b[32]; std::snprintf(b,sizeof b,"%.1f", d); return b; }

// ------------------------------------------------------------------ IPC bus
struct Bus {
    std::map<std::string,std::string> latest;   // topic -> last value (as text)
    std::map<std::string,long> count;
    void publish(const std::string& topic, const std::string& val){ latest[topic]=val; count[topic]++; }
};

// -------------------------------------------------------------- HAL (devices)
struct HAL {
    double battery = 100.0;      // percent, drains over time
    bool   estopped = false;
    bool   obstacle = false;
    double odom = 0.0;           // metres driven
    double drain_per_s = 4.0;
    void tick(double dt){ if(!estopped) battery -= drain_per_s*dt; if(battery<0) battery=0; }
    bool drive(double dist){ if(estopped) return false; odom += dist; return true; }
    void estop(){ estopped = true; }
};

// ------------------------------------------------------------------ services
enum class St { STARTING, RUNNING, STOPPED, FAILED, RESTARTING };
enum class Rp { NEVER, ON_FAILURE, ALWAYS };
static const char* stName(St s){ switch(s){case St::STARTING:return "STARTING";case St::RUNNING:return "RUNNING";
    case St::STOPPED:return "STOPPED";case St::FAILED:return "FAILED";default:return "RESTARTING";} }
static const char* rpName(Rp r){ return r==Rp::NEVER?"never":r==Rp::ON_FAILURE?"on-failure":"always"; }

struct Service {
    std::string name, script;
    Rp restart = Rp::ON_FAILURE;
    std::vector<std::string> after;
    double rate_hz = 0.0;        // 0 = run once at boot
    bool autostart = true;
    // runtime
    St state = St::STOPPED;
    int restarts = 0;
    double startedAt = 0.0, nextRun = 0.0, retryAt = -1.0;
    NodePtr prog;
    std::unique_ptr<Interpreter> interp;
    std::deque<std::string> log;
    bool parsed = false;
    std::string parseErr;
};

// ------------------------------------------------------------------ the OS
class ChalaoOS {
    double clock = 0.0;
    const double DT = 0.5;
    HAL hal;
    Bus bus;
    std::vector<std::unique_ptr<Service>> svcs;
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
        auto trim=[](std::string s){ size_t a=s.find_first_not_of(" \t\r\n"); size_t b=s.find_last_not_of(" \t\r\n");
            return a==std::string::npos? std::string() : s.substr(a,b-a+1); };
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
        std::string nm = s->name;
        s->interp->outSink = [this,s](const std::string& out){
            s->log.push_back(out); if(s->log.size()>40) s->log.pop_front();
            kmsg(s->name, out);
        };
        s->interp->robotHook = [this,s](const std::string& m, std::map<std::string,Value>& A, Value& outv)->bool{
            auto num=[&](const char* k){ auto it=A.find(k); return it!=A.end()?it->second.num:0.0; };
            if(m=="battery"){ outv=Value::Num_(hal.battery); return true; }
            if(m=="publish"){
                auto tit=A.find("topic"); auto vit=A.find("value");
                std::string topic = tit!=A.end()? (tit->second.t==VT::Str?tit->second.str:fmtValue(tit->second)) : "?";
                std::string val   = vit!=A.end()? fmtValue(vit->second) : "";
                bus.publish(topic,val); outv=Value::Nil(); return true;
            }
            if(m=="obstacle_near"){ outv=Value::Bool_(hal.obstacle); return true; }
            if(m=="base_move"){
                if(hal.drive(num("distance"))) { kmsg(s->name, "[hal] wheels drove "+f2(num("distance"))+" m (odom "+f2(hal.odom)+")"); }
                else kmsg(s->name, "[hal] BLOCKED -- actuators are in e-stop");
                outv=Value::Nil(); return true;
            }
            if(m=="base_rotate"){ if(hal.estopped){ kmsg(s->name,"[hal] BLOCKED -- e-stop"); } outv=Value::Nil(); return true; }
            if(m=="estop"){
                bool was=hal.estopped; hal.estop();
                if(!was){ kmsg("kernel","*** E-STOP asserted by service '"+s->name+"' -- actuators cut ***"); estopLogged=true; }
                outv=Value::Nil(); return true;
            }
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
            if(!f){ s->parseErr="script nahi mila: "+s->script; s->state=St::FAILED; kmsg("init","  FAILED: "+s->parseErr); return; }
            try { s->prog = parseSource(ss.str()); s->parsed=true; }
            catch(RCError& e){ s->parseErr=e.what_hinglish(); s->state=St::FAILED; kmsg("init","  FAILED to parse: "+s->parseErr); return; }
        }
        wire(s);
        s->state = St::RUNNING; s->nextRun = clock;
        kmsg("init", "  '"+s->name+"' is RUNNING");
        if(s->rate_hz==0.0) runOnce(s);      // run-once service executes at start
    }

    void runOnce(Service* s){
        try { s->interp->run(s->prog); }
        catch(RCError& e){ fail(s, e.what_hinglish()); }
    }

    void fail(Service* s, const std::string& why){
        s->state = St::FAILED;
        kmsg("kernel", "service '"+s->name+"' FAILED: "+why);
        if(s->restart==Rp::ALWAYS || s->restart==Rp::ON_FAILURE){
            double backoff = std::min(4.0, 0.5*(s->restarts+1));
            s->state = St::RESTARTING; s->retryAt = clock+backoff; s->restarts++;
            kmsg("kernel", "  restart policy '"+std::string(rpName(s->restart))+"': retry in "+f2(backoff)+"s (#"+std::to_string(s->restarts)+")");
        }
    }

    // ---- dependency-ordered boot ----
    void boot(){
        std::cout << "\n";
        std::cout << "  +--------------------------------------------------+\n";
        std::cout << "  |   ChalaoOS " << OS_VERSION << "  --  robot operating environment |\n";
        std::cout << "  |   services ki zubaan: Robot Chalao (.rc)         |\n";
        std::cout << "  +--------------------------------------------------+\n\n";
        kmsg("init", "boot: "+std::to_string(svcs.size())+" service(s) in manifest");
        // topological start by 'after'
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

    // ---- one scheduler tick ----
    void tick(){
        clock += DT;
        hal.tick(DT);
        for(auto& s:svcs){
            if(s->state==St::RESTARTING && s->retryAt>=0 && clock>=s->retryAt){
                kmsg("kernel","restarting '"+s->name+"'"); wire(s.get()); s->state=St::RUNNING; s->nextRun=clock;
            }
            if(s->state!=St::RUNNING || s->rate_hz<=0.0) continue;
            if(clock+1e-9 < s->nextRun) continue;
            s->nextRun += 1.0/s->rate_hz;
            try { s->interp->run(s->prog); }
            catch(RCError& e){ fail(s.get(), e.what_hinglish()); }
        }
    }

    void ps(){
        std::cout << "\n  SERVICE     STATE        RESTARTS  RATE     UPTIME\n";
        std::cout <<   "  -------     -----        --------  ----     ------\n";
        for(auto& s:svcs){
            char row[160];
            std::snprintf(row,sizeof row,"  %-10s  %-11s  %-8d  %-7s  %.1fs",
                s->name.c_str(), stName(s->state), s->restarts,
                s->rate_hz>0? (f2(s->rate_hz)+"Hz").c_str():"once",
                s->state==St::RUNNING? (clock-s->startedAt):0.0);
            std::cout << row << "\n";
        }
        std::cout << "\n";
    }

    void topics(){
        std::cout << "\n  TOPIC          LAST VALUE      MSGS\n";
        std::cout <<   "  -----          ----------      ----\n";
        for(auto& kv:bus.latest){
            char row[160]; std::snprintf(row,sizeof row,"  %-13s  %-14s  %ld",
                kv.first.c_str(), kv.second.c_str(), bus.count[kv.first]);
            std::cout << row << "\n";
        }
        std::cout << "\n";
    }

    void halt(){
        kmsg("init","halt: stopping services");
        for(auto& s:svcs) if(s->state==St::RUNNING) s->state=St::STOPPED;
        std::cout << "\n  ChalaoOS band. Alvida.\n";
    }

    // ---- non-interactive demo ----
    int demo(double seconds){
        boot();
        int ticks = (int)(seconds/DT);
        for(int i=0;i<ticks;++i) tick();
        kmsg("kernel","battery ab "+f2(hal.battery)+"%, e-stop="+(hal.estopped?"HAAN":"nahi"));
        ps();
        topics();
        halt();
        return 0;
    }

    // ---- interactive shell ----
    int shell(){
        boot();
        std::string line;
        std::cout << "chalaoos> " << std::flush;
        while(std::getline(std::cin,line)){
            std::stringstream ss(line); std::string cmd,arg; ss>>cmd; ss>>arg;
            if(cmd=="help"){ std::cout <<
                "  commands: ps | topics | start N | stop N | restart N | log N |\n"
                "            echo TOPIC | tick [n] | uptime | estop | halt | help\n"; }
            else if(cmd=="ps") ps();
            else if(cmd=="topics") topics();
            else if(cmd=="uptime") std::cout << "  uptime "<<f2(clock)<<"s, battery "<<f2(hal.battery)<<"%\n";
            else if(cmd=="tick"){ int n=arg.empty()?1:std::atoi(arg.c_str()); for(int i=0;i<n;++i) tick(); }
            else if(cmd=="estop"){ hal.estop(); std::cout<<"  e-stop asserted.\n"; }
            else if(cmd=="halt"){ halt(); break; }
            else if(cmd=="start"){ Service* s=find(arg); if(s) start(s); else std::cout<<"  no such service\n"; }
            else if(cmd=="stop"){ Service* s=find(arg); if(s){ s->state=St::STOPPED; std::cout<<"  stopped\n";} else std::cout<<"  no such service\n"; }
            else if(cmd=="restart"){ Service* s=find(arg); if(s){ wire(s); s->state=St::RUNNING; s->restarts++; s->nextRun=clock; std::cout<<"  restarted\n";} else std::cout<<"  no such service\n"; }
            else if(cmd=="log"){ Service* s=find(arg); if(s){ for(auto& l:s->log) std::cout<<"    "<<l<<"\n"; } else std::cout<<"  no such service\n"; }
            else if(cmd=="echo"){ auto it=bus.latest.find(arg); std::cout<<"  "<<(it!=bus.latest.end()?it->second:"(no such topic)")<<"\n"; }
            else if(cmd.empty()){}
            else std::cout << "  anjaan hukum: "<<cmd<<" (try 'help')\n";
            std::cout << "chalaoos> " << std::flush;
        }
        return 0;
    }
};

int main(int argc, char** argv){
    std::string manifest = "system/system.manifest";
    std::string baseDir = ".";
    bool demoMode=false; double seconds=8.0;
    for(int i=1;i<argc;++i){ std::string a=argv[i];
        if(a=="--demo") demoMode=true;
        else if(a=="--seconds"&&i+1<argc) seconds=std::atof(argv[++i]);
        else if(a=="--version"){ std::cout<<"ChalaoOS "<<OS_VERSION<<"\n"; return 0; }
        else if(a[0]!='-') manifest=a;
    }
    // baseDir = dir of the manifest, so relative script paths resolve
    size_t slash=manifest.find_last_of('/');
    if(slash!=std::string::npos) baseDir=manifest.substr(0,slash);
    ChalaoOS os;
    if(!os.loadManifest(manifest, baseDir)) return 2;
    return demoMode ? os.demo(seconds) : os.shell();
}
