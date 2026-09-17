// Robot Chalao -- tree-walking interpreter + simulation backend (native C++ core).
// Sim-only: har robot hukum ek Hinglish [nakli] line chaapta hai (bilkul
// reference jaisa) aur ek maqool value lautata hai, toh bina ROS ke poora
// program chalta hai.
#pragma once
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <functional>
#include <iostream>
#include <cstdio>
#include <cmath>
#include "ast.hpp"
#include "value.hpp"
#include "parser.hpp"

struct Env {
    std::map<std::string, Value> vars;
    std::shared_ptr<Env> parent;
    explicit Env(std::shared_ptr<Env> p = nullptr) : parent(std::move(p)) {}
    Value get(const std::string& n, int line) {
        for (Env* e = this; e; e = e->parent.get()) { auto it = e->vars.find(n); if (it != e->vars.end()) return it->second; }
        throw RCError("'" + n + "' naam ki koi cheez nahi bani", "undefined name '" + n + "'", line);
    }
    void assign(const std::string& n, const Value& v) {
        for (Env* e = this; e; e = e->parent.get()) { if (e->vars.count(n)) { e->vars[n] = v; return; } }
        vars[n] = v;
    }
    void set(const std::string& n, const Value& v) { vars[n] = v; }
};
using EnvPtr = std::shared_ptr<Env>;

struct ReturnEx { Value value; };
struct BreakEx {};

class Interpreter {
    EnvPtr globals = std::make_shared<Env>();
    std::map<std::string, std::pair<NodePtr, EnvPtr>> funcs;
    bool simulate = true;
    // sim state
    Value simPose = makePose(0.4, 0.0, 0.3, 0.0, 3.14, 0.0);
    Value simJoints = makeJoints({0.0, -1.57, 0.0, -1.57, 0.0, 0.0});
    static const int TIMER_TICKS = 3;
public:
    explicit Interpreter(bool sim = true) : simulate(sim) {}
    void run(const NodePtr& program) { execBlock(program->kids, globals); }
    // --- ChalaoOS integration hooks (null by default -> standalone [nakli] behaviour) ---
    std::function<void(const std::string&)> outSink;   // capture all program output (dikhao / [nakli] / ruko)
    std::function<bool(const std::string&, std::map<std::string,Value>&, Value&)> robotHook;  // intercept robot commands

private:
    void emit(const std::string& s) { if (outSink) outSink(s); else std::cout << s << "\n"; }
    void say(const std::string& m) { emit("[nakli] " + m); }
    static std::string fmtf(const char* f, double a){ char b[128]; std::snprintf(b,sizeof b,f,a); return b; }

    void execBlock(const std::vector<NodePtr>& body, const EnvPtr& env) { for (auto& st : body) execStmt(st, env); }

    void execStmt(const NodePtr& n, const EnvPtr& env) {
        switch (n->t) {
            case NT::Let: env->assign(n->s, eval(n->a, env)); return;
            case NT::Print: {
                std::string out; for (size_t i=0;i<n->kids.size();++i){ if(i) out+=" "; out += fmtValue(eval(n->kids[i], env)); }
                emit(out); return;
            }
            case NT::ExprStmt: eval(n->a, env); return;
            case NT::Return: throw ReturnEx{ n->a ? eval(n->a, env) : Value::Nil() };
            case NT::Break: throw BreakEx{};
            case NT::If:
                if (truthy(eval(n->a, env))) execBlock(n->kids, std::make_shared<Env>(env));
                else if (!n->kids2.empty()) execBlock(n->kids2, std::make_shared<Env>(env));
                return;
            case NT::While:
                while (truthy(eval(n->a, env))) { try { execBlock(n->kids, std::make_shared<Env>(env)); } catch (BreakEx&) { break; } }
                return;
            case NT::ForEach: {
                Value seq = eval(n->a, env);
                std::vector<Value> items;
                if (seq.t == VT::List) items = *seq.list;
                else if (seq.t == VT::Str) { for (char c : seq.str) items.push_back(Value::Str_(std::string(1,c))); }
                else throw RCError("is cheez par 'har' nahi chal sakta", "value is not iterable", n->line);
                for (auto& item : items) {
                    auto child = std::make_shared<Env>(env); child->set(n->s, item);
                    try { execBlock(n->kids, child); } catch (BreakEx&) { break; }
                }
                return;
            }
            case NT::FuncDef: funcs[n->s] = { n, env }; return;
            case NT::Try:
                try { execBlock(n->kids, std::make_shared<Env>(env)); }
                catch (ReturnEx&) { throw; } catch (BreakEx&) { throw; }
                catch (RCError& e) {
                    auto child = std::make_shared<Env>(env);
                    if (n->hasErr) child->set(n->errName, Value::Str_(e.what_hinglish()));
                    execBlock(n->kids2, child);
                }
                return;
            case NT::Timer: {
                for (int i = 0; i < TIMER_TICKS; ++i) { try { execBlock(n->kids, std::make_shared<Env>(env)); } catch (BreakEx&) { break; } }
                return;
            }
            case NT::Subscribe: {
                Value topic = eval(n->a, env);
                say("sun raha hai " + (topic.t==VT::Str?topic.str:fmtValue(topic)) + " (" + n->s + ")");
                auto child = std::make_shared<Env>(env);
                Value msg = Value::Dict_();
                msg.dict->push_back({"topic", topic}); msg.dict->push_back({"type", Value::Str_(n->s)});
                child->set(n->b->s, msg);
                execBlock(n->kids, child); return;
            }
            case NT::ActionSend: {
                Value name = eval(n->a, env), goal = eval(n->b, env);
                say("action bheja " + (name.t==VT::Str?name.str:fmtValue(name)) + " goal=" + reprValue(goal));
                if (n->hasErr) for (int pct : {25,50,75,100}) {
                    auto child = std::make_shared<Env>(env);
                    Value fb = Value::Dict_(); fb.dict->push_back({"progress", Value::Num_(pct)});
                    child->set(n->errName, fb); execBlock(n->kids, child);
                }
                return;
            }
            case NT::RobotCommand: runCommand(n, env); return;
            default: throw RCError("yeh statement chal nahi sakta", "cannot run statement", n->line);
        }
    }

    // ---- expressions ----
    Value eval(const NodePtr& n, const EnvPtr& env) {
        switch (n->t) {
            case NT::Num: return Value::Num_(n->num);
            case NT::Str: return Value::Str_(n->s);
            case NT::Bool: return Value::Bool_(n->bval);
            case NT::Ident: return env->get(n->s, n->line);
            case NT::ListLit: { Value v = Value::List_(); for (auto& k : n->kids) v.list->push_back(eval(k, env)); return v; }
            case NT::DictLit: {
                Value v = Value::Dict_();
                for (auto& pr : n->pairs) { Value k = eval(pr.first, env); v.dict->push_back({ k.t==VT::Str?k.str:fmtValue(k), eval(pr.second, env) }); }
                return v;
            }
            case NT::Member: {
                Value o = eval(n->a, env);
                if (o.t == VT::Dict) { for (auto& p : *o.dict) if (p.first == n->s) return p.second; }
                if (o.t == VT::Object) { Value* f = o.obj->find(n->s); if (f) return *f; }
                throw RCError("'" + n->s + "' is cheez mein nahi hai", "no attribute '" + n->s + "'", n->line);
            }
            case NT::UnaryOp: {
                Value v = eval(n->a, env);
                if (n->s == "-") { if (v.t != VT::Num) throw RCError("is par '-' nahi chalta", "cannot negate", n->line); return Value::Num_(-v.num); }
                return Value::Bool_(!truthy(v));   // nahi
            }
            case NT::BinOp: return binop(n, env);
            case NT::TypedCtor: return typedCtor(n, env);
            case NT::RobotCommand: return runCommand(n, env);
            case NT::Call: return call(n, env);
            default: throw RCError("yeh expression samajh nahi aaya", "cannot evaluate", n->line);
        }
    }

    Value binop(const NodePtr& n, const EnvPtr& env) {
        const std::string& op = n->s;
        if (op == "aur") return Value::Bool_(truthy(eval(n->a, env)) && truthy(eval(n->b, env)));
        if (op == "ya")  return Value::Bool_(truthy(eval(n->a, env)) || truthy(eval(n->b, env)));
        Value a = eval(n->a, env), b = eval(n->b, env);
        auto bothNum = [&]{ return a.t == VT::Num && b.t == VT::Num; };
        if (op == "+") {
            if (bothNum()) return Value::Num_(a.num + b.num);
            if (a.t == VT::Str && b.t == VT::Str) return Value::Str_(a.str + b.str);
            if (a.t == VT::List && b.t == VT::List) { Value v = Value::List_(); *v.list = *a.list; for (auto& x : *b.list) v.list->push_back(x); return v; }
            throw RCError("in do cheezon par '+' nahi chalta", "type error in +", n->line);
        }
        if (op=="-"||op=="*"||op=="/"||op=="%") {
            if (!bothNum()) throw RCError("in do cheezon par '"+op+"' nahi chalta", "numeric operands required", n->line);
            if (op=="-") return Value::Num_(a.num-b.num);
            if (op=="*") return Value::Num_(a.num*b.num);
            if (op=="/") { if (b.num==0) throw RCError("zero se bhaag nahi kar sakte","division by zero",n->line); return Value::Num_(a.num/b.num); }
            return Value::Num_(std::fmod(a.num,b.num));
        }
        if (op=="=="||op=="!=") { bool eq = valuesEqual(a,b); return Value::Bool_(op=="=="?eq:!eq); }
        if (op=="<"||op==">"||op=="<="||op==">=") {
            if (!bothNum()) throw RCError("in do cheezon par '"+op+"' nahi chalta", "numeric comparison required", n->line);
            if (op=="<")  return Value::Bool_(a.num <  b.num);
            if (op==">")  return Value::Bool_(a.num >  b.num);
            if (op=="<=") return Value::Bool_(a.num <= b.num);
            return Value::Bool_(a.num >= b.num);
        }
        throw RCError("anjaan operator '"+op+"'", "unknown operator", n->line);
    }
    static bool valuesEqual(const Value& a, const Value& b) {
        if (a.t != b.t) return false;
        switch (a.t) { case VT::Num: return a.num==b.num; case VT::Str: return a.str==b.str;
            case VT::Bool: return a.b==b.b; case VT::Nil: return true; default: return false; }
    }

    Value typedCtor(const NodePtr& n, const EnvPtr& env) {
        std::vector<double> a; for (auto& k : n->kids) a.push_back(eval(k, env).num);
        auto at=[&](size_t i){ return i<a.size()?a[i]:0.0; };
        if (n->s=="pose") return makePose(at(0),at(1),at(2),at(3),at(4),at(5));
        if (n->s=="twist") return a.size()==2 ? makeTwist(a[0],0,0,0,0,a[1]) : makeTwist(at(0),at(1),at(2),at(3),at(4),at(5));
        Value j=makeJoints(a); return j;
    }

    Value call(const NodePtr& n, const EnvPtr& env) {
        const std::string& name = n->s;
        std::vector<Value> args; for (auto& k : n->kids) args.push_back(eval(k, env));
        if (name == "_ruko") { double s = args.empty()?0.0:args[0].num; emit("[nakli] ruk raha hoon " + fmtf("%.2f", s) + " second"); return Value::Nil(); }
        if (name == "_deadline") { double s = args.empty()?0.0:args[0].num; emit("[nakli] deadline " + fmtf("%.2f", s) + " second"); return Value::Nil(); }
        // builtins
        if (name=="len") { double L=0; if(!args.empty()){ auto&x=args[0]; L = x.t==VT::List?x.list->size(): x.t==VT::Str?x.str.size(): x.t==VT::Dict?x.dict->size():0; } return Value::Num_(L); }
        if (name=="range") { Value v=Value::List_(); long a0=0,a1=0,st=1; if(args.size()==1){a1=(long)args[0].num;} else if(args.size()>=2){a0=(long)args[0].num;a1=(long)args[1].num; if(args.size()>=3) st=(long)args[2].num;} if(st!=0) for(long i=a0; (st>0)?(i<a1):(i>a1); i+=st) v.list->push_back(Value::Num_(i)); return v; }
        if (name=="abs") return Value::Num_(std::fabs(args.at(0).num));
        if (name=="round") { double d=args.at(0).num; int nd=args.size()>1?(int)args[1].num:0; double f=std::pow(10,nd); return Value::Num_(std::round(d*f)/f); }
        if (name=="int") return Value::Num_((double)(long long)args.at(0).num);
        if (name=="str") return Value::Str_(fmtValue(args.at(0)));
        if (name=="min"||name=="max") {
            std::vector<Value> items = (args.size()==1 && args[0].t==VT::List) ? *args[0].list : args;
            double best = items.at(0).num; for (auto& x : items) { if (name=="min") best=std::min(best,x.num); else best=std::max(best,x.num); } return Value::Num_(best);
        }
        auto it = funcs.find(name);
        if (it != funcs.end()) return callUser(name, args, n->line);
        throw RCError("'" + name + "' naam ka koi kaam nahi bana", "undefined function '" + name + "'", n->line);
    }
    Value callUser(const std::string& name, std::vector<Value>& args, int line) {
        auto& [fnode, closure] = funcs[name];
        if (args.size() != fnode->params.size())
            throw RCError("'"+name+"' ko "+std::to_string(fnode->params.size())+" cheezein chahiye, "+std::to_string(args.size())+" di", "argument count mismatch", line);
        auto local = std::make_shared<Env>(closure);
        for (size_t i=0;i<args.size();++i) local->set(fnode->params[i], args[i]);
        try { execBlock(fnode->kids, local); } catch (ReturnEx& r) { return r.value; }
        return Value::Nil();
    }

    // ---- robot command dispatch (simulation backend) ----
    Value runCommand(const NodePtr& n, const EnvPtr& env) {
        std::map<std::string, Value> A;
        for (auto& pr : n->cargs) A[pr.first] = eval(pr.second, env);
        const std::string& m = n->s;
        auto S = [&](const char* k){ auto it=A.find(k); return it!=A.end()? (it->second.t==VT::Str?it->second.str:fmtValue(it->second)) : std::string(); };
        auto N = [&](const char* k){ auto it=A.find(k); return it!=A.end()?it->second.num:0.0; };
        auto V = [&](const char* k){ auto it=A.find(k); return it!=A.end()?it->second:Value::Nil(); };
        auto B = [&](const char* k){ auto it=A.find(k); return it!=A.end()&&truthy(it->second); };
        (void)S;(void)N;(void)V;(void)B;
        if (robotHook) { Value out; if (robotHook(m, A, out)) return out; }

        if (m=="connect")    { say("robot '" + S("name") + "' se jud gaye"); return Value::Nil(); }
        if (m=="disconnect") { say("robot chhod diya"); return Value::Nil(); }
        if (m=="set_namespace"){ say("namespace set: " + S("ns")); return Value::Nil(); }
        if (m=="param_set")  { say("param set: " + S("key") + " = " + reprValue(V("value"))); return Value::Nil(); }
        if (m=="param_get")  { say("param get: " + S("key")); return Value::Num_(0.0); }
        // arm
        if (m=="arm_home")   { say("arm ghar (home) ja raha hai"); return Value::Nil(); }
        if (m=="arm_pose")   { Value p=V("pose"); say("pose pe ja raha hai: " + reprValue(p)); if(p.t==VT::Object && p.obj->type=="Pose") simPose=p; return Value::Nil(); }
        if (m=="arm_joints") { Value j=V("joints"); say("joints pe ja raha hai: " + reprValue(j)); if(j.t==VT::Object && j.obj->type=="Joints") simJoints=j; return Value::Nil(); }
        if (m=="arm_cartesian"){ emit("[nakli] seedha (Cartesian) ja raha hai: (" + fmtf("%.3f",N("x")) + ", " + fmtf("%.3f",N("y")) + ", " + fmtf("%.3f",N("z")) + ")"); return Value::Nil(); }
        if (m=="arm_named")  { say("named pose '" + S("name") + "' pe ja raha hai"); return Value::Nil(); }
        if (m=="gripper")    { say(std::string("gripper ") + (B("close") ? "band (pakad raha hai)" : "khol raha hai")); return Value::Nil(); }
        if (m=="set_speed")  { say("speed set: " + fmtf("%.2f", N("scale"))); return Value::Nil(); }
        if (m=="set_planner"){ say("planner set: " + S("planner")); return Value::Nil(); }
        if (m=="add_obstacle"){ say("rukawat '"+S("name")+"' jodi at "+reprValue(V("pose"))+" size "+reprValue(V("size"))); return Value::Nil(); }
        if (m=="remove_obstacle"){ say("rukawat '"+S("name")+"' hatai"); return Value::Nil(); }
        if (m=="attach")     { say("'"+S("name")+"' attach kiya"); return Value::Nil(); }
        if (m=="detach")     { say("'"+S("name")+"' detach kiya"); return Value::Nil(); }
        if (m=="get_pose")   { say("abhi yahan hai: " + reprValue(simPose)); return simPose; }
        if (m=="get_joints") { say("joints abhi: " + reprValue(simJoints)); return simJoints; }
        if (m=="ik")         { say("IK nikala for " + reprValue(V("pose"))); return simJoints; }
        if (m=="fk")         { say("FK nikala for " + reprValue(V("joints"))); return simPose; }
        // base
        if (m=="base_move")  { say(std::string(B("backward")?"peeche":"aage") + " chal raha hai " + fmtf("%.2f", N("distance")) + " " + S("unit")); return Value::Nil(); }
        if (m=="base_rotate"){ say("ghoom raha hai " + fmtf("%.1f", N("angle")) + " " + S("unit")); return Value::Nil(); }
        if (m=="base_twist") { say("twist: v=" + fmtf("%.2f",N("v")) + " w=" + fmtf("%.2f",N("w"))); return makeTwist(N("v"),0,0,0,0,N("w")); }
        if (m=="base_stop")  { say("base ruk gaya"); return Value::Nil(); }
        if (m=="map_load")   { say("map load: " + S("path")); return Value::Nil(); }
        if (m=="localize")   { auto it=A.find("pose"); say(std::string("localize") + (it!=A.end()? (" at " + reprValue(it->second)) : "")); return Value::Nil(); }
        if (m=="nav_to")     { emit("[nakli] nav: (" + fmtf("%.2f",N("x")) + ", " + fmtf("%.2f",N("y")) + ", theta=" + fmtf("%.2f",N("theta")) + ") pe ja raha hai"); return Value::Nil(); }
        if (m=="follow_waypoints"){ Value p=V("points"); int c = p.t==VT::List? (int)p.list->size():0; say("waypoints follow: " + std::to_string(c) + " points"); return Value::Nil(); }
        if (m=="slam_start") { say("SLAM shuru -- map banaya ja raha hai"); return Value::Nil(); }
        if (m=="map_save")   { say("map save: " + S("name")); return Value::Nil(); }
        if (m=="obstacle_near"){ say("obstacle check -- saaf raasta"); return Value::Bool_(false); }
        // perception
        if (m=="camera_view"){ say("camera dekh raha hai: " + S("topic")); return makeImage(640,480,"rgb8",S("topic")); }
        if (m=="photo")      { say("photo li -> " + S("path")); return Value::Nil(); }
        if (m=="find_object"){ say("dhoondh raha hai: '" + S("description") + "' ... mil gaya"); return makeDetection(S("description"), makePose(0.5,0.1,0.2,0,3.14,0), 0.91); }
        if (m=="read_lidar") { say("lidar padha"); Value o; o.t=VT::Object; o.obj=std::make_shared<Obj>(); o.obj->type="PointCloud"; return o; }
        if (m=="read_depth") { say("depth padha"); return makeImage(640,480,"16UC1","/depth"); }
        if (m=="read_imu")   { say("imu padha"); return Value::Dict_(); }
        if (m=="read_ft")    { say("force/torque padha"); return Value::Dict_(); }
        if (m=="battery")    { say("battery: 87%"); return Value::Num_(87.0); }
        if (m=="find_apriltag"){ say("AprilTag dhoondha -- 1 mila"); Value v=Value::List_(); v.list->push_back(makeDetection("tag_0",makePose(0.6,0,0.4,0,0,0),0.98)); return v; }
        if (m=="find_aruco") { say("ArUco dhoondha -- 1 mila"); Value v=Value::List_(); v.list->push_back(makeDetection("aruco_5",makePose(0.6,0,0.4,0,0,0),0.97)); return v; }
        if (m=="tf")         { say("TF " + S("from_frame") + " -> " + S("to_frame")); return makePose(0.1,0,0.5,0,0,0); }
        // raw
        if (m=="publish")    { say("bola " + S("topic") + " <- " + reprValue(V("value"))); return Value::Nil(); }
        if (m=="call_service"){ say("sewa bulai " + S("name") + " args=" + reprValue(V("args"))); Value d=Value::Dict_(); d.dict->push_back({"success",Value::Bool_(true)}); return d; }
        if (m=="list_nodes") { say("nodes: /sim_node"); Value v=Value::List_(); v.list->push_back(Value::Str_("/sim_node")); return v; }
        if (m=="list_topics"){ say("topics: /cmd_vel, /joint_states, /scan"); return Value::List_(); }
        if (m=="record_start"){ say("record shuru -> " + S("bag")); return Value::Nil(); }
        if (m=="record_stop"){ say("record band"); return Value::Nil(); }
        if (m=="launch")     { say("launch " + S("package") + " " + S("launch_file")); return Value::Nil(); }
        if (m=="urdf_load")  { say("urdf load: " + S("path")); return Value::Nil(); }
        // control
        if (m=="joint_to")   { say("joint '" + S("name") + "' ko " + fmtf("%.3f", N("value")) + " " + S("unit") + " pe le ja raha hai"); return Value::Nil(); }
        if (m=="controller_switch"){ say("controller switch: " + S("name")); return Value::Nil(); }
        if (m=="torque")     { say(std::string("torque ") + (B("on")?"on":"off")); return Value::Nil(); }
        // simulation
        if (m=="sim_start")  { say("Gazebo shuru: world '" + S("world") + "'"); return Value::Nil(); }
        if (m=="spawn")      { emit("[nakli] spawn '" + S("model") + "' at (" + fmtf("%.2f",N("x")) + ", " + fmtf("%.2f",N("y")) + ", " + fmtf("%.2f",N("z")) + ")"); return Value::Nil(); }
        if (m=="sim_stop")   { say("simulation band"); return Value::Nil(); }
        if (m=="rviz_open")  { say("RViz khol diya"); return Value::Nil(); }
        // safety
        if (m=="estop")      { say("!! BAND KARO -- sab kuch ruk gaya (e-stop) !!"); return Value::Nil(); }
        if (m=="set_workspace_limit"){ say("workspace limit set: " + reprValue(V("bounds"))); return Value::Nil(); }
        if (m=="set_speed_limit"){ say("speed limit set: " + fmtf("%.2f", N("v"))); return Value::Nil(); }
        // drone
        if (m=="takeoff")    { say("udaan bhari -- drone upar"); return Value::Nil(); }
        if (m=="land")       { say("utar raha hai (land)"); return Value::Nil(); }
        if (m=="set_height") { say("height " + fmtf("%.2f", N("h")) + " " + S("unit") + " pe ja raha hai"); return Value::Nil(); }

        throw RCError("anjaan hukum '" + m + "'", "unknown command", n->line);
    }
};
