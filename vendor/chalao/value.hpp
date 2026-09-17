// Robot Chalao -- runtime values (native C++ core).
// Yeh woh cheezein hain jo ek .rc program ke andar ghoomti hain: numbers,
// strings, bools, lists, dicts, typed objects (Pose/Twist/Joints/Detection),
// aur user functions. Formatting yahin hai taaki `dikhao` aur sim output
// bilkul reference (Python) jaisa dikhe.
#pragma once
#include <string>
#include <vector>
#include <utility>
#include <memory>
#include <map>
#include <cstdio>
#include <cstdlib>
#include <cmath>

struct Node;   // AST function node (forward)
struct Env;    // closure scope (forward)
struct Obj;

enum class VT { Nil, Num, Bool, Str, List, Dict, Object, Func };

struct Value {
    VT t = VT::Nil;
    double num = 0.0;
    bool b = false;
    std::string str;
    std::shared_ptr<std::vector<Value>> list;
    std::shared_ptr<std::vector<std::pair<std::string, Value>>> dict;
    std::shared_ptr<Obj> obj;
    std::shared_ptr<Node> fn;                 // FuncDef node
    std::shared_ptr<Env> closure;

    static Value Nil()            { return Value{}; }
    static Value Num_(double d)   { Value v; v.t = VT::Num;  v.num = d; return v; }
    static Value Bool_(bool x)    { Value v; v.t = VT::Bool; v.b = x;  return v; }
    static Value Str_(std::string s){ Value v; v.t = VT::Str; v.str = std::move(s); return v; }
    static Value List_() { Value v; v.t = VT::List; v.list = std::make_shared<std::vector<Value>>(); return v; }
    static Value Dict_() { Value v; v.t = VT::Dict; v.dict = std::make_shared<std::vector<std::pair<std::string,Value>>>(); return v; }
};

struct Obj {
    std::string type;                              // Pose / Twist / Joints / Detection / Image / PointCloud
    std::vector<std::pair<std::string, Value>> fields;   // ordered
    Value* find(const std::string& k) {
        for (auto& p : fields) if (p.first == k) return &p.second;
        return nullptr;
    }
};

// ---- number formatting ----------------------------------------------------
inline std::string shortestG(double d) {
    char buf[64];
    for (int p = 1; p <= 17; ++p) {
        std::snprintf(buf, sizeof buf, "%.*g", p, d);
        if (std::strtod(buf, nullptr) == d) break;
    }
    return std::string(buf);
}
inline std::string fmtNumDisplay(double d) {          // like Python _fmt(): 1.0 -> "1"
    if (d == std::floor(d) && std::fabs(d) < 1e15) {
        char b[32]; std::snprintf(b, sizeof b, "%lld", (long long) std::llround(d));
        return std::string(b);
    }
    return shortestG(d);
}

std::string reprValue(const Value& v);   // fwd (used by objRepr for nested)

inline bool truthy(const Value& v) {
    switch (v.t) {
        case VT::Nil:    return false;
        case VT::Bool:   return v.b;
        case VT::Num:    return v.num != 0.0;
        case VT::Str:    return !v.str.empty();
        case VT::List:   return v.list && !v.list->empty();
        case VT::Dict:   return v.dict && !v.dict->empty();
        case VT::Object: return true;
        case VT::Func:   return true;
    }
    return false;
}

inline double gf(Obj& o, const char* k) { Value* p = o.find(k); return p ? p->num : 0.0; }

inline std::string objRepr(Obj& o) {
    char b[512];
    if (o.type == "Pose") {
        std::snprintf(b, sizeof b,
            "Pose(x=%.3f, y=%.3f, z=%.3f, roll=%.3f, pitch=%.3f, yaw=%.3f)",
            gf(o,"x"), gf(o,"y"), gf(o,"z"), gf(o,"roll"), gf(o,"pitch"), gf(o,"yaw"));
        return b;
    }
    if (o.type == "Twist") {
        std::snprintf(b, sizeof b, "Twist(v=(%.3f, %.3f, %.3f), w=(%.3f, %.3f, %.3f))",
            gf(o,"vx"), gf(o,"vy"), gf(o,"vz"), gf(o,"wx"), gf(o,"wy"), gf(o,"wz"));
        return b;
    }
    if (o.type == "Joints") {
        Value* vals = o.find("values");
        std::string s = "Joints([";
        if (vals && vals->t == VT::List) {
            for (size_t i = 0; i < vals->list->size(); ++i) {
                char n[32]; std::snprintf(n, sizeof n, "%.3f", (*vals->list)[i].num);
                s += n; if (i + 1 < vals->list->size()) s += ", ";
            }
        }
        return s + "])";
    }
    if (o.type == "Detection") {
        Value* lab = o.find("label"); Value* ps = o.find("pose"); Value* cf = o.find("confidence");
        std::string pr = (ps && ps->t == VT::Object) ? objRepr(*ps->obj) : "Pose()";
        char c[16]; std::snprintf(c, sizeof c, "%.2f", cf ? cf->num : 0.0);
        return "Detection(label='" + (lab ? lab->str : "") + "', " + pr + ", confidence=" + c + ")";
    }
    if (o.type == "Image") {
        Value* w=o.find("width"); Value* h=o.find("height"); Value* e=o.find("encoding"); Value* tp=o.find("topic");
        std::snprintf(b, sizeof b, "Image(%dx%d, encoding='%s', topic='%s')",
            (int)(w?w->num:0),(int)(h?h->num:0), e?e->str.c_str():"", tp?tp->str.c_str():"");
        return b;
    }
    if (o.type == "PointCloud") return "PointCloud(0 points)";
    return o.type + "(...)";
}

inline std::string fmtValue(const Value& v) {         // for `dikhao`
    switch (v.t) {
        case VT::Nil:   return "kuch nahi";
        case VT::Bool:  return v.b ? "sach" : "jhooth";
        case VT::Num:   return fmtNumDisplay(v.num);
        case VT::Str:   return v.str;
        case VT::Object:return objRepr(*v.obj);
        case VT::Func:  return "<kaam>";
        case VT::List: {
            std::string s = "["; for (size_t i=0;i<v.list->size();++i){ s+=reprValue((*v.list)[i]); if(i+1<v.list->size()) s+=", ";} return s+"]";
        }
        case VT::Dict: {
            std::string s = "{"; for (size_t i=0;i<v.dict->size();++i){ auto&p=(*v.dict)[i]; s+="'"+p.first+"': "+reprValue(p.second); if(i+1<v.dict->size()) s+=", ";} return s+"}";
        }
    }
    return "";
}

inline std::string reprValue(const Value& v) {        // for sim %r
    if (v.t == VT::Str)  return "'" + v.str + "'";
    if (v.t == VT::Num)  return shortestG(v.num);
    if (v.t == VT::Bool) return v.b ? "True" : "False";
    if (v.t == VT::Nil)  return "None";
    return fmtValue(v);
}

// ---- constructors for typed objects ---------------------------------------
inline Value makePose(double x,double y,double z,double r,double p,double yw){
    auto o=std::make_shared<Obj>(); o->type="Pose";
    o->fields={{"x",Value::Num_(x)},{"y",Value::Num_(y)},{"z",Value::Num_(z)},
               {"roll",Value::Num_(r)},{"pitch",Value::Num_(p)},{"yaw",Value::Num_(yw)}};
    Value v; v.t=VT::Object; v.obj=o; return v;
}
inline Value makeTwist(double vx,double vy,double vz,double wx,double wy,double wz){
    auto o=std::make_shared<Obj>(); o->type="Twist";
    o->fields={{"vx",Value::Num_(vx)},{"vy",Value::Num_(vy)},{"vz",Value::Num_(vz)},
               {"wx",Value::Num_(wx)},{"wy",Value::Num_(wy)},{"wz",Value::Num_(wz)}};
    Value v; v.t=VT::Object; v.obj=o; return v;
}
inline Value makeJoints(std::vector<double> vals){
    auto o=std::make_shared<Obj>(); o->type="Joints";
    Value lst=Value::List_(); for(double d:vals) lst.list->push_back(Value::Num_(d));
    o->fields={{"values",lst}};
    Value v; v.t=VT::Object; v.obj=o; return v;
}
inline Value makeDetection(const std::string& label, Value pose, double conf){
    auto o=std::make_shared<Obj>(); o->type="Detection";
    o->fields={{"label",Value::Str_(label)},{"pose",pose},{"confidence",Value::Num_(conf)}};
    Value v; v.t=VT::Object; v.obj=o; return v;
}
inline Value makeImage(int w,int h,const std::string& enc,const std::string& topic){
    auto o=std::make_shared<Obj>(); o->type="Image";
    o->fields={{"width",Value::Num_(w)},{"height",Value::Num_(h)},{"encoding",Value::Str_(enc)},{"topic",Value::Str_(topic)}};
    Value v; v.t=VT::Object; v.obj=o; return v;
}
