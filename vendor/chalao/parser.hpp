// Robot Chalao -- recursive-descent parser (native C++ core). Tokens -> AST.
// Har Hinglish robot hukum ek RobotCommand(method,args,selector) banta hai.
#pragma once
#include <string>
#include <vector>
#include <set>
#include <memory>
#include "ast.hpp"
#include "lexer.hpp"

class Parser {
    std::vector<Token> toks; size_t pos = 0;
    const std::set<std::string> TIME  {"second","seconds","minute","minutes","ms"};
    const std::set<std::string> ANGLE {"degree","degrees","rad","radian","radians"};
    const std::set<std::string> DIST  {"meter","meters","metre","cm","mm"};
    const std::set<std::string> QUERY {"kahan","joints","battery","TF","object","IK","FK",
        "camera","lidar","depth","imu","dabav","obstacle","AprilTag","ArUco","param"};
public:
    explicit Parser(std::vector<Token> t) : toks(std::move(t)) {}

    NodePtr parse() {
        auto prog = mk(NT::Program);
        skipNL();
        while (cur().type != TT::Eof) { prog->kids.push_back(statement()); skipNL(); }
        return prog;
    }
private:
    const Token& cur() const { return toks[pos]; }
    const Token& nxt(int k = 1) const { size_t j = pos + (size_t)k; return j < toks.size() ? toks[j] : toks.back(); }
    int lineno() const { return cur().line; }
    bool isKw(const char* t) const { return cur().type == TT::Ident && cur().value == t; }
    bool isOp(const char* v) const { return cur().type == TT::Op && cur().value == v; }
    Token eat() { return toks[pos++]; }

    std::string eatKw(const char* t) {
        if (!isKw(t)) throw RCError(std::string("yahan '") + t + "' hona chahiye tha",
                                    std::string("expected '") + t + "'", lineno());
        return eat().value;
    }
    std::string eatOp(const char* v) {
        if (!isOp(v)) throw RCError(std::string("yahan '") + v + "' hona chahiye tha",
                                    std::string("expected '") + v + "'", lineno());
        return eat().value;
    }
    std::string eatIdent() {
        if (cur().type != TT::Ident) throw RCError("yahan ek naam hona chahiye tha", "expected an identifier", lineno());
        return eat().value;
    }
    std::string eatString() {
        if (cur().type != TT::String) throw RCError("yahan \"...\" (string) hona chahiye tha", "expected a string", lineno());
        return eat().value;
    }
    void expectNL() {
        if (cur().type == TT::Newline) eat();
        else if (cur().type == TT::Eof) return;
        else throw RCError("ek line par ek hi hukum -- yahan line khatam honi chahiye", "expected end of line", lineno());
    }
    void skipNL() { while (cur().type == TT::Newline) eat(); }

    NodePtr S(const std::string& s, int ln) { auto n = mk(NT::Str, ln); n->s = s; return n; }
    NodePtr Bn(bool v, int ln) { auto n = mk(NT::Bool, ln); n->bval = v; return n; }

    std::vector<NodePtr> block(const std::set<std::string>& terms) {
        std::vector<NodePtr> stmts;
        while (true) {
            skipNL();
            const Token& t = cur();
            if (t.type == TT::Eof) throw RCError("block poora nahi hua; 'khatam' bhool gaye?", "block not closed with 'khatam'", t.line);
            if (t.type == TT::Ident && terms.count(t.value)) break;
            stmts.push_back(statement());
        }
        return stmts;
    }

    NodePtr statement() {
        skipNL();
        const Token& t = cur();
        NodePtr node;
        if (t.type == TT::Ident) {
            const std::string& kw = t.value;
            if (kw == "maano")       node = letStmt();
            else if (kw == "agar")   { node = ifStmt();    expectNL(); return node; }
            else if (kw == "jab")    { node = whileStmt(); expectNL(); return node; }
            else if (kw == "har")    { node = harStmt();   expectNL(); return node; }
            else if (kw == "kaam")   { node = funcDef();   expectNL(); return node; }
            else if (kw == "koshish"){ node = tryStmt();   expectNL(); return node; }
            else if (kw == "sun")    { node = subscribeStmt(); expectNL(); return node; }
            else if (kw == "action") { node = actionStmt();    expectNL(); return node; }
            else if (kw == "dikhao") node = printStmt();
            else if (kw == "wapas")  node = returnStmt();
            else if (kw == "import") { eat(); if (cur().type == TT::String) eat(); node = mk(NT::ExprStmt, t.line); node->a = mk(NT::Num, t.line); node->a->num = 0.0; }
            else if (kw == "ruko_loop") { eat(); node = mk(NT::Break, t.line); }
            else {
                node = robotCommand();
                if (!node) { node = mk(NT::ExprStmt, t.line); node->a = expression(); }
            }
        } else {
            node = mk(NT::ExprStmt, t.line); node->a = expression();
        }
        expectNL();
        return node;
    }

    NodePtr letStmt() {
        int ln = lineno(); eatKw("maano");
        auto n = mk(NT::Let, ln); n->s = eatIdent(); eatOp("="); n->a = expression(); return n;
    }
    NodePtr printStmt() {
        int ln = lineno(); eatKw("dikhao");
        auto n = mk(NT::Print, ln); n->kids.push_back(expression());
        while (isOp(",")) { eat(); n->kids.push_back(expression()); }
        return n;
    }
    NodePtr returnStmt() {
        int ln = lineno(); eatKw("wapas");
        auto n = mk(NT::Return, ln);
        if (cur().type != TT::Newline && cur().type != TT::Eof) n->a = expression();
        return n;
    }
    NodePtr ifStmt() {
        int ln = lineno(); eatKw("agar");
        auto n = mk(NT::If, ln); n->a = expression(); eatKw("toh");
        n->kids = block({"warna","khatam"});
        if (isKw("warna")) { eat(); n->kids2 = block({"khatam"}); }
        eatKw("khatam"); return n;
    }
    NodePtr whileStmt() {
        int ln = lineno(); eatKw("jab"); eatKw("tak");
        auto n = mk(NT::While, ln); n->a = expression(); eatKw("karo");
        n->kids = block({"khatam"}); eatKw("khatam"); return n;
    }
    NodePtr harStmt() {
        int ln = lineno(); eatKw("har");
        if (cur().type == TT::Number && nxt().type == TT::Ident && TIME.count(nxt().value)) {
            auto n = mk(NT::Timer, ln); n->a = factor(); n->s = eatIdent(); eatKw("mein");
            n->kids = block({"khatam"}); eatKw("khatam"); return n;
        }
        auto n = mk(NT::ForEach, ln); n->s = eatIdent(); n->a = expression();
        eatKw("mein"); eatKw("karo"); n->kids = block({"khatam"}); eatKw("khatam"); return n;
    }
    NodePtr funcDef() {
        int ln = lineno(); eatKw("kaam");
        auto n = mk(NT::FuncDef, ln); n->s = eatIdent(); eatOp("(");
        if (!isOp(")")) { n->params.push_back(eatIdent()); while (isOp(",")) { eat(); n->params.push_back(eatIdent()); } }
        eatOp(")"); n->kids = block({"khatam"}); eatKw("khatam"); return n;
    }
    NodePtr tryStmt() {
        int ln = lineno(); eatKw("koshish");
        auto n = mk(NT::Try, ln); n->kids = block({"galti"});
        eatKw("galti"); eatKw("hone"); eatKw("par");
        if (cur().type == TT::Ident && cur().value != "khatam" && nxt().type == TT::Newline) { n->hasErr = true; n->errName = eatIdent(); }
        n->kids2 = block({"khatam"}); eatKw("khatam"); return n;
    }
    NodePtr subscribeStmt() {
        int ln = lineno(); eatKw("sun");
        auto n = mk(NT::Subscribe, ln); n->a = expression();
        n->s = (cur().type == TT::String) ? eatString() : eatIdent();   // type name in s
        eatKw("mein"); n->b = mk(NT::Ident, ln); n->b->s = eatIdent();  // var in b->s
        n->kids = block({"khatam"}); eatKw("khatam"); return n;
    }
    NodePtr actionStmt() {
        int ln = lineno(); eatKw("action"); eatKw("bhejo");
        auto n = mk(NT::ActionSend, ln); n->a = expression(); n->b = expression();
        if (isKw("progress")) { eat(); eatKw("mein"); n->hasErr = true; n->errName = eatIdent(); n->kids = block({"khatam"}); }
        else skipNL();
        eatKw("khatam"); return n;
    }

    // ---- robot command args ----
    std::vector<NodePtr> tuple(int need = -1) {
        eatOp("("); std::vector<NodePtr> items; items.push_back(expression());
        while (isOp(",")) { eat(); items.push_back(expression()); }
        eatOp(")");
        if (need >= 0 && (int)items.size() != need)
            throw RCError("yahan " + std::to_string(need) + " cheezein honi chahiye thi", "wrong number of values in ( )", lineno());
        return items;
    }
    std::string unitAfter(const std::set<std::string>& allowed, const char* def) {
        if (cur().type == TT::Ident && allowed.count(cur().value)) return eat().value;
        return def;
    }
    NodePtr RC(const char* method, int ln) { auto n = mk(NT::RobotCommand, ln); n->s = method; return n; }
    void arg(NodePtr& n, const char* k, NodePtr v) { n->cargs.push_back({k, v}); }

    NodePtr robotCommand() {
        const Token& t = cur(); std::string w = t.value; int ln = t.line;

        if (w == "robot") {
            eat();
            if (isKw("jodo")) { eat(); auto n = RC("connect", ln); arg(n, "name", S(eatString(), ln)); return n; }
            if (isKw("chhodo")) { eat(); return RC("disconnect", ln); }
            std::string sel = eatString();
            NodePtr inner = robotCommand();
            if (!inner || inner->t != NT::RobotCommand)
                throw RCError("robot \"" + sel + "\" ke baad ek hukum hona chahiye", "expected a command after a robot selector", ln);
            inner->hasSelector = true; inner->selector = sel; return inner;
        }
        if (w == "namespace") { eat(); auto n = RC("set_namespace", ln); arg(n, "ns", S(eatString(), ln)); return n; }
        if (w == "param") {
            eat();
            if (isKw("set")) { eat(); auto n = RC("param_set", ln); arg(n,"key",S(eatString(),ln)); arg(n,"value",expression()); return n; }
            eatKw("get"); auto n = RC("param_get", ln); arg(n,"key",S(eatString(),ln)); return n;
        }
        // arm
        if (w == "ghar") { eat(); eatKw("jao"); return RC("arm_home", ln); }
        if (w == "jao") {
            eat(); NodePtr e = expression();
            if (e->t == NT::TypedCtor && e->s == "joints") { auto n = RC("arm_joints", ln); arg(n,"joints",e); return n; }
            auto n = RC("arm_pose", ln); arg(n,"pose",e); return n;
        }
        if (w == "named") { eat(); eatKw("pose"); auto n = RC("arm_named", ln); arg(n,"name",S(eatString(),ln)); return n; }
        if (w == "seedha") { eat(); eatKw("jao"); auto v = tuple(3); auto n = RC("arm_cartesian", ln); arg(n,"x",v[0]); arg(n,"y",v[1]); arg(n,"z",v[2]); return n; }
        if (w == "pakdo") { eat(); auto n = RC("gripper", ln); arg(n,"close",Bn(true,ln)); return n; }
        if (w == "chhodo") { eat(); auto n = RC("gripper", ln); arg(n,"close",Bn(false,ln)); return n; }
        if (w == "speed") { eat(); if (isKw("limit")) { eat(); auto n = RC("set_speed_limit", ln); arg(n,"v",expression()); return n; } auto n = RC("set_speed", ln); arg(n,"scale",expression()); return n; }
        if (w == "planner") { eat(); auto n = RC("set_planner", ln); arg(n,"planner",S(eatString(),ln)); return n; }
        if (w == "rukawat") {
            eat();
            if (isKw("jodo")) {
                eat(); auto name = S(eatString(), ln); auto p = tuple(3); eatKw("size"); auto sz = tuple(3);
                auto pose = mk(NT::TypedCtor, ln); pose->s = "pose"; pose->kids = {p[0],p[1],p[2]};
                auto size = mk(NT::ListLit, ln); size->kids = {sz[0],sz[1],sz[2]};
                auto n = RC("add_obstacle", ln); arg(n,"name",name); arg(n,"pose",pose); arg(n,"size",size); return n;
            }
            eatKw("hatao"); auto n = RC("remove_obstacle", ln); arg(n,"name",S(eatString(),ln)); return n;
        }
        if (w == "attach") { eat(); auto n = RC("attach", ln); arg(n,"name",S(eatString(),ln)); return n; }
        if (w == "detach") { eat(); auto n = RC("detach", ln); arg(n,"name",S(eatString(),ln)); return n; }
        if (w == "kahan") { eat(); eatKw("hai"); return RC("get_pose", ln); }
        if (w == "joints" && nxt().type == TT::Ident && nxt().value == "kya") { eat(); eatKw("kya"); eatKw("hai"); return RC("get_joints", ln); }
        if (w == "IK") { eat(); eatKw("nikalo"); auto n = RC("ik", ln); arg(n,"pose",expression()); return n; }
        if (w == "FK") { eat(); eatKw("nikalo"); auto n = RC("fk", ln); arg(n,"joints",expression()); return n; }
        // base
        if (w == "aage") { eat(); eatKw("chalo"); auto d = expression(); std::string u = unitAfter(DIST,"meter"); auto n = RC("base_move", ln); arg(n,"distance",d); arg(n,"unit",S(u,ln)); arg(n,"backward",Bn(false,ln)); return n; }
        if (w == "peeche") { eat(); eatKw("chalo"); auto d = expression(); std::string u = unitAfter(DIST,"meter"); auto n = RC("base_move", ln); arg(n,"distance",d); arg(n,"unit",S(u,ln)); arg(n,"backward",Bn(true,ln)); return n; }
        if (w == "ghumo") { eat(); auto a = expression(); std::string u = unitAfter(ANGLE,"degree"); auto n = RC("base_rotate", ln); arg(n,"angle",a); arg(n,"unit",S(u,ln)); return n; }
        if (w == "speed_move") { eat(); auto v = expression(); auto wv = expression(); auto n = RC("base_twist", ln); arg(n,"v",v); arg(n,"w",wv); return n; }
        if (w == "ruk") { eat(); eatKw("jao"); return RC("base_stop", ln); }
        if (w == "map") {
            eat();
            if (isKw("load")) { eat(); auto n = RC("map_load", ln); arg(n,"path",S(eatString(),ln)); return n; }
            if (isKw("save")) { eat(); auto n = RC("map_save", ln); arg(n,"name",S(eatString(),ln)); return n; }
            eatKw("banao"); return RC("slam_start", ln);
        }
        if (w == "localize") { eat(); if (isOp("(")) { auto v = tuple(3); auto pose = mk(NT::TypedCtor, ln); pose->s="pose"; pose->kids={v[0],v[1],v[2]}; auto n = RC("localize", ln); arg(n,"pose",pose); return n; } return RC("localize", ln); }
        if (w == "yahan") { eat(); eatKw("jao"); auto v = tuple(3); auto n = RC("nav_to", ln); arg(n,"x",v[0]); arg(n,"y",v[1]); arg(n,"theta",v[2]); return n; }
        if (w == "waypoints") { eat(); eatKw("follow"); auto n = RC("follow_waypoints", ln); arg(n,"points",expression()); return n; }
        if (w == "obstacle") { eat(); eatKw("nazdeek"); eatKw("hai"); if (isOp("?")) eat(); return RC("obstacle_near", ln); }
        // perception
        if (w == "camera") { eat(); eatKw("dekho"); auto n = RC("camera_view", ln); arg(n,"topic",S(eatString(),ln)); return n; }
        if (w == "photo") { eat(); eatKw("lo"); auto n = RC("photo", ln); arg(n,"topic",S("",ln)); arg(n,"path",S(eatString(),ln)); return n; }
        if (w == "object") { eat(); eatKw("dhundo"); auto n = RC("find_object", ln); arg(n,"description",S(eatString(),ln)); return n; }
        if (w == "lidar" || w == "depth" || w == "imu" || w == "dabav") {
            eat(); eatKw("padho");
            const char* m = w=="lidar"?"read_lidar":w=="depth"?"read_depth":w=="imu"?"read_imu":"read_ft";
            return RC(m, ln);
        }
        if (w == "battery") { eat(); eatKw("kitni"); eatKw("hai"); return RC("battery", ln); }
        if (w == "AprilTag") { eat(); eatKw("dhundo"); return RC("find_apriltag", ln); }
        if (w == "ArUco") { eat(); eatKw("dhundo"); return RC("find_aruco", ln); }
        if (w == "TF") { eat(); eatKw("pucho"); auto frm = S(eatString(),ln); eatKw("se"); auto to = S(eatString(),ln); auto n = RC("tf", ln); arg(n,"from_frame",frm); arg(n,"to_frame",to); return n; }
        // raw
        if (w == "bolo") { eat(); auto topic = S(eatString(),ln); auto n = RC("publish", ln); arg(n,"topic",topic); arg(n,"value",expression()); return n; }
        if (w == "sewa") { eat(); eatKw("bulao"); auto name = S(eatString(),ln); NodePtr args; if (cur().type==TT::Newline||cur().type==TT::Eof){ args=mk(NT::DictLit,ln);} else args=expression(); auto n = RC("call_service", ln); arg(n,"name",name); arg(n,"args",args); return n; }
        if (w == "nodes") { eat(); eatKw("dikhao"); return RC("list_nodes", ln); }
        if (w == "topics") { eat(); eatKw("dikhao"); return RC("list_topics", ln); }
        if (w == "record") { eat(); if (isKw("shuru")) { eat(); auto n = RC("record_start", ln); arg(n,"bag",S(eatString(),ln)); return n; } eatKw("band"); return RC("record_stop", ln); }
        if (w == "launch") { eat(); auto pkg = S(eatString(),ln); auto n = RC("launch", ln); arg(n,"package",pkg); arg(n,"launch_file",S(eatString(),ln)); return n; }
        if (w == "urdf") { eat(); eatKw("load"); auto n = RC("urdf_load", ln); arg(n,"path",S(eatString(),ln)); return n; }
        // control
        if (w == "joint") { eat(); auto name = S(eatString(),ln); eatKw("ko"); auto val = expression(); std::string u = unitAfter(ANGLE,"rad"); eatKw("pe"); eatKw("le"); eatKw("jao"); auto n = RC("joint_to", ln); arg(n,"name",name); arg(n,"value",val); arg(n,"unit",S(u,ln)); return n; }
        if (w == "controller") { eat(); eatKw("switch"); auto n = RC("controller_switch", ln); arg(n,"name",S(eatString(),ln)); return n; }
        if (w == "torque") { eat(); std::string on = eatIdent(); auto n = RC("torque", ln); arg(n,"on",Bn(on=="on",ln)); return n; }
        // simulation
        if (w == "simulation") { eat(); if (isKw("shuru")) { eat(); eatKw("gazebo"); auto n = RC("sim_start", ln); arg(n,"world",S(eatString(),ln)); return n; } eatKw("band"); return RC("sim_stop", ln); }
        if (w == "spawn") { eat(); auto model = S(eatString(),ln); auto v = tuple(3); auto n = RC("spawn", ln); arg(n,"model",model); arg(n,"x",v[0]); arg(n,"y",v[1]); arg(n,"z",v[2]); return n; }
        if (w == "rviz") { eat(); eatKw("kholo"); return RC("rviz_open", ln); }
        // safety & timing
        if (w == "band") { eat(); eatKw("karo"); return RC("estop", ln); }
        if (w == "workspace") { eat(); eatKw("limit"); auto items = tuple(); auto n = RC("set_workspace_limit", ln); auto lst = mk(NT::ListLit, ln); lst->kids = items; arg(n,"bounds",lst); return n; }
        if (w == "ruko") { eat(); auto secs = expression(); unitAfter(TIME,"second"); auto call = mk(NT::Call, ln); call->s = "_ruko"; call->kids = {secs}; auto es = mk(NT::ExprStmt, ln); es->a = call; return es; }
        if (w == "deadline") { eat(); auto secs = expression(); unitAfter(TIME,"second"); auto call = mk(NT::Call, ln); call->s = "_deadline"; call->kids = {secs}; auto es = mk(NT::ExprStmt, ln); es->a = call; return es; }
        // drone
        if (w == "udaan") { eat(); eatKw("bharo"); return RC("takeoff", ln); }
        if (w == "utro") { eat(); return RC("land", ln); }
        if (w == "height") { eat(); auto h = expression(); std::string u = unitAfter(DIST,"meter"); auto n = RC("set_height", ln); arg(n,"h",h); arg(n,"unit",S(u,ln)); return n; }

        return nullptr;
    }

    // ---- expressions ----
    NodePtr expression() { return orExpr(); }
    NodePtr orExpr() { auto n = andExpr(); while (isKw("ya")) { int ln=lineno(); eat(); auto b=mk(NT::BinOp,ln); b->s="ya"; b->a=n; b->b=andExpr(); n=b; } return n; }
    NodePtr andExpr() { auto n = notExpr(); while (isKw("aur")) { int ln=lineno(); eat(); auto b=mk(NT::BinOp,ln); b->s="aur"; b->a=n; b->b=notExpr(); n=b; } return n; }
    NodePtr notExpr() { if (isKw("nahi")) { int ln=lineno(); eat(); auto u=mk(NT::UnaryOp,ln); u->s="nahi"; u->a=notExpr(); return u; } return compare(); }
    NodePtr compare() {
        auto n = sum();
        while (cur().type==TT::Op && (cur().value=="=="||cur().value=="!="||cur().value=="<"||cur().value==">"||cur().value=="<="||cur().value==">=")) {
            int ln=lineno(); std::string op=eat().value; auto b=mk(NT::BinOp,ln); b->s=op; b->a=n; b->b=sum(); n=b;
        }
        return n;
    }
    NodePtr sum() { auto n=term(); while (cur().type==TT::Op && (cur().value=="+"||cur().value=="-")) { int ln=lineno(); std::string op=eat().value; auto b=mk(NT::BinOp,ln); b->s=op; b->a=n; b->b=term(); n=b; } return n; }
    NodePtr term() { auto n=factor(); while (cur().type==TT::Op && (cur().value=="*"||cur().value=="/"||cur().value=="%")) { int ln=lineno(); std::string op=eat().value; auto b=mk(NT::BinOp,ln); b->s=op; b->a=n; b->b=factor(); n=b; } return n; }

    NodePtr factor() {
        const Token& t = cur(); int ln = t.line;
        if (isOp("-")) { eat(); auto u=mk(NT::UnaryOp,ln); u->s="-"; u->a=factor(); return u; }
        if (t.type == TT::Number) { eat(); auto n=mk(NT::Num,ln); n->num=std::strtod(t.value.c_str(),nullptr); return n; }
        if (t.type == TT::String) { eat(); return S(t.value, ln); }
        if (isOp("(")) { eat(); auto n=expression(); eatOp(")"); return postfix(n); }
        if (isOp("[")) return postfix(listLit());
        if (isOp("{")) return postfix(dictLit());
        if (t.type == TT::Ident) {
            std::string w = t.value;
            if (w == "sach" || w == "jhooth") { eat(); return Bn(w=="sach", ln); }
            if ((w=="pose"||w=="twist"||w=="joints") && nxt().type==TT::Op && nxt().value=="(") {
                eat(); auto args=tuple(); auto n=mk(NT::TypedCtor,ln); n->s=w; n->kids=args; return postfix(n);
            }
            if (QUERY.count(w)) {
                size_t save = pos;
                NodePtr cmd = robotCommand();
                if (cmd && cmd->t == NT::RobotCommand) return cmd;
                pos = save;   // not a query after all
            }
            if (nxt().type == TT::Op && nxt().value == "(") {
                eat(); eatOp("("); auto n=mk(NT::Call,ln); n->s=w;
                if (!isOp(")")) { n->kids.push_back(expression()); while (isOp(",")) { eat(); n->kids.push_back(expression()); } }
                eatOp(")"); return postfix(n);
            }
            eat(); auto n=mk(NT::Ident,ln); n->s=w; return postfix(n);
        }
        throw RCError("yeh samajh nahi aaya", "unexpected token '" + t.value + "'", ln);
    }
    NodePtr postfix(NodePtr n) {
        while (isOp(".")) { int ln=lineno(); eat(); auto m=mk(NT::Member,ln); m->a=n; m->s=eatIdent(); n=m; }
        return n;
    }
    NodePtr listLit() {
        int ln=lineno(); eatOp("["); auto n=mk(NT::ListLit,ln);
        if (!isOp("]")) { n->kids.push_back(expression()); while (isOp(",")) { eat(); if (isOp("]")) break; n->kids.push_back(expression()); } }
        eatOp("]"); return n;
    }
    NodePtr dictLit() {
        int ln=lineno(); eatOp("{"); auto n=mk(NT::DictLit,ln);
        if (!isOp("}")) {
            NodePtr k=expression(); eatOp(":"); NodePtr v=expression(); n->pairs.push_back({k,v});
            while (isOp(",")) { eat(); if (isOp("}")) break; k=expression(); eatOp(":"); v=expression(); n->pairs.push_back({k,v}); }
        }
        eatOp("}"); return n;
    }
};

inline NodePtr parseSource(const std::string& src) { return Parser(Lexer(src).tokenize()).parse(); }
