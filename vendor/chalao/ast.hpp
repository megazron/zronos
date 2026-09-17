// Robot Chalao -- AST nodes and the Hinglish error type (native C++ core).
#pragma once
#include <string>
#include <vector>
#include <utility>
#include <memory>
#include <stdexcept>

// Hinglish error: `Bhai, line N: <msg> (<hint>)`
struct RCError : std::runtime_error {
    std::string msg_hi, hint_en; int line;
    RCError(std::string m, std::string h, int ln)
        : std::runtime_error(m), msg_hi(std::move(m)), hint_en(std::move(h)), line(ln) {}
    std::string what_hinglish() const {
        std::string s = "Bhai";
        if (line > 0) s += ", line " + std::to_string(line);
        s += ": " + msg_hi;
        if (!hint_en.empty()) s += " (" + hint_en + ")";
        return s;
    }
};

enum class NT {
    Program, Let, Print, Return, ExprStmt, If, While, ForEach, FuncDef, Try,
    Break, Timer, Subscribe, ActionSend,
    Num, Str, Bool, Ident, ListLit, DictLit, Call, Member, BinOp, UnaryOp,
    TypedCtor, RobotCommand
};

struct Node;
using NodePtr = std::shared_ptr<Node>;

struct Node {
    NT t;
    int line = 0;
    double num = 0.0;
    bool bval = false;
    std::string s;          // name / op / attr / kind / method / unit / var / errName / type_name
    NodePtr a, b, c;        // generic sub-expressions (cond/left/obj/operand/iterable/expr/period/topic/goal)
    std::vector<NodePtr> kids;   // block body / then / call args / list items
    std::vector<NodePtr> kids2;  // else / try-handler
    std::vector<std::string> params;
    std::vector<std::pair<NodePtr, NodePtr>> pairs;              // dict literal
    std::vector<std::pair<std::string, NodePtr>> cargs;          // robot command args (ordered)
    std::string selector; bool hasSelector = false;
    std::string errName;   bool hasErr = false;

    explicit Node(NT type, int ln = 0) : t(type), line(ln) {}
};

inline NodePtr mk(NT t, int ln = 0) { return std::make_shared<Node>(t, ln); }
