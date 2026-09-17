// Robot Chalao -- lexer (native C++ core). Source text -> tokens.
// Newlines are significant (end statements) but ignored inside ( [ {.
// `#` starts a comment. Keywords stay IDENT; the parser matches by text.
#pragma once
#include <string>
#include <vector>
#include <cctype>
#include <unordered_map>
#include "ast.hpp"

enum class TT { Number, String, Ident, Op, Newline, Eof };

struct Token {
    TT type; std::string value; int line, col;
};

class Lexer {
    std::string src; size_t i = 0; int line = 1, col = 1, depth = 0;
    std::vector<Token> out;
    char peek(int k = 0) const { size_t j = i + (size_t)k; return j < src.size() ? src[j] : '\0'; }
    char advance() { char c = src[i++]; if (c == '\n') { line++; col = 1; } else col++; return c; }
    void add(TT t, const std::string& v, int l, int c) { out.push_back({t, v, l, c}); }
public:
    explicit Lexer(std::string s) : src(std::move(s)) {}

    std::vector<Token> tokenize() {
        while (i < src.size()) {
            char ch = peek();
            if (ch == '#') { while (i < src.size() && peek() != '\n') advance(); continue; }
            if (ch == '\n') {
                int l = line, c = col; advance();
                if (depth > 0) continue;
                if (!out.empty() && out.back().type != TT::Newline) add(TT::Newline, "\\n", l, c);
                continue;
            }
            if (ch == ' ' || ch == '\t' || ch == '\r') { advance(); continue; }
            if (ch == '\\' && peek(1) == '\n') { advance(); advance(); continue; }
            int l = line, c = col;
            if (ch == '"') { out.push_back(str(l, c)); continue; }
            if (std::isdigit((unsigned char)ch) || (ch == '.' && std::isdigit((unsigned char)peek(1)))) {
                out.push_back(number(l, c)); continue;
            }
            if (std::isalpha((unsigned char)ch) || ch == '_') { out.push_back(ident(l, c)); continue; }
            std::string two; two += ch; two += peek(1);
            if (two == "==" || two == "!=" || two == "<=" || two == ">=") {
                advance(); advance(); add(TT::Op, two, l, c); continue;
            }
            if (std::string("()[]{}=<>+-*/%,:.?").find(ch) != std::string::npos) {
                advance();
                if (ch == '(' || ch == '[' || ch == '{') depth++;
                else if (ch == ')' || ch == ']' || ch == '}') depth = depth > 0 ? depth - 1 : 0;
                add(TT::Op, std::string(1, ch), l, c); continue;
            }
            throw RCError(std::string("yeh akshar '") + ch + "' samajh nahi aaya",
                          "unexpected character", l);
        }
        if (!out.empty() && out.back().type != TT::Newline) add(TT::Newline, "\\n", line, col);
        add(TT::Eof, "", line, col);
        return out;
    }
private:
    Token str(int l, int c) {
        advance(); std::string buf;
        while (true) {
            if (i >= src.size()) throw RCError("string band nahi hui (\" missing)", "unterminated string", l);
            char ch = advance();
            if (ch == '\\') {
                char n = (i < src.size()) ? advance() : '\0';
                switch (n) { case 'n': buf += '\n'; break; case 't': buf += '\t'; break;
                             case '"': buf += '"'; break; case '\\': buf += '\\'; break;
                             default: buf += n; }
                continue;
            }
            if (ch == '"') break;
            buf += ch;
        }
        return {TT::String, buf, l, c};
    }
    Token number(int l, int c) {
        std::string buf; bool dot = false;
        while (i < src.size()) {
            char ch = peek();
            if (std::isdigit((unsigned char)ch)) buf += advance();
            else if (ch == '.' && !dot && std::isdigit((unsigned char)peek(1))) { dot = true; buf += advance(); }
            else break;
        }
        // scientific notation: e/E [ + | - ] digits  (e.g. 2.25e-05, 1E6)
        if ((peek() == 'e' || peek() == 'E') &&
            (std::isdigit((unsigned char)peek(1)) ||
             ((peek(1) == '+' || peek(1) == '-') && std::isdigit((unsigned char)peek(2))))) {
            buf += advance();                       // e / E
            if (peek() == '+' || peek() == '-') buf += advance();
            while (std::isdigit((unsigned char)peek())) buf += advance();
        }
        return {TT::Number, buf, l, c};
    }
    // Short aliases -> canonical keyword (aliases; full Hinglish still works).
    // Kept identical to the robot-chalao core so .rc services can be typed short.
    static const std::string& canonShort(const std::string& w) {
        static const std::unordered_map<std::string, std::string> m = {
            {"bas","khatam"}, {"bol","dikhao"}, {"rakh","maano"},
            {"try","koshish"}, {"de","wapas"}, {"tod","ruko_loop"},
        };
        auto it = m.find(w);
        return it == m.end() ? w : it->second;
    }
    Token ident(int l, int c) {
        std::string buf;
        while (i < src.size()) { char ch = peek(); if (std::isalnum((unsigned char)ch) || ch == '_') buf += advance(); else break; }
        return {TT::Ident, canonShort(buf), l, c};
    }
};
