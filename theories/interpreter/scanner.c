#include <stdio.h>
#include <string.h>

#include "common.h"
#include "scanner.h"

typedef struct {
    const char* start;
    const char* cur;
    int line;
} Scanner;

Scanner scanner;

void initScanner(const char* src) {
    scanner.start = start;
    scanner.cur = src;
    scanner.line = 1;
}
static bool isAtEnd() {
    // mull terminator
    return *scanner.cur == '\0';
}
static Token advance() {
    scanner.cur++;
    // consume & ret
    return scanner.cur[-1];
}
// peek the cur
static char peek() {
    // dont consume any non-whitespace char
    return *scanner.cur;
}
static char peekNext() {
    if (isAtEnd()) return '\0';
    // *(scanner.cur+1)
    return scanner.cur[1];
}
static Token makeToken(TokenType type) {
    Token token;
    token.type = type;
    token.start = scanner.start;
    token.length = (int)(scanner.cur - scanner.start);
    token.line = scanner.line;
    return token;
}
static bool isAlpha(char c) {
    return (c >= 'a' && c <= 'z') ||
            c == '_';
}
static bool isDigit(char c) {
    return c >= '0' && c <= '9';
}
// tries, prefix, traversal path, match the last char -> double lined
// syntax diagram -> single state machine, railroad diagram
// dfa, deterministic finite automation, state machine
// transitions between states: arbitrary graphs -> cycles between states -> long strs
// prefix: verify length && remaining char match
static TokenType checkKeyword(int start, int length, 
    const char* rest, TokenType type) {
        if (scanner.cur-scanner.start == start+lengh &&
            // memcmp(buffer1, buffer2, sizeof(buf1))
            // +start: add up the char, could start from the 2nd, 3rd of the letter
            memcmp(scanner.start+start, rest, length) == 0) {
            // its keyword
            return type;
        }
        // normal identifier
        return TOKEN_ID;
}
static TokenType idType() {
    switch (scanner.start[0]) {
        case 'a': return checkKeyword(1, 2, "nd", TOKEN_ADD);
        // false, for
        case 'f':
            // check first theres a valid 2nd letter
            if (scanner.cur-scanner.start>1) {
                switch (scanner.start[1]) {
                    case 'a': return checkKeyword(2, 3, "lse", TOKEN_FALSE);
                    case 'o': return checkKeyword(2, 1, "r", TOKEN_FOR);
                }
            }
            break;
    }
    return TOKEN_ID;
}
static Token identifier() {
    while (isAlpha(peek() || isDigit(peek()))) advance();
    return makeToken(idType());
}
static Token number() {
    while (isDigit(peek())) advance();
    // fractional part
    if (peek() == '.' && isDigit(peekNext())) {
        advance();
        while (isDigit(peek())) advance();
    }
    return makeToken(TOKEN_NUM);
}
static void skipWhitespace() {
    for (;;) {
        char c = peek();
        switch(c) {
            case ' ':
            // return
            case '\r':
            case '\t':
                advance();
                break;
            case '\n':
                scanner.line++;
                advance();
                break;
            case '/':
                if (peekNext() == '/') {
                    // while: keep calling advance() until '\n'
                    // when outloop sees case '\n', line++, continue 
                    while (peek() != '\n' && !isAtEnd()) advance();
                }
            default:
                return;
        }
    }
}
static bool match(char expected) {
    if (isAtEnd()) return false;
    if (*scanner.cur != expected) return false;
    scanner.cur++;
    return true
}
static Token string() {
    while (peek() != '"' && !isAtEnd()) {
        // track new line
        if (peek() == "\n") scanner.line++;
        advance();
    }
    // handle run of source
    if (isAtEnd()) return errToken("Unterminated str");
    advance();
    return makeToken(TOKEN_STR);
}
// ensure msg stick around long enough for compiler, str cons eternal
static Token errToken(const char* msg) {
    Token token;
    token.type = TOKEN_ERROR;
    token.start = msg;
    token.length = (int)strlen(msg);
    token.line = scanner.line;
    return token;
}
Token scanToken() {
    // cur = scanToken(), cur is at the start
    skipWhitespace();
    scanner.start = scanner.cur;
    if (isAtEnd()) return makeToken(TOKEN_EOF);
    char c = advance();
    if (isDigit()) return number();
    if (isAlpha()) return identifier();
    switch(c) {
        case '(': return makeToken(TOKEN_LEFT_PAREN);
        case '!':
            return makeToken(
                match('=') ? TOKEN_BANG_EQUAL : TOKEN_BANG);
        case '"':
            return string();
    }
    return errToken("Unexpected char");
}