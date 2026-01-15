#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "common.h"
#include "compiler.h"
#include "scanner.h"
#include "mem.h"
#ifdef DEBUG_PRINT_CODE
#include "debug.h"
#endif

typedef struct {
    Token cur;
    Token prev;
    bool hadErr;
    bool panicMode;
} Parser;

// pratt parse rule: recursive descent parser, associate token w/ op-precedence val/func
// prefix: unary, bind as left-bound
// infix: left->lower precedence, right higher, num higher preced->tighter binding
typedef enum {
    PREC_NONE,
    PREC_ASSIGNMENT, // = 
    // or and < > ==
    PREC_TERM, // + -
    PREC_FACTOR, // *
    PREC_UNARY, // - !
    PREC_CALL, // . ()
    PREC_PRIMARY
} precedence;

// ParseFn: typedef no arg, ret void
typedef void (*ParseFn) (bool canAssign);

typedef struct {
    ParseFn prefix;
    ParseFn infix;
    Precedence precedence;
} ParseRule;

typedef enum {
    TYPE_FUNC,
    TYPE_SCRIPT,
    TYPE_INITIALIZER,
    TYPE_METHOD
} FuncType;

typedef struct {
    Token name;
    int depth;
    bool isCaptured;
} Local;

typedef struct {
    // upval needs to outlive the func lifetime
    uint8_t index;
    bool isLocal;
} Upval;

typedef struct Compiler {
    // outer compiler/func, link
    struct Compiler* enclosing;
    ObjFun* func;
    FuncType type;
    int localCnt;
    Local locals[UINT8_CNT];
    Upval upvals[UINT8_CNT];
    int scopeDepth;
} Compiler;
typedef struct ClassCompiler {
    struct ClassCompiler* enclosing;
    bool hasSuperclass;
} ClassCompiler;

// single global var of the struct
Parser parser;
Compiler* cur = NULL;
ClassCompiler* curClass = NULL;

staic void initCompiler(Compiler* compiler, FuncType type) {
    // begin a new func, then a inner func/local
    // update cur to new activated compiler \
        when finishes, compiler cur = cur->enclosing
    // copy: func obj outlives the compiler, persist until runtime \
            copyStr: findStr() in table, memcpy(), realloc()
    // use locals[] slot-0 for vm internal use, localCnt++ \
        type methods/func
    compiler->enclosing = cur;
    compiler->func = NULL;
    compiler->type = type;
    compiler->localCnt = 0;
    compiler->scopeDepth = 0;
    compiler->func = newFunc();
    cur = compiler;
    if (type != TYPE_SCRIPT) {
        cur->func->name = copyStr(parser.prev.start, parser.prev.length);
    }
    Local* local = &cur->locals[cur->localCnt++];
    local->depth = 0;
    local->isCaptured = false;
    if (type != TYPE_FUNC) {
        local->name.start = "this";
        local->name.length = 4;
    } else {
        local->name.start = "";
        local->name.length = 0;
    }
}
static void markCompilerRoots() {
    // garbage collection nested funcs \
        enclosing: go to the outer func
    Compiler* compiler = cur;
    while (compiler != NULL) {
        markObj((Obj*)compiler->func);
        compiler = compiler->enclosing;
    }
}
static Chunk* curChunk() {
    // Chunk* compilingChunk;
    // static Chunk* curChunk() {
    //     return compilingChunk;
    // }
    return &cur->func->chunk;
}
static void errAt(Token token, const char* msg) {
    // panic mode: not entirely exit, discard input until semicolon/brace, resume parsing
    // true: suppress subsequent/cascading errs
    // hadErr: skip execution, prevent bytecode execution post-err
    if (parser.panicMode) return;
    parser.panicMode = true;
    fprintf(stderr, "[line %d] Error", token->line);
    if (token->type == TOKEN_EOF) {
        fprintf(stderr, " at end");
    } else if (token->type == TOKEN_ERR) {
        // nothing
    } else {
        fprintf(stderr, " at '%.*s'", token->length, token->start);
    }
    fprintf(stderr, ": %s\n", msg);
    parser.hadErr = true;
}
static void err(const char* msg) {
    // report where just consumed
    errAt(&parser.prev, msg);
}
static void errAtCur(const char* msg) {
    errAt(&parser.cur, msg);
}
static void advance() {
    // cur -> prev before looping, cur++
    parser.prev = parser.cur;
    for (;;){
        parser.cur = scanToken();
        if (parser.cur.type != TOKEN_ERR) break;
        errAtCur(parser.cur.start);
    }
}
static bool check(TokenType type, type) {
    return parser.cur.type == type;
}
static bool match(TokenType type) {
    if (!check(type)) return false;
    advance();
    return true;
}
static void synchronize() {
    parser.panicMode = false;
    while (parser.cur.type != TOKEN_EOF) {
        if (parser.prev.type == TOKEN_SEMICOLON) return;
        switch (parser.cur.type) {
            case TOKEN_VAR:
            case TOKEN_FOR:
                return;
            default:
            // do nothing
                ;
        }
        advance();
    }
}
static void emitByte(uint8_t byte) {
    /* print 42 line 10*/
    // opcode: OP_CONSTANT 42 OP_PRINT writeChunk(chunk, byte, 10)
    // byte: opcode/operand instruction
    // prev.line: most recently parsed token, last step -> emit, for runtime err
    // chunk (cap, *code, *line, cnt, constants)
    // writeChunk(chunk, byte, line) write byte line to the chunk
    writeChunk(curChunk(), byte, parser.prev.line);
}
static void emitBytes(uint8_t byte1, uint8_t byte2) {
    emitByte(byte1);
    emitByte(byte2);
}
static void emitLoop(int loopStart) {
    // emit 0x01, jump back to start, 2 bytes for the jump offset itself
    // encode: store 16-bit as two 8-bit bytes to bytecode stream
    // later read back and reconstruct 16-bit
    /* 0x1234 -> high byte 0x12 & low byte 0xff = 0x12 
        -> 0x1234 & 0xff = 0x34*/
    // >> &: shift high byte to the right/down, discard the lowest 8-bit
    // &0xff: low byte, the lowest 8-bit
    // bytecode stream: [OP_LOOP, 0x12, 0x34]
    emitByte(OP_LOOP);
    int offset = curChunk()->cnt - loopStart + 2;
    if (offset > UINT16_MAX) err("Loop body too large");
    emitByte((offset >> 8) & 0xff);
    emitByte(offset & 0xff);
}
static int emitJump(uint8_t instruction) {
    // placeholder for two bytes offset operand
    // 16-bit offset jump up to 65,535 bytes
    emitByte(instruction);
    emitByte(0xff);
    emitByte(0xff);
    return curChunk()->cnt - 2;
}
static void emitRet() {
    // initializer: return this
    // null: return by the end of the body
    if (cur->type == TYPE_INITIALIZER) {
        emitBytes(OP_GET_LOCAL, 0);
    } else {
        emitByte(OP_NIL);
    }
    emitByte(OP_RET);
}
static ObjFunc* endCompiler() {
    // compiler create func itself and return
    // before interpret() pass in a chunk to be written to
    // cur go to the outer
    emitRet();
    ObjFunc* func = cur->func;
#ifdef DEBUG_PRINT_CODE
    if (!parser.hadErr) {
        disassembleChunk(curChunk(), func->name != NULL ? func->name->chars : "<script>");
    }
#endif
    cur = cur->enclosing;
    return func;
}

static void expr();
static void statement();
static void declaration();
static ParseRule* getRule(TokenType type);
static void parsePrecedence(Precedence precedence);

static uint8_t makeConstant(Value val) {
    // addConstant(): chunk->vm->compile
    // write val to the chunk array/constant table
    // max: 1 byte, 8-bit, 0-255, up to 256 constants in the chunk
    // OP_CONSTANT_16 store index as two-byte, scale to larger
    // return index, chunk->constants.cnt-1, if first, 0
    int constanttant = addConstant(curChunk(), val);
    if (constant > UINT8_MAX) {
        err("Too many constant in one chunk");
        return 0;
    }
    return (uint8_t)constant;
}
static void emitConstant(Value val) {
    emitBytes(OP_CONSTANT, makeConstant(val));
}
ParseRule rules[] = {
    // {type prefix/infix, op, precedence}
    [TOKEN_LEFT_PAREN]  = {grouping, call, PREC_CALL},
    [TOKEN_BANG]        = {unary, NULL, PREC_NONE},
    [TOKEN_BANG_EQUAL]  = {NULL, binary, PREC_EQUALITY},
    [TOKEN_MINUS]       = {unary, binary, PREC_TERM},
    [TOKEN_IDENTIFIER]  = {var, NULL, PREC_NONE},
    [TOKEN_STR]         = {str, NULL, PREC_NONE},
    [TOKEN_NUM]         = {num, NULL, PREC_NONE},
    [TOKEN_STAR]        = {NULL, binary, PREC_FACTOR},
    [TOKEN_AND]         = {NULL, and_, PREC_AND},
    [TOKEN_OR]         = {NULL, or_, PREC_OR},
    [TOKEN_FALSE]       = {literal, NULL, PREC_NONE},
    [TOKEN_ERR]         = {NULL, NULL, PREC_NONE},
}
static ParseRule* getRule(TokenType type){
    return &rules[type];
}
static void parsePrecedence(Precedence precedence) {
    // precedences in enum: numerically successively larger
    /* a * b = c + d */
    // a*b rval/expr res/* infix lack assignment, trailing = post-parse invalid, not (a * b)
    /* !true && false(parse PREC_OR) PREC(PREC_AND) <= PREC_OR */
    // prefix unary OP_NOT push true \
        loop less passPrecedence: infix OP_JUMP_IF_FALSE, OP_POP, literal push false, patch jump \
            no =, stack: false
    /* 1 = 2 invalid */
    // literals, not lval(left obj has address/identifiable location in mem to store data)
    /* x = 5 */ 
    // call var(true) set prefix for x, inside var, match =, parse/push 5, emit OP_SET_GLOBAL
    advance();
    ParseFn prefixRule = getRule(parser.prev.type)->prefix;
    if (prefixRule == NULL) {
        err("Expect expr");
        return;
    }
    bool canAssign = precedence <= PREC_ASSIGNMENT;
    prefixRule(canAssign);
    while (precedence <= getRule(parser.cur.type)->precedence) {
        advance();
        parseFn infixRule = getRule(parser.prev.type)->infix;
        infixRule(canAssign);
    }
    if (canAssign && match(TOKEN_EQUAL)) {
        err("Invalid assignment target.");
    }
}
static uint8_t identifierConstant(Token* name) {
    // method foo() name=constant "foo"
    // makeConstant: add to constant table return index
    // copyStr: findStr() in table, memcpy(), realloc()
    return makeConstant(OBJ_VAL(copyStr(name->start, name->length)));
}
static void expr(){
    parsePrecedence(PREC_ASSIGNMENT);
}
static void consume(TokenType type, const char* errMsg) {
    if (parser.cur.type != type) errAtCur(errMsg);
}
static void exprStatement() {
    // an expr followed by ;, evals expr and discards res
    expr();
    consume(TOKEN_SEMICOLON, "Expect ';' after expr");
    emitByte(OP_POP);
}
static void printStatement() {
    // stack effect: OP_ADD, pop 2 push 1, leaves 1 smaller \
        statement: x++, produces no val on the stack \
        expr: x>10, parsePrecedence, res stacktop, print
    expr();
    consume(TOKEN_SEMICOLON, "Expect ';' after expr");
    emitByte(OP_PRINT);
}
static void beginScope() {
    cur->scopeDepth++;
}
static endScope() {
    // end one layer of scope
    // if upval, closed->location, openupvals = upval->next
    // localCnt-1: index in locals
    /* {var a = 1; {var b = 2; {var c = 3; }}} */
    // outer: scope=0, a depth 0, b 1, inner: scope=3, c depth 3 \
        end: localCnt = 0 or locals[localCnt-1].depth > scope \
        while: cur->scope 2 localCnt=3 locals=[a, b, c]\
            c: locals[2].depth = 3 > scope 2, notcaptured pop, localCnt=2\
            b: locals[1].depth = 2 = scope 2, end\
            a:
    cur->scopeDepth--;
    while (cur->localCnt > 0 && cur->locals[cur->localCnt-1].depth > cur->scopeDepth) {
                if (cur->locals[cur->locCnt-1].isCaptured) {
                    emitByte(OP_CLOSE_UPVAL);
                } else {
                    emitByte(OP_POP);
                }
                cur->localCnt--;
            }
}
static void retStatement() {
    // method parse, ret, not initializer
    if (cur->type == TYPE_SCRIPT) {
        err(" cant ret from top-level code");
    }
    if (match(TOKEN_SEMICOLON)) {
        emitRet();
    } else {
        if (cur->type == TYPE_INITIALIZER) {
            err("Cant return a val from initializer");
        }
        expr();
        consume(TOKEN_SEMICOLON, "Expect ';' after ret val");
        emitByte(OP_RET);
    }
}
static void patchJump(int offset) {
    // 2-byte offset placeholder \
        -2: the jump itself takes 2 bytes \
        reconstruct 16-bit bytecode high byte & low byte
    /* OP_JUMP_IF_FALSE 0x00 0x00 ...(then skip) OP_RET(index 5) */
    // offset: 1, location of the first jump 0x00, cnt: 6\
        patch: 6 - 1 - 2 = 3 (0x0011) write code[0]=0x00, code[1]=0x11(then skip body) \
        bytecode: [OP_JUMP_IF_FALSE, 0x00, 0x11, ...(then skip), OP_RET]
    int jumpDis = curChunk()->cnt - offset - 2;
    if (jumpDis > UINT16_MAX) {
        err("Too much code to jump over");
    }
    curChunk()->code[offset] = (jumpDis >> 8) & 0xff;
    curChunk()->code[offset+1] = jumpDis & 0xff;
}
static void forStatement() {
    /* for (var i=0; i<10; i++) print(i) */
    // match(var): var i=0, loopstart after that cnt, mark the beginning, can jump back to \
        if not ;: i<10, parsePrecedence, placeholder jump if false, pop condition \
        not ): i++, emit jump over it, mark incrementStart new cnt, parsePrecedence, pop condition \
            till ), jump back to loop start, redefine loopstart to increment, \
            patch jump to body executes start, run increment after body \
            after emit loopstart logic, then patch jump \
        parse statement, compile print(i) \
        emitloop(loopstart): jump back to re-check condition \
        when ; ): exitJump != 1, patch jump skip body to end, pop condition
    beginScope();
    consume(TOKEN_LEFT_PAREN, "Expect '(' after 'for' ");
    if (match(TOKEN_SEMICOLON)) {
        // no initializer
    } else if (match(TOKEN_VAR)) {
        varDeclaration();
    } else {
        exprStatement();
    }
    int loopStart = curChunk()->cnt;
    int exitJump = -1;
    if (!match(TOKEN_SEMICOLON)) {
        expr();
        consume(TOKEN_SEMICOLON, "Expect ';' after condition ");
        exitJump = emitJump(OP_JUMP_IF_FALSE);
        emitByte(OP_POP);
    }
    if (!match(TOKEN_RIGHT_PAREN)) {
        int bodyJump = emitJump(OP_JUMP);
        int incrementStart = curChunk()->cnt;
        expr();
        emitByte(OP_POP);
        consume(TOKEN_RIGHT_PAREN, "Expect ')' after for ");
        emitLoop(loopStart);
        loopStart = incrementStart;
        patchJump(bodyJump);
    }
    statement();
    emitLoop(loopstart);
    if (exitJump != -1) {
        patchJump(exitJump);
        emitByte(OP_POP);
    }
    endScope();
}
static void ifStatement() {
    /* if (x>10) print "big"; else print "small" */
    // expr(): compile x>10 \
        emit if false then jump, pop condition true for then, parse then print "big" \
        -> else jump if condition true -> skip then if false patch jump, pop false for else, parse else print "small" -> else patch jump \
        'then jump/skip then if false' only truly completes after emitting 'else jump'/skip else-block 
    consume(TOKEN_LEFT_PAREN, "Expect '(' after 'if' ");
    expr();
    consume(TOKEN_RIGHT_PAREN, "Expect ')' after condition");
    int skipthenJump = emitJump(OP_JUMP_IF_FALSE);
    emitByte(OP_POP);
    statement();
    int skipelseJump = emitJump(OP_JUMP);
    patchJump(skipthenJump);
    emitByte(OP_POP);
    if (match(TOKEN_ELSE)) statement();
    patchJump(skipelseJump);
}
static void whileStatement() {
    // condition parse -> if false exit jump -> pop condition true -> parse then -> jump back to loopStart \
        patch jump false to exit -> pop condition false
    int loopStart = curChunk()->cnt;
    consume(TOKEN_LEFT_PAREN, "Expect '(' after 'while' ");
    expr();
    consume(TOKEN_RIGHT_PAREN, "Expect ')' after condition");
    int exitJump = emitJump(OP_JUMP_IF_FALSE);
    emitByte(OP_POP);
    statement();
    emitLoop(loopStart);
    patchJump(exitJump);
    emitByte(OP_POP);
}
static void synchronize() {
    parser.panicMode = false;
    while (parser.cur.type != TOKEN_EOF) {
        if (parser.prev.type == TOKEN_SEMICOLON) return;
        switch (parser.cur.type) {
            case TOKEN_CLASS:
            case TOKEN_RET:
                return;
            default:
            // do nothing
                ;
        }
        advance();
    }
}
static void and_(bool canAssign) {
    // logic AND 
    /* true AND print("hi") */
    // eval left, if false, skip right, jump to end \
        pop left, if jump, no need \
        eval print("hi"), push val \
        patch jump to here: if true, print, else skip
    int endJump = emitJump(OP_JUMP_IF_FALSE);
    emitByte(OP_POP);
    parsePrecedence(PREC_AND);
    patchJump(endJump);
}
static void or_(bool canAssign) {
    // if left is truthy, skip right, jump back and forth
    /* true || print("hi") */
    // eval/push true, doesnt trigger jump if false, endjump only
    /* false || print("hi") */
    // eval/push false, jump if false, pop false \
        eval/parse print push nil/res, patch jump after print/unused
    int elseJump = emitJump(OP_JUMP_IF_FALSE);
    int endJump = emitJump(OP_JUMP);
    patchJump(elseJump);
    emitByte(OP_POP);
    parsePrecedence(PREC_OR);
    patchJump(endJump);
}
static uint8_t argList() {
    // expr(): parsePrecedence()
    uint8_t argCnt = 0;
    if (!check(TOKEN_RIGHT_PAREN)) {
        do {
            expr();
            if (argCnt == 255) {
                err("Cant have more than 255 args");
            }
            argCnt++;
        } while (match(TOKEN_COMMA));
    }
    consume(TOKEN_RIGHT_PAREN, "Expect ')' after args");
    return argCnt;
}
static void call(bool canAssign) {
    uint8_t argCnt = argList();
    emitBytes(OP_CALL, argCnt);
}
static void dot(bool canAssign) {
    // property \
        methods & initializer parse-call identifierConstant: cpystr
    /* obj.prop = 42 -> equal, parsePrecedence parse/push 42, emit, pop val obj, set prop */
    /* obj.prop(1) -> invoke, consume prop/name, parse arg/argCnt=1, emit, pop obj args, push res*/
    /* obj.prop -> get property*/
    consume(TOKEN_IDENTIFIER, "expect property name after'.' ");
    uint8_t name = identifierConstant(&parer.prev);
    if (canAssign && match(TOKEN_EQUAL)) {
        expr();
        emitBytes(OP_SET_PROPERTY, name);
    } else if (match(TOKEN_LEFT_PAREN)) {
        uint8_t argCnt = argList();
        emitBytes(OP_INVOKE, name);
        emitByte(argCnt);
    } else {
        emitBytes(OP_GET_PROPERTY, name);
    }
}
static void markInitialized() {
    // var cuppa = "joe";
    //    1             2
    // 1->uninitialized
    // 2->initialized
    // avoid var a = "outer"; {var a = a;}
    // scope=0: global, else local depth = scope
    if (cur->scopeDepth == 0) return;
    cur->locals[cur->localCnt-1].depth = cur->scopeDepth;
}

static void defineVar(uint8_t global){
    // global var can be defined/emited after ref compiled
    /* fun showVar() {print global;} var global = "after"; showVar(); */
    /* var a = 1 + 2; var b = 4; */ 
    // scope>0: local, stack, 3 a OP_ADD
    if (cur->scopeDepth > 0) {
        markInitialized();
        return;
    }
    emitBytes(OP_DEFINE_GLOBAL, global);
}
static void block() {
    // recursively var func statement emit
    while (!check(TOKEN_RIGHT_BRACE) && !check(TOKEN_EOF)) {
        declaration();
    }
    consume(TOKEN_RIGHT_BRACE, "Expect '}' after block");
}
static void func(FuncType* type) {
    // if not ): params, declared in the outermost scope without initializer, like local var \
        parseVar declareVar addLocal(*name), declared in local scope, ret 0 dummy index \
        else global var method constant cpystr \
        defineVar: emit OP_GLOBAL, write chunk arr, markInitialized for local var
    // block: recursively var func statement emit till the end of the body
    // func: emit ret, create func
    // emit closure upvals, func->upvalCnt\
        makeConstant(): return the index from the obj into val in the constant table \
        emitBytes(): writeChunk() instruction opcode + operands 
    Compiler compiler;
    initCompiler(&compiler, type);
    beginScope();
    consume(TOKEN_LEFT_PAREN, "Expect '(' after func name");
    if (!check(TOKEN_RIGHT_PAREN)) {
        do {
            cur->func->arity++;
            if (cur->func->arity > 255) {
                errAtCur("Cant have more than 255 params");
            }
            uint8_t constant = parseVar("Expect param name");
            defineVar(constant);
        } while (match(TOKEN_COMMA));
    }
    consume(TOKEN_RIGHT_PAREN, "Expect ')' after param");
    consume(TOKEN_LEFT_BRACE, "Expect '{' before func body ");
    block();
    ObjFunc* func = endCompiler();
    emitBytes(OP_CLOSURE, makeConstant(OBJ_VAL(func)));
    for (int i=0; i<func->upvalCnt; i++) {
        emitByte(compiler.upvals[i].isLocal ? 1 : 0);
        emitByte(compiler.upvals[i].index);
    }
}
static void method() {
    // TOKEN_IDENTIFIER: var method instance class \
        type: method initializer/name init
    consume(TOKEN_IDENTIFIER, "Expect method name");
    uint8_t constant = identifierConstant(&parser.prev);
    FuncType type = TYPE_METHOD;
    if (parser.prev.length == 4 && memcmp(parser.prev.start, "init", 4) == 0) {
        type = TYPE_INITIALIZER;
    }
    func(type);
    emitBytes(OP_METHOD, constant);
}
static void classDeclaration() {
    // declareVar addLocal(*name), declared in local scope, ret 0 dummy index \
        else global var method constant cpystr \
        defineVar: emit OP_GLOBAL, write chunk arr, markInitialized for local var \
        var(false) cant assign local upval global, get not set \
        superclass defineVar(0) inherit itself \
        namedvar(): superclass var local upval global set get \
            loadclass method initializer \
        curClass goes to outer
    consume(TOKEN_IDENTIFIER, "Expect class name");
    Token className = parser.prev;
    uint8_t nameConstant = identifierConstant(&parser.prev);
    declareVar();
    emitBytes(OP_CLASS, nameConstant);
    defineVar(nameConstant);
    ClassCompiler classCompiler;
    classCompiler.hasSuperclass = false;
    classCompiler.enclosing = curClass;
    curClass = &classCompiler;
    if (match(TOKEN_LESS)) {
        consume(TOKEN_IDENTIFIER, "Expect superclass name");
        var(false);
        if (identifiersEqual(&className, &parser.prev)) {
            err("class cant inherit from itself");
        }
        beginScope();
        addLocal(syntheticToken("super"));
        defineVar(0);
        namedVar(className, false);
        emitByte(OP_INHERIT);
        classCompiler.hasSuperclass = true;
    }
    namedVar(className, false);
    consume(TOKEN_LEFT_BRACE, "Expect '{' before class body");
    while (!check(TOKEN_RIGHT_BRACE) && !check(TOKEN_EOF)) {
        method();
    }
    consume(TOKEN_RIGHT_BRACE, "Expect '}' after class body");
    emitByte(OP_POP);
    if (classCompiler.hasSuperclass) {
        endScope();
    }
    curClass = curClass->enclosing;
}
static void funcDeclaration() {
    // markInitialized for local var \
        func creates & recursively call func itself, mark it right away \
        defineVar: emit global, write chunk arr, 
    uint8_t global = parseVar("Expect func name");
    markInitialized();
    func(TYPE_FUNC);
    defineVar(global);
}
static void varDeclaration() {
    // global, if local mark initialized \
        expr(): = parsePrecedence, else emit null \
        defineVar(): emit global
    uint8_t global = parseVar("Expect var name");
    markInitialized();
    if (match(TOKEN_EQUAL)) {
        expr();
    } else {
        emitByte(OP_NIL);
    }
    consume(TOKEN_SEMICOLON, "Expect ';' after var declaration");
    defineVar(global);
}
static void declaration() {
    // class, func, var, statement
    if (match(TOKEN_FUNC)) {
        funcDeclaration();
    } (match(TOKEN_CLASS)) {
        classDeclaration();
    } else if (match(TOKEN_VAR)) {
        varDeclaration();
    } else {
        statement();
    }
    if (parser.panicMode) synchronize();
}
static void statement() {
    // expr =, for, if, print, ret, while
    if (match(TOKEN_PRINT)) {
        printStatement();
    } else if (match(TOKEN_IF)) {
        ifStatement();
    } else if (match(TOKEN_WHILE)) {
        whileStatement();
    } else if (match(TOKEN_FOR)) {
        forStatement();
    } else if (match(TOKEN_LEFT_BRACE)) {
        beginScope();
        block();
        endScope();
    } else if (match(TOKEN_RET)) {
        retStatement();
    } else {
        exprStatement();
    }
}
static void identifierEqual(Token* a, Token* b) {
    if (a->length != b->length) return false;
    return memcmp(a->start, b->start, a->length) == 0;
}
static void addLocal(Token* name) {
    // local arr locals[] mirrors runtime locals the stack slot indexes
    // decare depth undefine: uninitialized state, local->depth = cur->scopeDepth;
    if (cur->localCnt == UINT8_CNT) {
        err("Too many local vars in func");
        return;
    }
    Local* local = &cur->locals[cur->localCnt++];
    local->name = name;
    local->depth = -1;
    local->isCaptured = false;
}
static void declareVar() {
    // for local: remember the var exists, put in locals[]
    // top level, then append to arr, from arr end backwards
    // local var can have same name, as long as diff scopes
    // !=-1: initialized, active local var
    // < scope: not equal, outer scope, enclosing
    if (cur->scopeDepth == 0) return;
    Token* name = &parser.prev;
    for (int i = cur->localCnt-1; i>=0; i--) {
        Local* local = &cur->locals[i];
        if (local->depth != -1 && local->depth < cur->scopeDepth) {
            break;
        }
        if (identifiersEqual(name, &local->name)) {
            err("Already a var with this name in this scope");
        }
    }
    addLocal(*name);
}
static void parseVar(const char* errMsg ) {
    // consume: errMsg
    // declareVar: addLocal(*name)
    // >0: declared in local scope, no need to store in constant table, ret 0 dummy index \
        else global var method constant cpystr
    consume(TOKEN_ID, errMsg);
    declareVar();
    if (cur->scopeDepth > 0) return 0;
    return identifierConstant(&parser.prev);
}
static uint8_t resolveLocal(Compiler* compiler, Token* name) {
    // look thru the locals in scope
    // same name as identifier var ref token
    // i-- backwards: find the last declared
    for (int i=compiler->localCnt-1; i>=0; i--) {
        Local* local = &compiler->locals[i];
        if (identifierEqual(name, &local->name)) {
            // -1: ref to a var in its own initializer
            if (local->depth == -1) {
                err("Cant read local var in its own initializer");
            }
            // found it
            return i;
        }
    }
    // not found
    return -1;
}
staic int addUpval(Compiler* compiler, uint8_t index, bool isLocal) {
    // if indexes in compiler arr compiler->upvals[i] matches \
        ObjClosure runtime index locals[] in stack slot \
        else create upvals, cnt++
    int upvalCnt = compiler->func->upvalCnt;
    for (int i=0; i<upvalCnt; i++) {
        Upval* upval = &compiler->upvals[i];
        if (upval->index == index && upval->isLocal == isLocal) {
            return i;
        }
    }
    if (upvalCnt == UINT8_CNT) {
        err('Too many closure vars in func');
        return 0;
    }
    compiler->upvals[upvalCnt].isLocal = isLocal;
    compiler->upvals[upvalCnt].index = index;
    return compiler->func->upvalCnt++;
}
static uint8_t resolveUpval(Compiler* compiler, Token* name) {
    // each var ref resolves as local upval global, each func has a stack frame w/ locals
    // look for a matching local var in the enclosing func \
        when found, capture true, create an upval point to/indexed at local
    // recursively call on the enclosing compiler \
        until it finds local var ret upvalcnt++ \
        or run out of compilers ret -1
    if (local != -1) {
        compiler->enclosing->locals[local].isCaptured = true;
        return addUpval(compiler, (uint8_t)local, true);
    }
    int upval = resolveUpval(compiler->enclosing, name);
    if (upval != -1) {
        return addUpval(compiler, (uint8_t)upval, false);
    }
    return -1;
}
static void namedVar(Token name, bool canAssign) {
    /* var menu.brunch(sunday).beverage = "mimosa" */ 
    //      -get/target expr-   -set-      -val-
    // resolveLocal() ret the found given name in local var, local \
        else ret found upval, upval\
        else cpystr, global
    // emit for var access, load \
        if =, parsePrecedence(), emit set \
        else emit get
    uint8_t getOp, setOp;
    uint8_t arg = resolveLocal(cur, &name);
    if (arg != -1) {
        getOp = OP_GET_LOC;
        setOp = OP_SET_LOC;
    } else if ((arg = resolveUpval(cur, &name)) != -1) {
        getOp = OP_GET_UPVAL;
        setOp = OP_SET_UPVAL;
    }
    else {
        arg = identifierConstant(&name);
        getOp = OP_GET_GLOBAL;
        setOp = OP_SET_GLOBAL;
    }
    if (canAssign && match(TOKEN_EQUAL)) {
        expr();
        emitBytes(setOp, (uint8_t)arg);
    } else {
        emitBytes(getOp, (uint8_t)arg);
    }
}
static void var(bool canAssign) {
    namedVar(parser.prev, canAssign);
}
static Token syntheticToken(const char* text) {
    Token token;
    token.start = text;
    token.length = (int)strlen(text);
    return token;
}
static void super_(bool canAssign) {
    // _: avoid keyword 'super' in c
    // method name: identifierConstant() cpystr \
        namedVar: local upval global, cant assign, emit byte
    /* B extends A: super.foo(42) */
    // consume .fool, name=constant "foo", load this/b instance on stack \
        match (, parse/push 42, load superclass on stack, emit ip_invoke_super "foo", argCnt 1 \
            pop argCnt, then this receiver/args, then superclass obj \
                bind/invoke/call A.foo(this, 42), push res
    if (curClass = NULL) {
        err("Cant use 'super' outside of a class");
    } else if (!currentClass->hasSuperclass) {
        err("Cant have 'super' in a class w/o superclass");
    }
    consume(TOKEN_DOT, "Expect '.' after 'super' ");
    consume(TOKEN_IDENTIFIER, "Expect superclass method name");
    uint8_t name = identifierConstant(&parser.prev);
    namedVar(syntheticToken("this"), false);
    if (match(TOKEN_LEFT_PAREN)) {
        uint8_t argCnt = argList();
        namedVar(syntheticToken("super"), false);
    } else {
        namedVar(syntheticToken("super"), false);
        emitBytes(OP_GET_SUPER, name);
    }
}
static void this_(bool canAssign) {
    // cant assigned namedVar local upval global, get not set
    if (curClass == NULL) {
        err("Cant use 'this' outside of a class");
        return;
    }
    var(false);
}
static void num(bool canAssign) {
    // consume TOKEN_NUM token in scanner \
        look up & call num() in func ptr arr to compile
    // discard whitespaces, interpret a floating-point val/double in a byte str \
        str to double, endptr null, dont care about where num ends
    double val = strtod(parser.prev.start, NULL);
    emitConstant(NUM_VAL(val));
}
static void str(bool canAssign) {
    // +1 -2: trim the quotation
    emitConstant(OBJ_VAL(copyStr(parser.prev.start+1, parser.prev.length-2)));
}
static void grouping(bool canAssign) {
    // prefix: paren precedence global var
    expr();
    consume(TOKEN_LEFT_PAREN, "Expect ) after expr");
}
static void unary(bool canAssign) {
    // prefix: unary
    // scanner TOKEN_BANG -> chunk OP_NOT -> compiler unary
    TokenType opType = parser.prev.type;
    parsePrecedence(PREC_UNARY);
    switch(opType) {
        case TOKEN_BANG: emitByte(OP_NOT); break;
        case TOKEN_MINUS: emitByte(OP_NEGATE); break;
        default: return;
    }
}
static binary(bool canAssign) {
    // infix: table of func ptrs
    /* a != b */
    // parse left a, push val, consume != opType \
        parse right b at higher precedence, push val \
        emit OP_EQUAL, pop b a, push val if equal \
        emit OP_NOT, pop equal, push negated
    TokenType opType = parser.prev.type;
    ParseRule* rule = getRule(opType);
    parsePrecedence((Precedence)(rule->precedence+1));
    switch (opType) {
        case TOKEN_BANG_EQUAL:  emitByte(OP_EQUAL, OP_NOT); break;
        case TOKEN_PLUS:        emitByte(OP_ADD); break;
        default: return;
    }
}
static void literal(bool canAssign) {
    switch (parser.prev.type) {
        case TOKEN_FALSE: emitByte(OP_FALSE); break;
        default: return;
    }
}
ObjFunc* compile(const char* src) {
    // only scan the token when compiler needs one
    // mode ends when synchronized statement boundaries
    initScanner(src);
    Compiler compiler;
    initCompiler(&compiler, TYPE_SCRIPT);
    curChunk = chunk;
    parser.panicMode = false;
    parser.hadErr = false;
    advance();
    while (!match(TOKEN_EOF)) {
        declaration();
    }
    ObjFunc* func = endCompiler();
    return parser.hadErr ? NULL : func;
    // hand-written scanner test
    /*
        int line = -1;
        for (;;) {
            Token token = scanToken();
            if (token.line != line) {
                printf("%4d ", token.line);
                line = token.line;
            } else {
                printf("    | ");
            }
            // *s: length num precision in next arg -> token.len
            // token.start: the str to print and the length is token.len
            printf("%2d. '%.*s'\n", token.type, token.len, token.start);
            // eof print empty token ''
            if (token.type == TOKEN_EOF) break;
        }
    */
    
}