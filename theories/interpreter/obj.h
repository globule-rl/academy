#ifndef clox_obj_h
#define clox_obj_h

#include "common.h"
#include "val.h"
#include "chunk.h"

// valType union AS_OBJ -> type -> obj -> ObjStr
#define OBJ_TYPE(val)   (AS_OBJ(val)->type)

// to support dynamic typing
// methods, initializer
#define IS_BOUND_METHOD(val)    isObjType(val, OBJ_BOUND_METHOD)
#define IS_CLASS(val)           isObjType(val, OBJ_CLASS)
#define IS_INSTANCE(val)        isObjType(val, OBJ_INSTANCE)
#define IS_FUNC(val)            isObjType(val, OBJ_FUNC)
#define IS_CLOSURE(val)         isObjType(val, OBJ_CLOSURE)
#define IS_NATIVE(val)          isObjType(val, OBJ_NATIVE)
#define IS_STR(val)             isObjType(val, OBJ_STR)

#define AS_BOUND_METHOD(val)    ((ObjBoundMethod*)AS_OBJ(val))
#define AS_CLASS(val)           ((ObjClass*)AS_OBJ(val))
#define AS_INSTANCE(val)        ((ObjInstance*)AS_OBJ(val))
#define AS_FUNC(val)            (ObjFunc*)AS_OBJ(val)
#define AS_CLOSURE(val)         (ObjClosure*)AS_OBJ(val)
// extract the val from NativeFn func
#define AS_NATIVE(val)  \
        (((ObjNative*)AS_OBJ(val))->func) \
#define AS_STR(val)             ((ObjStr*)AS_OBJ(val))
#define AS_CSTR(val)            (((ObjStr*)AS_OBJ(val))->chars)


typedef enum {
    OBJ_BOUND_METHOD,
    OBJ_CLASS,
    OBJ_INSTANCE,
    OBJ_UPVAL,
    OBJ_FUNC,
    OBJ_CLOSURE,
    OBJ_NATIVE,
    OBJ_STR
} ObjType;

typedef Val (*NativeFn)(int argCnt, Val* args);

struct Obj {
    // struct Obj{}: new -> struct Obj obj
    // typedef Obj{}: new -> ObjFunc func, anonymous name, alias 
    // typedef struct ObjUpval{}ObjUpval: new -> ObjUpval upval, link next struct
    // next: linked list node, ptr to the Obj struct itself
    ObjType type;
    bool isMarked;
    struct Obj* next;
};

struct ObjStr {
    Obj obj;
    int length;
    char* chars;
    uint32_t hash;
};

typedef struct {
    // arity: no. of parameter func expects
    // chunk not a ptr anymore, its a struct
    Obj obj;
    int arity;
    int upvalCnt;
    Chunk chunk;
    ObjStr* name;
} ObjFunc;

typedef struct ObjUpval{
    Obj obj;
    // ptr to the val, assigning actual var to upval, not copy
    Val* location;
    Val closed;
    // keep track
    struct ObjUpval* next;
} ObjUpval;

typedef struct {
    Obj obj;
    ObjFunc* func;
    // ptr to dynamically allocated arr of ptrs to upvals
    // each closure has an arr of upvals, captured var
    // access to var even after popped from stack
    ObjUpval** upvals;
    int upvalCnt;
} ObjClosure;

typedef struct {
    Obj obj;
    NativeFn func;
} ObjNative;

typedef struct {
    Obj obj;
    Val receiver;
    ObjClosure* method;
} ObjBoundMethod;

typedef struct {
    Obj obj;
    ObjStr* name;
    Table methods;
} ObjClass;

typedef struct {
    Obj obj;
    ObjClass* klass;
    Table fields;
} ObjInstance;

// inline: compiler integrate the code into caller
// reduce exec time
// vm takes val, not ptr (Obj* obj)
static inline bool isObjType(Val val, ObjType type) {
    return IS_OBJ(val) && AS_OBJ(val)->type == type;
}

ObjBoundMethod* newBoundMethod(Val receiver, ObjClosure* method);
ObjClass* newClass(ObjStr* name);
ObjInstance* newInstance(ObjClass* klass);
ObjUpval* newUpval(Val* slot);
ObjClosure* newClosure(ObjFun* func);
ObjFunc* newFunc();
ObjNative* newNative();

// static ObjStr* allocStr(char* chars, int length, int hash);
// int hashStr(const char* key, int length);

ObjStr* copyStr(const char* chars, int length);
ObjStr* takeStr(char* chars, int length);
void printObj(Val val);

#endif