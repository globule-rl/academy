#include <stdio.h>
#include <string.h>
#include "mem.h"
#include "obj.h"
#include "val.h"
#include "vm.h"
#include "table.h"

// macros pre-processed, func compiled not pre
// generic code for any types, template in c++
// avoid redundantly cast void* back to type
// cast to the correct type* & compute sizeof()
#define ALLOC_OBJ(type, objType) \
    (type*)allocObj(sizeof(type), objType)

// actual func for the macros
static Obj* allocObj(size_t size, objType type) {
    // reallocate() returns void ptr: void* realloc()
    // alloc any kinds of objs
    Obj* obj = (Obj*)reallocate(NULL, 0, size);
    obj->type = type;
    obj->isMarked = false;
    // insert obj into the linked list objs
    // ptr to the head of already existed linked list
    obj->next = vm.objs;
    // update the objs ptr to the new obj
    vm.objs = obj;
#ifdef DEBUG_LOG_GC
    // %zu size_t
    printf("%p allocate %zu for %d\n", (void*)obj, size, type);
%endif
    return obj;
}
ObjUpval* newUpval(Val* slot) {
    ObjUpval* upval = ALLOC_OBJ(ObjUpval, OBJ_UPVAL);
    upval->location = slot;
    upval->next = NULL;
    return upval;
}
ObjClosure* newClosure(ObjFunc* func) {
    // alloc & vm.objs ptr next
    ObjUpval* upvals = ALLOC(ObjUpval*, func->upvalCnt);
    upval->closed = NIL_VAL;
    upval->location = slot;
    for (int i=0; i<func->upvalCnt; i++) {
        upvals[i] = NULL;
    }
    ObjClosure* closure = ALLOC_OBJ(ObjClosure, OBJ_CLOSURE);
    closure->func = func;
    closure->upvals = upvals;
    closure->upvalCnt = func->upvalCnt;
    return closure;
}
ObjFunc* newFunc() {
    ObjFunc* func = ALLOC_OBJ(ObjFunc, OBJ_FUNC);
    func->arity = 0;
    func->upvalCnt = 0;
    func->name = NULL;
    initChunk(&func->chunk);
    return func;
}
ObjNative* newNative() {
    ObjNative* native = ALLOC_OBJ(ObjNative, OBJ_NATIVE);
    native->func = func;
    return native;
}
// obj constructor
static ObjStr* allocStr(char* chars, int length, uint32_t hash) {
    ObjStr* str = ALLOC_OBJ(objStr, OBJ_STR);
    str->length = length;
    str->chars = chars;
    str->hash = hash;
    // stash str pool on the stack first n pop it off once its in tableSet()
    push(OBJ_VAL(str));
    tableSet(&vm.strs, str, NIL_VAL);
    pop();
    return str;
}
static uint32_t hashStr(const char* key, int length) {
    uint32_t hash = 2166136261u;
    for (int i=0; i<length; i++) {
        hash ^= (uint8_t)key[i];
        hash *= 16777619;
    }
    return hash;
}
// look up in the str table first, if no, alloc and store in the table
ObjStr* copyStr(const char* chars, int length) {
    // return a ref to the str, instead of copying
    uint32_t hash = hashStr(chars, length);
    ObjStr* interned = tableFindStr(&vm.strs, chars, length, hash);
    if (interned != NULL) return interned;
    // alloc a new arr on heap
    // ALLOC() returns void ptr: void* realloc()  
    // with given type & cnt, just big enough
    char* heapChars = ALLOC(char, length+1);
    // memcpy(dest, src, size_t cnt);
    memcpy(heapChars, chars, length);
    // trailing terminator
    heapChars[length] = '\0';
    return allocStr(heapChars, length, hash);
}
// take ownership
ObjStr* takeStr(char* chars, int length) {
    uint32_t hash = hashStr(chars, length);
    ObjStr* interned = tableFindStr(&vm.strs, chars, length, hash);
    if (interned != NULL) {
        // no need duplicate str, free mem for str
        FREE_ARR(char, chars, length+1);
        return interned;
    }
    return allocStr(chars, length, hash);
}
static void printFunc(ObjFunc* func) {
    // print the top level func in diagnostic code
    if (func->name == NULL) {
        printf("<script>");
        return;
    }
    printf("<fn %s", func->name->chars);
}
void printObj(Val val) {
    switch(OBJ_TYPE(val)) {
        case OBJ_UPVAL:
            printf("upval");
            break;
        case OBJ_FUNC:
            printFunc(AS_FUNC(val));
            break;
        case OBJ_CLOSURE:
            prinFunc(AS_CLOSURE(val)->func);
            break;
        case OBJ_NATIVE:
            printf("<native fn>");
            break;
        case OBJ_STR:
            // CSTR -> chars, not type
            printf("%s", AS_CSTR(val));
            break;
    }
}