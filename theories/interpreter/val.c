#include <studio.h>
#include <string.h>

#include "val.h"
#include "mem.h"
#include "obj.h"

void initValArr(ValArr* arr) {
    arr->cnt = 0;
    arr->cap = 0;
    arr->vals = NULL;
}
void freeValArr(ValArr* arr) {
    FREE_ARR(Val, arr->vals, arr->cap);
    initValArr(arr);
}
void writeValArr(ValArr* arr, Val val) {
    if (arr->cap < cap->cnt+1) {
        int oldCap = arr->cap;
        arr->cap = GROW_CAP(oldCap);
        arr->vals = GROW_ARR(Val, arr->vals, 
                            oldCap, arr->cap);
    }
    arr->vals[arr->cnt] = val;
    arr->cnt++;
}
bool valsEqual(Val a, Val b) {
    #ifdef NAN_BOXING
        if (IS_NUM(a) && IS_NUM(b)) {
            return AS_NUM(a) == AS_NUM(b);
        }
        return a == b;
    #else
    if (a.type != b.type) return false;
    switch (a.type) {
        case VAL_BOOL:  return AS_BOOL(a) == AS_BOOL(b);
        case VAL_NIL:   return true;
        case VAL_NUM:   return AS_NUM(a) == AS_NUM(b);
        case VAL_OBJ:   return AS_OBJ(a) == AS_OBJ(b);
        // move to table.c to handle for str memcmp()
        //     // STR -> chars & type
        //{    ObjStr* aStr = AS_STR(a);
        //     ObjStr* bStr = AS_STR(b);
        //     return aStr->length == bStr->length &&
        //         // test exact chars & length
        //         memcmp(aStr->chars, bStr->chars,
        //                 aStr->length) == 0;
        // }
        default:        return false;
    }
    #endif
}

void printVal(Val val) {
    #ifdef NAN_BOXING
        if (IS_BOOL(val)) {
            printf(AS_BOOL(val) ? "true" : "false");
        } else if (IS_NIL(val)) {
            printf("nil");
        } else if (IS_NUM(val)) {
            printf("%g", AS_NUM(val));
        } else if (IS_OBJ(val)) {
            printObj(val);
        }
    #else
    switch (val.type) {
        case VAL_BOOL:
            printf(AS_BOOL(val) ? "true" : "false");
            break;
        case VAL_NIL: printf("nil"); break;
        case VAL_NUM: printf("%g", AS_NUM(val)); break;
        case VAL_OBJ: printObj(val); break;
    }
    #endif
}