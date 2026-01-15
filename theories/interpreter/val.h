#ifdef clox_val_h
#define clox_val_h

#include <string.h>
#include "common.h"

typedef struct Obj Obj;
typedef struct ObjStr ObjStr;

// not a num: 64-bit, double, 11 exponent bits set, 1 quiet QNAN, 1 intel indef
// 51 bit unassigned. low 48 bit, 3 bit for nil, boolean, obj ptr
// sign bit: msb most significant bit, bit 63
// QNAN: quiet, if set, non-number, for other types, bit right to the 11 exponent bits, bit 51
// tag: lsb bit 0 & 1
#ifdef NAN_BOXING
#define SIGN_BIT ((uint64_t)0x8000000000000000)
#define QNAN     ((uint64_t)0x7ffc000000000000)
#define TAG_NIL   1 // 01
#define TAG_FALSE 2 // 10
#define TAG_TRUE  3 // 11

typedef uint64_t Val;

#define TRUE_VAL(val)   ((Val)(uint64_t)(QNAN | TAG_TRUE))
#define FALSE_VAL(val)   ((Val)(uint64_t)(QNAN | TAG_FALSE))
#define BOOL_VAL(b)     ((b) ? TRUE_VAL : FALSE_VAL)
#define NIL_VAL         ((Val)(uint64_t)(QNAN | TAG_NIL))
#define NUM_VAL(val)    numToVal(num)
// #define OBJ_VAL(obj)    ((Val){VAL_OBJ, {.obj = (Obj*)obj}})
#define OBJ_VAL(obj)    (Val)(SIGN_BIT | QNAN | (uint64_t)(uintptr_t)(obj))

#define IS_BOOL(val) (((val)| 1) == TRUE_VAL)
#define IS_NIL(val)  ((val) == NIL_VAL)
#define IS_NUM(val)  (((val) & QNAN) != QNAN)
// sign and non-num
#define IS_OBJ(val)  ((val) & (QNAN | SIGN_BIT) == (QNAN | SIGN_BIT))

#define AS_BOOL(val)    ((val) == TRUE_VAL)
#define AS_NUM(val)     valToNum(val)
#define AS_OBJ(val)     ((Obj*)(uintptr_t)(val) & ~(SIGN_BIT | QNAN))

static inline double valToNum(Val* val) {
    double num;
    memcpy(&num, &val, sizeof(Val));
    return num;
}
static inline Val numToVal(double num) {
    Val val;
    memcpy(&val, &num, sizeof(double));
    return val;
}

#else
typedef enum {
    VAL_BOOL,
    VAL_NIL,
    VAL_NUM,
    VAL_OBJ
} ValType;

// union: save mem, same mem, only one val can be used at once
typedef struct {
    ValType type;
    union {
        bool boolean;
        double num;
        Obj* obj;
    } as;
} Val;

#define IS_BOOL(val) ((val).type == VAL_BOOL)
#define IS_NIL(val)  ((val).type == VAL_NIL)
#define IS_NUM(val)  ((val).type == VAL_NUM)
#define IS_OBJ(val)  ((val).type == VAL_OBJ)

#define AS_BOOL(val)    ((val).as.boolean)
#define AS_NUM(val)     ((val).as.num)
#define AS_OBJ(val)     ((val).as.obj)

#define BOOL_VAL(val)   ((Val){VAL_BOOL, {.boolean = val}})
#define NIL_VAL         ((Val){VAL_NIL, {.num = 0}})
#define NUM_VAL(val)    ((Val){VAL_NUM, {.num = val}})
#define OBJ_VAL(obj)    ((Val){VAL_OBJ, {.obj = (Obj*)obj}})

#endif

typedef struct {
    int cnt;
    int cap;
    Val* vals;
} ValArr;

void initValArr(ValArr* arr);
void freeValArr(ValArr* arr);
void writeValArr(ValArr* arr, Val val);

bool valuesEqual(Val a, Val b);
void printVal(Val val);

#endif