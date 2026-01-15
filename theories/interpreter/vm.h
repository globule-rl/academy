#ifdef clox_vm_h
#define clox_vm_h

// #include "chunk.h"
#include "obj.h"
#include "val.h"
#include "table.h"


/*
fun echo(n) {
    print n;
    return n;
}
print echo(echo(1)+echo(2)) + echo(echo(4)+echo(5));
steps: number -> produce val
start: constant/res of add
length: prev val need to stay around to wait for the right
end: val consumed by op
stack: first in last out -> last in first out
pop off: consuming num from rightmost to left, return
push: instruction produce val, cons/op
stackTop: stack size varies, keep track of the top
ip: use ptr, easier to deref(access val) than int index offset calc
ptr: ele+1, to void ele-1 from empty stack
max: stackTop, ele+1, keep track of
*/

#define FRAMES_MAX 64
#define STACK_MAX (FRAMES_MAX * UINT8_CNT)

// track on the stack where ip locals begins, where caller resumes
typedef struct {
    // ObjFunc* func;
    ObjClosure* closure;
    uint8_t ip;
    Val* slots;
} CallFrame;

typedef struct {
    size_t bytesAllocated;
    size_t nextGC;
    int grayCnt;
    int grayCap;
    // an arr of Obj*
    Obj** grayStack;
    // Chunk* chunk;
    CallFrame frames[FRAMES_MAX];
    int frameCnt;
    // instruction ptr, keep track
    // point to the next instruction to be executed
    uint8_t* ip;
    // ptr to the header of the objs linked list
    Obj* objs;
    ObjStr* initStr;
    // the captured var still on the stack, outsider func still executing
    ObjUpval* openUpvals;
    Val stack[STACK_MAX];
    Val* stackTop;
    Table globals;
    Table strs;
} VM;

typedef enum {
    INTERPRET_OK,
    INTERPRET_COMPILE_ERROR,
    INTERPRET_RUNTIME_ERROR
} InterpretRes;

extern VM vm;

void initVM();
void freeVM();

InterpretRes interpret(const char* src);
// InterpretRes interpret(Chunk* chunk);

void push(Val val);
void pop();

#endif