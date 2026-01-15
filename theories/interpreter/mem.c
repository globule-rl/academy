#include <stdlib.h>
#include "mem.h"
#include "vm.h"
#include "compiler.h"

#ifdef DEBUG_LOG_GC
#include <stdio.h>
#include "debug.h"
#endif

// re-traversing
#define GC_HEAP_GROW_FACTOR 2

// mark to not be garbage collected
void markObj(Obj* obj) {
    if (obj == NULL) return;
#ifdef DEBUG_LOG_GC
    printf("%p mark ", (void*)obj);
    printVal(OBJ_VAL(obj));
    printf("\n");
#endif
    obj->isMarked = true;

    // white->isMarked gray dynamic growing arr->black
    // sys realloc(): mem for the grayStack shouldnt grow during gc and start a new gc
    // sizeof(Obj*): size needed for one ptr to obj * int = total bytes
    // if cant create stack, exit
    // update obj to stack, cnt++
    if (vm.grayCap < vm.grayCnt+1) {
        vm.grayCap = GROW_CAP(vm.grayCap);
        vm.grayStack = (Obj**)realloc(vm.grayStack, sizeof(Obj*) * vm.grayCap);
        if (vm.grayStack == NULL) exit(1);
    }
    vm.grayStack[vm.grayCnt++] = obj;
}
void markVal(Val val) {
    if (IS_OBJ(val)) markObj(AS_OBJ(val));
}
void markArr(ValArr* arr) {
    for (i=0; i<arr.cap; i++) {
        markVal(arr->val[i]);
    }
}
void markRoots() {
    // vm stacks: func call state, callFrame ptr closure, upvals list
    for (Val* slot=vm.stack; slot<vm.stackTop; slot++) {
        markVal(*slot);
    }
    for (int i=0; i<frameCnt; i++) {
        markObj((Obj*)vm.frames[i]->closure);
    }
    for (ObjUpval* upval = vm.openUpvals; upval != NULL; upval = upval->next) {
        markObj((Obj*)upval);
    }
    markTable(&vm.globals);
    markCompilerRoots();
}
static void blackenObject(Obj* obj) {
#ifdef DEBUG_LOG_GC
    printf("%p blacken ", (void*)obj);
    printVal(OBJ_VAL(obj));
    printf("\n");
#endif
    switch (obj->type) {
        case OBJ_BOUND_METHOD:
        ObjBoundMethod* bound = (ObjBoundMethod*)obj;
        markVal(bound->receiver);
        break;
    }
}
void traceRef() {

}
void sweep() {
    // loop: keep track of prev, prev null case
    // if marked black to be protected, unmark, leave it, go to next, obj->prev, next->obj
    // if not, unlink, obj->free, next->obj
    // if prev == null, head vm.objs >obj, if not null, next->obj
    Obj* prev = NULL;
    Obj* obj = vm.objs;
    while (obj != NULL) {
        if (obj->isMarked) {
            obj->isMarked = false;
            prev = obj;
            obj = obj->next;
        } else {
            Obj* unreached = obj;
            obj = obj->next;
            if (!prev == NULL) {
                prev->next = obj;
            } else {
                vm.objs = obj;
            }
            freeObj(unreached);
        }
    }
}
void collectGarbage() {
#ifdef DEBUG_LOG_GC
    printf("-- gc begin\n");
    size_t before = vm.byteAllocated;
#endif
    markRoots();
    traceRef();
    tableRemoveWhite(&vm.strs);
    sweep();
    vm.nextGC = vm.bytesAllocated * GC_HEAP_GROW_FACTOR;
#ifdef DEBUG_LOG_GC
    printf("-- gc end\n");
    printf("collected %zu bytes (from %zu to %zu) next at %zu\n", 
            before-vm.bytesAllocated, before, vm.bytesAllocated, vm.nextGC);
#endif
}
void* reallocate(void* ptr, size_t oldSize, size_t newSize) {
    // if oldSize == 0, equals to malloc()
    // return void 
    vm.bytesAllocated += newSize - oldSize;
    if (newSize > oldSize) {
#ifdef DEBUG_STRESS_GC
        collectGarbage();
#endif
    if (vm.bytesAllocated > vm.nextGC) {
        collectGarbage();
    }
    }
    if (newSize == 0) {
        free(ptr);
        return NULL;
    }
    void* res = realloc(ptr, newSize);
    if (res == NULL) exit(1);
    return res;
}

static void freeObj(Obj* obj) {
    // free obj node -> mem
    // %p: ptr
#ifdef DEBUG_LOG_GC
    printf("%p free type %d\n", (void*)obj, obj->type);
#endif
    switch(obj->type) {
        case OBJ_UPVAL: {
            FREE(ObjUpval, obj);
            break;
        }
        case OBJ_FUNC: {
            ObjFunc* func = (ObjFunc*)obj;
            freeChunk(&func->chunk);
            FREE(ObjFunc, obj);
            break;
        }
        case OBJ_CLOSURE: {
            ObjClosure* closure = (ObjClosure*)obj;
            // FREE_ARR(type, ptr, oldCnt, 0)
            // FREE(ptr, size, 0) check null
            FREE_ARR(ObjClosure*, closure->upvals, closure->upvalCnt);
            FREE(Objclosure, obj);
            break;
        }
        case OBJ_NATIVE: {
            FREE(ObjNative, obj);
            break;
        }
        case OBJ_STR: {
            ObjStr* str = (ObjStr*)obj;
            FREE_ARR(char, str->chars, str->length+1);
            FREE(ObjStr, obj);
            break;
        }
    }
}
void freeObjs() {
    // free objs linked list
    // ptr to the objs
    Obj* obj = vm.objs;
    while (obj != NULL) {
        Obj* next = obj->next;
        freeObj(obj);
        // go to the next
        obj = next;
    }
}