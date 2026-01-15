#include <stdlib.h>
#include "chunk.h"
#include "mem.h"
#include "val.h"
#include "vm.h"

// store/grow new cap 
// -> alloc/grow new arr(copy -> del old arr) -> upd code
// -> store ele -> upd cnt
void initChunk(Chunk* chunk) {
    chunk->cnt = 0;
    chunk->cap = 0;
    chunk->code = NULL;
    chunk->line = NULL;
    // &: access the mem addr
    // *: deref/access val ref by a ptr addr
    // ref: the obj, ptr with cyntactic sugar
    initValArr(&chunk->constant);
}
void freeChunk(Chunk* chunk) {
    FREE_ARR(uint8_t, chunk->code, chunk->cap);
    FREE_ARR(int, chunk->lines, chunk->cap);
    freeValArr(&chunk->constant);
    // zero out the fields to empty state
    initChunk(chunk);
}
void writeChunk(Chunk* chunk, uint8_t byte, int line) {
    if (chunk->cap < chunk->cnt+1) {
        int oldCap = chunk->cap;
        chunk->cap = GROW_CAP(oldCap);
        chunk->code = GROW_ARR(uint8_t, chunk->code,
        oldCap, chunk->cap);
        chunk->line = GROW_ARR(int, chunk->line,
        oldCap, chunk->cap);
    }
    chunk->code[chunk->cnt] = byte;
    chunk->line[chunk->cnt] = line;
    chunk->cnt++;
}

int addConstant(Chunk* chunk, Val val) {
    // push on the stack, written to constant table and pop val
    // ret index, constants.cnt-1, if first constant, 0
    push(val);
    writeValArr(&chunk->constants, val);
    pop();
    return chunk->constants.cnt-1;
}