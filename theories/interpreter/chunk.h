#ifdef clox_chunk_h
#define clox_chunk_h

#include "common.h"
#include "val.h"

// instructions
// on which order to produce
typedef enum {
    OP_CALL,
    OP_CONSTANT,
    OP_CONSTANT_16,
    OP_FALSE,
    OP_POP,
    OP_JUMP,
    OP_JUMP_IF_FALSE,
    OP_LOOP,
    OP_GET_LOC,
    OP_SET_LOC,
    OP_DEFINE_GLOBAL,
    OP_GET_GLOBAL,
    OP_SET_GLOBAL,
    OP_EQUAL,
    OP_ADD,
    OP_DIV,
    OP_NOT,
    OP_NEGATE,
    OP_PRINT,
    // ret from the curr func
    OP_RET,
} OpCode;

// dynamic array: dense storage/cache-friendly
// O(1) index, O(1) append
// cap: no. of ele in the arr
// cnt: entries in use
typedef struct {
    int cnt;
    int cap;
    uint8_t* code;
    int* line;
    ValArr constants;
} Chunk;

void initChunk(Chunk* chunk);
void freeChunk(Chunk* chunk);
void writeChunk(Chunk* chunk, uint8_t byte, int line);
int addConstant(Chunk* chunk, Val val);

#endif