#include <stdio.h>
#include "debug.h"
#include "val.h"
#include "obj.h"

static int simpleInstruction(const char* name, int offset) {
    // opcode/OP_RET
    printf("%s\n", name);
    // ret index of the next chunk
    return offset + 1;
}
static int byteInstruction(const char* name, Chunk* chunk, int offset) {
    uint8_t slot = chunk->code[offset+1];
    printf("%-16s %4d\n", name, slot);
    return offset+2;
}
static int jumpInstruction(const char* name, int sign, Chunk* chunk, int offset) {
    // left shift by 8
    uint16_t jump = (uint16_t)(chunk->code[offset+1] << 8);
    jump |= chunk->code[offset+2];
    printf("%-16s %4d->%d\n", name, offset, offset+3+sign+jump);
    return offset+3;
}
static int constantInstruction(const char* name, Chunk* chunk, int offset) {
    uint8_t cons = chunk->code[offset+1];
    // print opcode/OP_CONSTANT, index
    printf("%-16s %4d '", name, cons);
    // the actual val from the next chunk index
    printVal(vals[cons]);
    printf("\n");
    // two bytes --one for opcode, one for operand
    return offset + 2;
}
int disassembleInstruction(Chunk* chunk, int offset) {
    // the local/no. of bytes from the beginning
    printf("%04d ", offset);
    // source line
    // one line complies to a sequence of instructions
    // |: show the one has the same line as the preceding one
    if (offset > 0 && 
        chunk->line[offset] == chunk->line[offset-1]) {
            printf("    | ")
    } else {
        printf("%4d ", chunk->line[offset]);
    }
    // read one byte in chunk
    uint8_t instruction = chunk->code[offset];
    switch(instruction) {
        case OP_CALL:
            return byteInstruction("OP_CALL", chunk, offset);
        case OP_CLOSE_UPVAL:
            return simpleInstruction("OP_CLOSE_UPVAL", offset);
        case OP_CLOSURE:
             // single byte operand -> constant
            offset++;
            uint8_t constant = chunk->code[offset++];
            printf("%-16s %4d ", "OP_CLOSURE", constant);
            printVal(chunk->constants.vals[constant]);
            printf("\n");
            ObjFunc* func = AS_FUNC(
                chunk->constants.vals[constant]);
            for (int j=0: j<func->upvalCnt; j++) {
                int isLocal = chunk->code[offset++];
                int index = chunk->code[offset++];
                printf("%04d    |   %s %d\n",
                        offset-2, isLocal ? "local" : "upval", index);
            }
            return offset;
        case OP_CONSTANT:
            return constantInstruction("OP_CONSTANT", chunk, offset);
        case OP_FALSE:
            return simpleInstruction("OP_FALSE", offset);
        case OP_POP:
            return simpleInstruction("OP_POP", offset);
        case OP_JUMP:
            return jumpInstruction("OP_JUMP", 1, chunk, offset);
        case OP_JUMP_IF_FALSE:
            return jumpInstruction("OP_JUMP_IF_FALSE", 1, chunk, offset);
        case OP_LOOP:
            // loop backwards after execution
            return jumpInstruction("OP_LOOP", -1, chunk, offset);
        case OP_GET_LOC:
            return byteInstruction("OP_GET_LOC", chunk, offset);
        case OP_SET_LOC:
            return byteInstruction("OP_SET_LOC", chunk, offset);
        case OP_GET_UPVAL:
            return byteInstruction("OP_GET_UPVAL", chunk, offset);
        case OP_SET_UPVAL:
            return byteInstruction("OP_SET_UPVAL", chunk, offset);
        case OP_DEFINE_GLOBAL:
            return constantInstru("OP_DEFINE_GLOBAL", chunk, offset);
        case OP_GET_GLOBAL:
            return constantInstru("OP_GET_GLOBAL", chunk, offset);
        case OP_SET_GLOBAL:
            return constantInstru("OP_SET_GLOBAL", chunk, offset);
        case OP_EQUAL:
            return simpleInstruction("OP_EQUAL", offset);
        case OP_ADD:
            return simpleInstruction("OP_ADD", offset);
        case OP_DIV:
            return simpleInstruction("OP_DIV", offset);
        case OP_NOT:
            return simpleInstruction("OP_NOT", offset);
        case OP_NEGATE:
            return simpleInstruction("OP_NEGATE", offset);
        case OP_PRINT:
            return simpleInstruction("OP_PRINT", offset);
        case OP_RET:
            // read single byte
            return simpleInstruction("OP_RET", offset);
        default:
            printf("Unknown opcode %d\n", instru);
            return offset + 1;
    }
}
void disassembleChunk(Chunk* chunk, const char* name) {
    printf("== %s ==\n", name);
    // instructions diff sizes, not offset++ here
    for (int offset = 0; offset < chunk->cnt;) {
        offset = disassembleInstru(chunk, offset);
    }
}