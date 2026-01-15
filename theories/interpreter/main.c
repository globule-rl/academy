// 100x two orders of magnitude faster than ast
// cache: spatial locality/nearby block mem, buffer between processor & memory
// L1 cache: instructions(l-cache) data(d-cache) L3 cache: data memory
// overhead, ptr(mem addr) fields push objs away, out of cache
// emulator vm
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "common.h"
#include "chunk.h"
#include "vm.h"


// read-eval-print-loop
static void repl() {
    char line[1024];
    for (;;) {
        printf("> ");
        if (!fgets(line, sizeof(line), stdin)) {
            printf("\n");
            break;
        }
        interpret(line);
    }
}
// need to alloc str, but dont know the size until read it
// rb: read in binary mode
// seek to the end before reading it to get the fileSize
// rewind to the beginning
// alloc a buffer/str of that size + null byte
// read the whole file in one batch
// \0: null terminator
static char* readFile(const char* path) {
    FILE* file = fopen(path, "rb");
    if (file == NULL) {
        fprintf(stderr, "Could not open file \"%s\"\n", path);
        exit(74);
    }
    fseek(file, OL, SEEK_END);
    size_t fileSize = ftell(file);
    rewind(file);
    char* buffer = (char*)malloc(fileSize+1);
    if (buffer == NULL) {
        fprintf(stderr, "Not enough mem to read \"%s\"\n", path);
        exit(74);
    }
    size_t bytesRead = fread(buffer, sizeof(char), fileSize, file);
    if (bytesRead < fileSize) {
        fprintf(stderr, "Could not read file \"%s\"\n", path);
        exit(74);
    }
    buffer[bytesRead] = '\0';

    fclose(file);
    return buffer;
}
static void runFile(const. char* path) {
    char* src = readFile(path);
    InterpretRes res = interpret(src);
    free(src);
    if (res == INTERPRET_COMPILE_ERROR) exit(65);
    if (res == INTERPRET_RUNTIME_ERROR) exit(70);
}

int main(int argc, const char* argv[]) {
    initVM();

    // single arg: path/nmae, no arg to exec
    if (argc == 1) {
        repl();
    } else if (argc == 2) {
        runFile(argv[1]);
    } else {
        fprintf(stderr, "Usage: clox [path]\n");
        exit(64);
    }

    freeVM();
    /*
        // hand-written chunks of bytecodes test
        #include "debug.h"
        Chunk chunk;
        initChunk(&chunk);
        // ret the index
        int cons = addCons(&chunk, 1.2);
        // first byte opcode: how many operand bytes & what its like
        // which constant to load from chunk arr/constant pool
        // OP_CONS -> 1.2
        writeChunk(&chunk, OP_CONS, 123);
        // 2nd byte index operand in the constant pool
        writeChunk(&chunk, cons, 123);

        // (1.2+3.4)/5.6
        cons = addCons(&chunk, 3.4);
        writeChunk(&chunk, OP_CONS, 123);
        writeChunk(&chunk, cons, 123);
        writeChunk(&chunk, OP_ADD, 123);
        cons = addCons(&chunk, 5.6);
        writeChunk(&chunk, OP_CONS, 123);
        writeChunk(&chunk, cons, 123);
        writeChunk(&chunk, OP_DIV, 123);
        writeChunk(&chunk, OP_NEGATE, 123);

        // 3rd single one byte return instruction
        writeChunk(&chunk, OP_RET, 123);
        // print 3 bytes (offset0000 line123 opcode_name index val)
        // == test chunk ==
        // 0000 123 OP_CONS     0 '1.2'
        // 0002     | OP_RET
        disassembleChunk(&chunk, "test chunk");
        interpret(&chunk);
        freeVM();
        freeChunk(&chunk);
    */

    return 0;
}