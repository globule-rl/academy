#ifdef clox_debug_h
#define clox_debug_h

#include "chunk.h"

static int simpleInstru(const char* name, int offset);
static int byteInstru(const char* name, Chunk* chunk, int offset);
static int consInstru(const char* name, Chunk* chunk, int offset);
int disassembleInstru(Chunk* chunk, int offset);
void disassembleChunk(Chunk* chunk, const char* name);

#endif