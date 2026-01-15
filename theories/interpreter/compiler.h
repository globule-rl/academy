#ifndef clox_compiler_h
#define clox_compiler_h
#include "vm.h"
#include "obj.h"


void markCompilerRoots();
// bool compile(const char* src, Chunk* chunk);
ObjFunc* compile(const char* src);

#endif