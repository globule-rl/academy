#include "memManagement.h"

using namespace os;
using namespace os::common;

MemManager* MemManager::activeMemManager = nullptr;

/* this: ptr to cur obj
*/
MemManager::MemManager(size_t_32 start, size_t_32 size) {
    activeMemManager = this;
    if (size < sizeof(MemChunk)) first = 0;
    else {
        first = (MemChunk*)start;
        first->allocd = false;
        first->prev = nullptr;
        first->next = nullptr;
        first->size = size  - sizeof(MemChunk);
    }
}
MemManager::~MemManager() {
    if (activeMemManager == this) activeMemManager = nullptr;
}

/* first-fit: alloc first chunk > size, faster search, external/between blocks fragmentation
        best-fit: alloc smallest chunk, full scan, sort by size, less fragmentation, slow
        next-fit: start from last allocd chunk, spread alloc, internal/inside blocks fragmentation, doesnot restart search
    chunk && !res: until not the end and not found
         !chunk/end or res/found the fit chunk 
        -> res <- chunk
    res->size: total cap/original chunk, header+old data
        sizeof(Memchunk): header struct (size, allod, next, prev)
        1: extra, avoid zero-size
        size: user data size
    *temp = res + MemChunk + size: split/temp if enough space for remainder
            place right after user size 
        res->old_size - size - sizeof(MemChunk): remainder
            ptr to new free chunk 
        set temp 
            size: remainder res->size-size-MemChunk
            prev/next: res->temp->next
        update links/double-linked list
            old_res->next/temp->next -> prev <= temp
            res->next <= temp
        shrink res->size to size
            temp/next -> ptr after size/new chunk
*/
void* MemManager::malloc(size_t_32 size) {
    if (size == 0) return nullptr;
    MemChunk* res = nullptr;
    for (MemChunk* chunk = first; chunk && !res; chunk=chunk->next)
        if (chunk->size > size && !chunk->allocd) res=chunk;
    if (!res) return nullptr;
    if (res->size >= size + sizeof(MemChunk) + 1) {
        MemChunk* temp = (MemChunk*)((size_t_32)res + sizeof(MemChunk) + size);
        temp->allocd = false;
        temp->size = res->size - size - sizeof(MemChunk);
        temp->prev = res;
        temp->next = res->next;
        if (temp->next) temp->next->prev = temp;
        res->size = size;
        res->next = temp;
    }
    res->allocd = true;
    return (void*)(((size_t_32)res) + sizeof(MemChunk));
}

/* coalese: merge next then prev -> chunk
    + size + header
        reduce external fragmentation, reusable
        merge next first: avoid ptr invalidation, addr same
*/
void MemManager::free(void* ptr) {
    if (!ptr) return;
    MemChunk* chunk = (MemChunk*)((size_t_32)ptr - sizeof(MemChunk));
    chunk -> allocd = false;
    if (chunk->next && !chunk->next->allocd) {
        chunk->size += chunk->next->size + sizeof(MemChunk);
        chunk->next = chunk->next->next;
        if (chunk->next) chunk->next->prev = chunk;
    }
    if (chunk->prev && !chunk->prev->allocd) {
        chunk->prev->next = chunk->next;
        chunk->prev->size += chunk->size + sizeof(MemChunk);
        if (chunk->next) chunk->next->prev = chunk->prev;
        // chunk = chunk->prev;
    }
    
}

void *operator new(std::size_t size) {
    if (!MemManager::activeMemManager) return nullptr;
    return MemManager::activeMemManager->malloc(size);
}
void *operator new[](std::size_t size) {
    if (!MemManager::activeMemManager) return nullptr;
    return MemManager::activeMemManager->malloc(size);
}

void *operator new(std::size_t size, void* ptr)
{
    return ptr;
}

void operator delete(void* ptr) {
    if (!MemManager::activeMemManager) MemManager::activeMemManager->free(ptr);
}
void operator delete[](void* ptr) {
    if (!MemManager::activeMemManager) MemManager::activeMemManager->free(ptr);
}