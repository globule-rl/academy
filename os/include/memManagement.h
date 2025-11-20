#pragma once

#include <common/types.h>
#include <cstddef>

/* protected: used by subclass
        private: only by itself
    ~MemManager(): destructor deactivates self
        global activeMemmenager: nullptr
            prevent dangling ref post-deletion
*/
namespace os {
    struct MemChunk {
        MemChunk *next;
        MemChunk *prev;
        bool allocd;
        common::size_t_32 size;
    };
    class MemManager {
        protected: MemChunk* first;
        public: 
            static MemManager *activeMemManager;
            MemManager(common::size_t_32 first, common::size_t_32 size);
            ~MemManager();
            void* malloc(common::size_t_32 size);
            void free(void* ptr);
    };
}

/* operator new: global overload mem alloc op
    new: alloc raw mem before constructor
        redirect alloc to custom MemManger::malloc
    unsigned size -> std::size_t: 64 for 64-bit sys, 32 for 32-bit
        <cstddef>: freestanding header 
*/
void* operator new(std::size_t size);
void* operator new[](std::size_t size);
void* operator new(std::size_t size, void* ptr);
void* operator new[](std::size_t size, void* ptr);
void operator delete(void* ptr);
void operator delete[](void* ptr);


