#include <common/types.h>
#include <gdt.h>
#include <memManagement.h>
#include <multiTasking.h>
#include <sysCalls.h>

/* kernel mem:
        BIOS/multiboot: firmware bootloader
        kernel code/,text: executable instructions 
        kernel data/.data: initialize globals
        kernel bss/.bss: uninitialized globals/zeroed
        kernel stack: per-cpu stacks
        vmalloc: hight mem/dynamic alloc
        modules: var/loadable mods
        user space: processes/virtual
    start at 1MB post-boot 
        paging -> high mem mapping
*/

using namespace os;
using namespace os::common;

/* 0xb8000: x86 protected text mode base addr: 80colx25chars, each 2 bytes(char+attr)
    &0xFF00: mask preserve high/attr byte
    | str[i]: update (x, y) to str[i]
    | ' ': set each char at offset/80*y+x to space/clear line 
*/
void printf(char* str) {
    static uint16_t* videoMem = (uint16_t*)0xb8000;
    static uint8_t x=0, y=0;
    for (int i=0; str[i]!='\0'; ++i) {
        switch(str[i]) {
            case '\n': x =0; y++; break;
            default: videoMem[80*y+x] = (videoMem[80*y+x] & 0xFF00) | str[i]; x++; break;
        }
        if (x >= 80) {x=0; y++;}
        if (y >= 25) {
            for (y=0; y<25; y++)
                for (x=0; x<80; x++) videoMem[80*y+x] = (videoMem[80*y+x] & 0xFF00) | ' '; x=0; y=0;
            }
    }
}

/* (*i)(): deref call each as func to run global/static init 
    +8: add 8 bytes/offset to mem_upper kb
    memManager: start from heap 0x100000; kb->bytes, size 10kb reserve
        prevent overflow 
            dynamic allc malloc/free within bounds
    irq: interrupt request from device
*/
typedef void (*ctor)();
extern "C" ctor start_ctors;
extern "C" ctor end_ctors;
extern "C" void callCtors() {
    for (ctor* i=&start_ctors; i!=&end_ctors; i++) (*i)();
}

extern "C" void kernelMain(const void* multiboot_structure, uint32_t /*multiboot_magic*/) {
    printf("hello world");
    Gdt gdt;

    uint32_t* memupper = (uint32_t*)(((size_t_32)multiboot_structure) + 8);
    size_t_32 heap = 10*1024*1024;
    MemManager memManager(heap, (*memupper)*1024 - heap - 10*1024);
    printf("heap: 0x");
    // printHex((heap >> 24) & oxFF);    

    void* alllocd = memManager.malloc(1024);

    TaskManager taskManager;
    InterrupManager interrupts(0x20, &gdt, &taskManager);
    SyscallHandler syscalls(&interrupts, 0x88);
}