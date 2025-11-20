#include <gdt.h>

using namespace os;
using namespace os::common;

/* 1 byte = 8 bits 
        0xF 2^4-1
        0x10 1<<4 16 
        0x40 2^6 64
        0xFF 2^8-1
        0xFFF 2^12-1/4*1024/4kb, <<12: *4096, |0xFFF: +4096
        0x10000 1<<16 2^16 65536 64kb 64*1024
        0xC0 (1<<7)|(1<<6) 2^7+2^6 bit7-G bit6-DB -> granularity 4KB
        0xFFFFF 2^20-1 4GB-1
        << 16: shift to upper/left limit high/flag inside base
            * 2^16/65536, adds 0x0000 to the right
             format/little-endian store in ram: 
                16-bit limit low + 32-bit base high
                    skip low 16-bit limit=0/+2 bytes
            when load + 2 ptr/bytes, point to base, know the limit lo/limit high inside base
        limit>>12 discard lower 12 bits, divide by 2^12/4096->4KB pages
        (limit>>8) & 0xFF 000000000 11111111
            discard 8 bit, shift 15-8 to 7-0
            & 0xFF -> zeros all bits except 7-0/original 15-8
             -> extract/isolate original 15-8  
    initializer/private (base, limit, type)
        codeSeg: base=0 limit=0xFFFFF 64*1024*1024 64MiB/Mebi
            type/access
                0x9A/1001 1010b 1010b->executable 
                0x92/1001 0010b->writable
        addr n size:
            i[2]: 16-bit limit + 32-bit base
                i[0] -> low = 0, limit high 16-bit
                    offset + 2 -> first byte  
                i[1] -> base 32-bit addr/this
    lgdt (%0): load addr n size register to Gdt
        %0: place holder
            mem addr of gdtr loaded to lgdt/local
        asm volatile: as-is, prevent reordering/optimizing remove/move this instruction
*/
Gdt::Gdt() 
    : nullSegSelector(0,0,0),
        unusedSegSelector(0,0,0),
        codeSegSelector(0, 64*1024*1024, 0x9A),
        dataSegSelector(0, 64*1024*1024, 0x92) {
            /* uint32_t i[2];
            i[0] = sizeof(Gdt) << 16;
            i[1] = (uint32_t)this;
            asm volatile("lgdt (%0)": :"p" (((uint8_t*)i) + 2));
            */
           struct GDTR {uint16_t limit; uint32_t base;} __attribute__((packed));
           GDTR gdtr;
           gdtr.limit = sizeof(Gdt) - 1;
           gdtr.base = (uint32_t)this;
            asm volatile("lgdt %0" : : "m"(gdtr));
        }
Gdt::~Gdt() {}

/* offset 16-bit ptr, selector points to descriptors within the gdt table
*/
uint16_t Gdt::CodeSegSelector() {
    return (uint8_t*)&codeSegSelector - (uint8_t*)this;
}
uint16_t Gdt::DataSegSelector() {
    return (uint8_t*)&dataSegSelector - (uint8_t*)this; 
}

/*  limit <= 1<<16/64kb/16-bit: set limit high to 2^6/64, no scaling
        elif larger than 64kb, 32-bit: set for 4kb granularity/page boundary, scaling/2^12/4096 steps each
            if not aligned w/ 0xFFFF/not all 1s/not full page
                 limit page count/discard 12-bit - 1, cant increase limit/highest index -> (limit+1)pages 
                    round down, limit->page-1, force to full pages -> 4095 wasted bytes at most
                        actual max page offset size: (limit*4096)+4095/(limit+1)x4096-1 bytes 
            elif 0xFFFF page count/2^12/4096/4kb
        set flag at entry[6]
            -> maximize seg size in 20-bit/0xFFFFF/4GB-1, flat 4GB seg 
    entry[0] [1] [6]: limit low bits: 7-0, 15-8 
        low nibble : byte6 high 4-bit 19-16
        20-bit->0xF
    entry[5] high nibble 4-bit/flags P/DPL/S/Type
    entry[2 3 4 7]: 4 base entries 
        selector 0x00/null, 0x08 8/kernel code, 0x10 16/kernel data, 0x18 24/user code
            index*8
*/
Gdt::SegDescriptor::SegDescriptor(uint32_t base, uint32_t limit, uint8_t type) {
    uint8_t* entry = (uint8_t*)this;
    if (limit <= (1<<16)) {entry[6]=(1<<6);}
    else {
        if ((limit & 0xFFF) != 0xFFF) limit = (limit>>12)-1;
        else limit = limit>>12;
        entry[6] = 0xC0; 
    }
    entry[0] = limit & 0xFF;
    entry[1] = (limit>>8) & 0xFF;
    entry[6] = (limit>>16) & 0xF;

    entry[2] = base & 0xFF;
    entry[3] = (base>>8) & 0xFF;
    entry[4] = (base>>16) & 0xFF;
    entry[7] = (base>>24) & 0xFF;

    entry[5] = type;
}

/* rebuild limit/base addr
    0xC0/bit 7 & 6: apply 4kb page granularity to seg limit 
        count pages, not bytes
        flat seg, convert back to actual bytes-1 -> 4GB limit 
            otherwise 1MB limit tiny */
uint32_t Gdt::SegDescriptor::Limit() {
    uint8_t* entry = (uint8_t*)this;
    uint32_t res = (entry[6] << 16) | (entry[1] << 8) | entry[0];
    if ((entry[6] & 0xC0) == 0xC0) res = (res << 12) | 0xFFF;
    return res;
}
uint32_t Gdt::SegDescriptor::Base() {
    uint8_t* entry = (uint8_t*)this;
    uint32_t res = (entry[7] << 24) | (entry[4] << 16) | (entry[3] << 8) | entry[2];
    return res;
}
