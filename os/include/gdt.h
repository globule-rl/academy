#pragma once

#include <common/types.h>

/* build Global Descriptor Table/Gdt for x86 protected mode
    from real mode to protected/long mode in kernel boot
            protected: mem protection of the not allocd
        define mem segments/code-instruction/data/stack/sys privileges
    32-bit: 64-bit also based on 32-bit
            seg selector: which entry to use
        limit: 20-bit
            0-1/16-bit 0-15, limit low
            6 limit high
        base: 
            byte2-3/bit0-15 base low
            byte4/bit16-23 base mid
            byte5 access/type 
            byte6/16-19 seg limit high/flag 
                    G/bit7: 1 -> 4KB granularity of seg limit, 
                            count pages/virtual mem protection, 4kb/2mb/1gb/4gb
                        0 -> seg limit count bytes
                    DB/bit6: default operand size/0-16bit 1-32bit
                    A: freely avail for use by sys software
                P: segment present
                DPL: descriptor privilege level
                S: system segment 0-sys 1-code/data
                Type: code/data, read/write/execute privilege
                    flag/kernel code: present=1, dpl/ring=0, 
                        code/data, exec/read /write
            byte7/bit24-31 base high
        CS/code seg, DS/data seg, SS/stack seg registers loaded 
            offset bit 8/16/24 
            -> Gdt
        descriptor: translate mem seg logic addr to linear
            define property in each seg
    little endian: lsb least significant/val byte -> lowest/left mem addr
    'little end' store first/left, increasing significance/val order bytes
        0x12345678 -> 78 56 34 12v
            start from lower bytes -> simplify multiplication/addition 
                    <- carry to next
                big endian: network protocol, intuitive
        32-bit int 1, addr offset 0x1234 hex 01 00 00 00 00
            -> only need to look at 1 byte/bool 
                align/caches no need to offset to offset
                    big-endian: 0x1234 + 3 -> packed to a byte, offset to 0x1237
        -> depends on requirement little/big
    paging: use disk as extra RAM
        virtual addr -> page table -> physical RAm
        split mem into fixed pages/4kb granuity
    attribute packed: no data structure padding -> less size
        data alignment -> 4 byte on 32-bit sys -> multiples of 4 -> increase performace for cpu
            -> increase size <- padding/insert meaningless bytes between last and next
*/
namespace os {
    class Gdt {
        public:
            class SegDescriptor {
                private:
                    os::common::uint16_t limit_lo;
                    os::common::uint16_t limit_hi;
                    os::common::uint16_t base_lo;
                    os::common::uint8_t base_hi;
                    os::common::uint8_t base_vhi;
                    os::common::uint8_t type;
                public:
                    SegDescriptor(os::common::uint32_t base, os::common::uint32_t limit, os::common::uint8_t type);                  
                    os::common::uint32_t Base();
                    os::common::uint32_t Limit();
            } __attribute__((packed));
        private:
            SegDescriptor nullSegSelector;
            SegDescriptor unusedSegSelector;
            SegDescriptor codeSegSelector;
            SegDescriptor dataSegSelector;
        public:
            Gdt();
            ~Gdt();
            os::common::uint16_t CodeSegSelector();
            os::common::uint16_t DataSegSelector();
    };
}

