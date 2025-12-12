#pragma once

#include <common/types.h>

/* inline assembly
    volatile: side-effect i/o, not optimize away
    inl: long
    inb: inp bytes, read from I/O port into al register/cpu
        outb: write from al register/cpu to I/O port
    inw word 16, inl long 32
    =a: = output, a -> store in eax/al lower 8-bit, then var res
        Nd: N constant/8-bit/0-255, 
            d -> edx/large port 
    %0: first operand res, _data
        %1: snd operand port
            start from output %0
    : output : input <- cpp to assembly
        assembly: input byte from I/O port %1 into register/cpu %0
    ::input no output <- cpp to assembly 
        assembly: output/cpu out byte %0 to I/O port %1
    jmp: small delay, hw processing time after I/O op before next instruction 
        jump to next label 1
*/
namespace os {
    namespace hwCom {
        class Port {
            protected:
                Port(uint16_t port);
                virtual ~Port();
                uint16_t port;
        };
        
        class Port8Bit : public Port {
            protected:
                static inline uint8_t Read8(uint16_t _port) {
                    uint8_t res;
                    __asm__ volatile("inb %1, %0" : "=a" (res) : "Nd" (_port));
                    return res;
                }
                static inline void Write8(uint16_t _port, uint8_t _data) {
                    __asm__ volatile("outb %0, %1" : : "a" (_data), "Nd" (_port));
                }
            public:
                Port8Bit(uint16_t port);
                ~Port8Bit();
                virtual uint8_t Read();
                virtual void Write(uint8_t _data);
        };

        class Port8BitSlow : public Port8Bit {
            protected:
                static inline void Write8Slow(uint16_t _port, uint8_t _data) {
                    __asm__ volatile("outb %0, %1\njmp 1f\n1: jmp 1f\n1:" : : "a" (_data), "Nd" (_port));
                }
            public:
                Port8BitSlow(uint16_t port);
                ~Port8BitSlow();
                virtual void Write(uint8_t data);
        };

        class Port16Bit : public Port {
            protected:
                static inline uint16_t Read16(uint16_t _port) {
                    uint16_t res;
                    __asm__ volatile("inw %1, %0" : "=a" (res) : "Nd" (_port));
                    return res;
                }
                static inline void Write16(uint16_t _port, uint16_t _data) {
                    __asm__ volatile("outw %0, %1" : : "a" (_data), "Nd" (_port));
                }
            public:
                Port16Bit(uint16_t port);
                ~Port16Bit();
                virtual uint16_t Read();
                virtual void Write(uint16_t data);
        };

        class Port32Bit : public Port {
            protected:
                static inline uint16_t Read32(uint16_t _port) {
                    uint32_t res;
                    __asm__ volatile("inl %1, %0" : "=a" (res) : "Nd" (_port));
                    return res;
                }
                static inline void Write32(uint16_t _port, uint32_t _data) {
                    __asm__ volatile("outl %0, %1" : : "a" (_data), "Nd" (_port));
                }
            public:
                Port32Bit(uint16_t port);
                ~Port32Bit();
                virtual uint32_t Read();
                virtual void Write(uint32_t data);
        };
    }
}


