#pragma once

#include <common/types.h>
#include <gdt.h>
#include <multiTasking.h>
#include <hwCom/port.h>

/* interrupts: asychronous/extern events, timer maskable/can disable
        exception: synchronous/instruction event, division by zero fault/recoverable/can retry, breakpoint, watchdog
        trap: intentional/dont retry, demand paging, syscalls, breakpoint
    256: 2^8
    0x31 2^5=32 2^6=64 49 custom
    0x80 2^7 128 old linux syscall 
    irq: interrupt request
    idt: interrupt descriptor table
        dpl: descriptorprivilegelevel
    pic: programmableIrqController
            peripheral interface controller
        python write 00 to 0F/13 in hex
            hex => {i:02X} 
            for i in range(16): print(f"static void HandleIrq0x{i:02X}();")
            for i in range(20): print(f"static void HandleException0x{i:02X}();")
    class IrqHandler;: forward declare cls
    virtual HandleIrq: polymophism override in derive class
    8259 pic i/o port irq mask:  master  cmd    port 0x0020
                                        data         0x0021
                                slave   cmd          0x00A0
                                        data         0x00A1
            real mode: master irq 0-7   offset 0x08 0x08-0x0F
                        slave irq 8-15          0x70 0x70-0x77
            protected mode: conflict w/ cpu exception
                 -> remapping 0-0xF -> 0x20-0x2F
    each hw irq -> assembly entry/register/pin
        0x20-0x2F -> irq0 irq1-irq15 16
            exception 0x00-0x13 20
                0x00 divide by zero
                0x01 debug
                0x02 non-maskable
                0x03 breakpoint
                0x04 overflow
                0x05 bound range exceeded
                0x06 invalid opcode
                0x07 device not avail
                ...
 */
namespace os {
    namespace hwCom {
        class IrqHandler;
        class IrqManager {
            friend class IrqHandler;
            protected:
                static IrqManager* activeIrqManager;
                IrqHandler* irqHandlers[256];
                TaskManager* taskManager;
                Port8BitSlow picMasterCmdPort;
                Port8BitSlow picMasterDataPort;
                Port8BitSlow picSlaveCmdPort;
                Port8BitSlow picSlaveDataPort;
                common::uint8_t hwIrqOffset;

                struct GateDescriptor {
                    common::uint16_t idtHandlerAddrLowBits;
                    common::uint16_t idtHandlerAddrHighBits;
                    common::uint16_t gdt_codeSegSelector;
                    common::uint8_t access;
                    common::uint8_t reserved;
                } __attribute__((packed));
                static GateDescriptor idt[256]; 
                struct IdtPtr {
                    common::uint16_t limit;
                    common::uint32_t base;
                } __attribute__((packed));
                static void SetIdtEntry(
                            common::uint8_t irq,
                            common::uint16_t codeSeg,
                            void (*idtHandler)(),
                            common::uint8_t DPL, 
                            common::uint8_t DescriptorType);
                            
                static void IrqIgnore();
                static void HandleException0x00();
                static void HandleException0x01();
                static void HandleException0x02();
                static void HandleException0x03();
                static void HandleException0x04();
                static void HandleException0x05();
                static void HandleException0x06();
                static void HandleException0x07();
                static void HandleException0x08();
                static void HandleException0x09();
                static void HandleException0x0A();
                static void HandleException0x0B();
                static void HandleException0x0C();
                static void HandleException0x0D();
                static void HandleException0x0E();
                static void HandleException0x0F();
                static void HandleException0x10();
                static void HandleException0x11();
                static void HandleException0x12();
                static void HandleException0x13();
                static void HandleIrq0x00();
                static void HandleIrq0x01();
                static void HandleIrq0x02();
                static void HandleIrq0x03();
                static void HandleIrq0x04();
                static void HandleIrq0x05();
                static void HandleIrq0x06();
                static void HandleIrq0x07();
                static void HandleIrq0x08();
                static void HandleIrq0x09();
                static void HandleIrq0x0A();
                static void HandleIrq0x0B();
                static void HandleIrq0x0C();
                static void HandleIrq0x0D();
                static void HandleIrq0x0E();
                static void HandleIrq0x0F(); 
                static void HandleIrq0x31(); 
                static void HandleIrq0x80(); 
                static common::uint32_t HandleIrq(common::uint8_t irq, common::uint32_t esp);
                common::uint32_t DoHandleIrq(common::uint8_t irq, common::uint32_t esp);
            public:
                IrqManager(TaskManager* taskManager, common::uint8_t hwIrqOffset, Gdt* gdt);
                ~IrqManager();
                common::uint8_t HwIrqOffset();
                void Activate();
                void Deactivate(); 
        };

        class IrqHandler {
            protected:
                common::uint8_t irq;
                IrqManager* irqs;
                IrqHandler(common::uint8_t irq, IrqManager* irqs);
                ~IrqHandler();
            public:
                virtual common::uint32_t HandleIrq(common::uint32_t esp);
        };
    }
}
