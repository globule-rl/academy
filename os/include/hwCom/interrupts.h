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
        python write 00 to 0F/13 in hex
            hex => {i:02X} 
            for i in range(16): print(f"static void HandleIrq0x{i:02X}();")
            for i in range(20): print(f"static void HandleException0x{i:02X}();")
    class IrgHandler;: forward declare cls
    virtual HandleIrq: polymophism override in derive class
    8259 pic i/o port irq mask:  master  cmd    port 0x0020
                                        data         0x0021
                                slave   cmd          0x00A0
                                        data         0x00A1
            real mode: master irq 0-7   offset 0x08 0x08-0x0F
                        slave irq 8-15          0x70 0x70-0x77
            protected mode: conflict w/ cpu exception
                 -> remapping 0-0xF -> 0x20-0x2F
    each hw irq needs its own tiny assembly entry/register/pin
        0x20-0x2F -> irq0 irq1-irq15 16
            exception 0x00-0x13 20
 */
namespace os {
    namespace hwcom {
        class IrgHandler;
        class IrqManager {
            friend class IrqHandler;
            protected:
                static IrqManager* ActiveIrqManager;
                IrqHandler* handlers[256];
                TaskManager* taskManager;

                struct GateDescriptor {
                    os::common::uint16_t handlerAddrLowBits;
                    os::common::uint16_t handlerAddrHighBits;
                    os::common::uint16_t gdt_codeSegSelector;
                    os::common::uint8_t access;
                    os::common::uint8_t reserved;
                } __attribute__((packed));
                static GateDescriptor idt[256]; 
                struct IdtPtr {
                    os::common::uint16_t limit;
                    os::common::uint32_t base;
                } __attribute__((packed));
                static void SetIdtEntry(
                            os::common::uint8_t irq,
                            os::common::uint16_t codeSeg,
                            void (*handler)(),
                            os::common::uint8_t DPL, 
                            os::common::uint8_t DescriptorType);
                            
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

                Port8BitSlow picMasterCmdPort;
                Port8BitSlow picMasterDataPort;
                Port8BitSlow picSlaveCmdPort;
                Port8BitSlow picSlaveDataPort;
                os::common::uint8_t hwIrqOffset;
                static os::common::uint32_t HandleIrq(os::common::uint8_t irq, os::common::uint32_t esp);
                os::common::uint32_t DoHandleIrq(os::common::uint8_t irq, os::common::uint32_t esp);

            public:
                IrqManager(os::TaskManager* taskManager, os::common::uint8_t hwIrqOffset, os::Gdt* gdt);
                ~IrqManager();
                os::common::uint8_t HwIrqOffset();
                void Activate();
                void Deactivate(); 
        };

        class IrqHandler {
            protected:
                os::common::uint8_t irqNum;
                IrqManager* irqManager;
                IrqHandler(IrqManager* irqManager, os::common::uint8_t irqNum);
                ~IrqHandler();
            public:
                virtual os::common::uint32_t HandleIrq(os::common::uint32_t esp);
        };
    }
}
