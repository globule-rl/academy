#pragma once

#include <common/types.h>
#include <hwCom/interrupts.h>
#include <multiTasking.h>

namespace os {
    class SysCallHandler : public hwCom::IrqHandler {
        public:
            SysCallHandler(uint8_t irq, hwCom::IrqManager* irqs);
            ~SysCallHandler();
            virtual uint32_t HandleIrq(uint32_t esp);
    };
}