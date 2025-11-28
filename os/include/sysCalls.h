#pragma once

#include <common/types.h>
#include <hwCom/interrupts.h>
#include <multiTasking.h>

namespace os {
    class SysCallHandler : public hwCom::IrqHandler {
        public:
            SysCallHandler(common::uint8_t irq, hwCom::IrqManager* irqs);
            ~SysCallHandler();
            virtual common::uint32_t HandleIrq(common::uint32_t esp);
    };
}