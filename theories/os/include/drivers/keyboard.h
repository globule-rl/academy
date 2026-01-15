#pragma once

#include <hwCom/port.h>
#include <hwCom/interrupts.h>
#include <driver.h>

namespace os {
    namespace drivers {
        class KeyboardEvHandler {
            public:
                KeyboardEvHandler();
                virtual void OnKeyDown(char);
                virtual void OnKeyUp(char);
        };
        class KeyboardDriver: public Driver, public hwCom::IrqHandler {
            hwCom::Port8Bit cmdPort;
            hwCom::Port8Bit dataPort;
            KeyboardEvHandler* kEvHandler;
            public:
                KeyboardDriver(KeyboardEvHandler* kEvHandler, hwCom::IrqManager* irqs);
                ~KeyboardDriver();
                virtual void Activate();
                virtual uint32_t HandleIrq(uint32_t esp);
        };
    }
}