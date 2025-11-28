#pragma once

#include <common/types.h>
#include <hwCom/port.h>
#include <hwCom/interrupts.h>
#include <driver.h>

/* intel 8042 ps/2 mouse controller 6-pin DIN/irq give cmd 100Hz, not usb/cpu give cmd 125Hz
    packets: 3 bytes
        0 flags + signs + btns
            bit7 Y overflow, bit6 X overflow/1=lost data, bit5: Y sign/1=neg, bit4: X sign/1=neg
            bit3 always 1, bit2: mid, bit1: right, bit0: left 
        1 X
        2 Y
        (3 Z)
        0x09 0000 1001 lsb bit0->always 1, bit3->1 left pressed
        0x0A +10 X right
        0xFF -1 Y down
    datasheet: https://www.eecg.utoronto.ca/~pc/courses/241/DE1_SoC_cores/ps2/ps2.html
*/
namespace os {
    namespace drivers {
        class MouseEvHandler {
            public:
                MouseEvHandler();
                virtual void OnActivate();
                virtual void OnMouseDown(common::uint8_t btn);
                virtual void OnMouseUp(common::uint8_t btn);
                virtual void onMouseMove(int x, int y);
        };
        class MouseDriver: public Driver, public hwCom::IrqHandler {
            hwCom::Port8Bit cmdPort;
            hwCom::Port8Bit dataPort;
            common::uint8_t buffer[3];
            common::uint8_t offset;
            common::uint8_t btns;
            MouseEvHandler* mEvHandler;
            public:
                MouseDriver(MouseEvHandler* mEvHandler, hwCom::IrqManager* irqs);
                ~MouseDriver();
                virtual void Activate();
                virtual common::uint32_t HandleIrq(common::uint32_t esp);
        };
    }
}
