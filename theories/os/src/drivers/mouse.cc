#include <drivers/mouse.h>

using namespace os::common;
using namespace os::drivers;
using namespace os::hwCom;


MouseEvHandler::MouseEvHandler() {}
void MouseEvHandler::OnActivate() {}
void MouseEvHandler::OnMouseDown(uint8_t btn) {}
void MouseEvHandler::OnMouseUp(uint8_t btn) {}
void MouseEvHandler::onMouseMove(int x , int y) {}

MouseDriver::MouseDriver(MouseEvHandler* mEvHandler, IrqManager* irqs)
: IrqHandler(0x2C, irqs), Driver(),
    cmdPort(0x64), dataPort(0x60) {
        this->mEvHandler = mEvHandler;
}
MouseDriver::~MouseDriver() {}

/* cmd:
        0xA8: enable snd port
        0x20: read config 'byte0' of internal ram
            8-bit bit5:port clk
        0x60 | 2: 0000 0010 bit1-> enable IRQ12
            0x60: read next config/status/data 'byte0' of internal ram
        0xD4: next/data byte to port input buffer
        0xF4: pulse output line/bit0 reset
    output: A20 gate
    datasheet: https://wiki.osdev.org/I8042_PS/2_Controller
 */
void MouseDriver::Activate() {
    offset = 0;
    btns = 0;
    if (mEvHandler != 0) mEvHandler->OnActivate();
    cmdPort.Write(0xA8);
    cmdPort.Write(0x20);
    uint8_t status = dataPort.Read() | 2;
    cmdPort.Write(0x60);
    dataPort.Write(status);
    cmdPort.Write(0xD4);
    dataPort.Write(0xF4);
    dataPort.Read();
}

/* data -> buffer -> offset==0: full cycle -> move
    detect btn changes
        btn: i+1 -> 1 2 3
            bit2: mid, bit1: right, bit0: left
    btns = buffer[0]: save byte0 to btn state/buffer
        0x07: 0000 0111 clear bits 3-7, only store btn state
*/
uint32_t MouseDriver::HandleIrq(uint32_t esp) {
    uint8_t status = cmdPort.Read();
    if (!(status & 0x20)) return esp;
    if (mEvHandler == 0) return esp;
    buffer[offset] = dataPort.Read();
    offset = (offset+1) % 3;
    if (offset == 0) {
        if (buffer[1] != 0 || buffer[2] != 0) {
            mEvHandler->onMouseMove((int8_t)buffer[1], -((int8_t)buffer[2]));
        }
        for (uint8_t i=0; i<3; i++) {
            bool wasPressed = btns & (0x1<<i);
            bool nowPressed = buffer[0] & (0x1<<i);
            if (nowPressed != wasPressed) {
                if (wasPressed) mEvHandler->OnMouseUp(i+1);
                else mEvHandler->OnMouseDown(i+1);
            }
        }
        btns = buffer[0] & 0x07;
    }
    return esp;
}
