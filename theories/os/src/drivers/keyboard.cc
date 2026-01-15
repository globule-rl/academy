#include <drivers/keyboard.h>

using namespace os::common;
using namespace os::drivers;
using namespace os::hwCom;

KeyboardEvHandler::KeyboardEvHandler() {}
void KeyboardEvHandler::OnKeyDown(char) {}
void KeyboardEvHandler::OnKeyUp(char) {}


KeyboardDriver::KeyboardDriver(KeyboardEvHandler* kEvHandler, IrqManager* irqs)
: IrqHandler(0x21, irqs),
    cmdPort(0x64), dataPort(0x60) {
        this->kEvHandler = kEvHandler;
}
KeyboardDriver::~KeyboardDriver() {}

/* busy-wait on parallel
        until cmd status register lsb/bit0 0
            continuous reading/discarding data
    fifo drain device output
    0xAE: activate interrupts
    0x20: read cmd byte
    0x60: set cmd byte
*/
void KeyboardDriver::Activate() {
    while (cmdPort.Read() & 0x1) dataPort.Read();
    cmdPort.Write(0xAE);
    cmdPort.Write(0x20);
    uint8_t status = (dataPort.Read() | 1) | ~0x10;
    cmdPort.Write(0x60);
    dataPort.Write(status);
    dataPort.Write(0xF4);
}

void printf(char*);
void printfHex(uint8_t);

/* for i in range(11): 
    char = str(i) 
    print(f"case 0x{i+1:02X}: kEvHandler->OnKeyDown('{char}'); break;")
    qwert yuiop
    asdf ghjkl
    zxcv bnm,./
*/
uint32_t KeyboardDriver::HandleIrq(uint32_t esp) {
    uint8_t key = dataPort.Read();
    if (kEvHandler == 0) return esp;
    if (key < 0x80) {
        switch (key) {
            case 0x02: kEvHandler->OnKeyDown('1'); break;
            case 0x03: kEvHandler->OnKeyDown('2'); break;
            case 0x04: kEvHandler->OnKeyDown('3'); break;
            case 0x05: kEvHandler->OnKeyDown('4'); break;
            case 0x06: kEvHandler->OnKeyDown('5'); break;
            case 0x07: kEvHandler->OnKeyDown('6'); break;
            case 0x08: kEvHandler->OnKeyDown('7'); break;
            case 0x09: kEvHandler->OnKeyDown('8'); break;
            case 0x0A: kEvHandler->OnKeyDown('9'); break;
            case 0x0B: kEvHandler->OnKeyDown('10'); break;

            case 0x10: kEvHandler->OnKeyDown('q'); break;
            case 0x11: kEvHandler->OnKeyDown('w'); break;
            /* */

            case 0x1C: kEvHandler->OnKeyDown('\n'); break;
            case 0x39: kEvHandler->OnKeyDown(' '); break;
            default: {
                printf("Keyboard 0x");
                printfHex(key);
                break;
            }
        };
    }
    return esp;
}