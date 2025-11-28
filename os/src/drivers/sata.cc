#include <drivers/sata.h>

using namespace os::common;
using namespace os::drivers;

Sata::Sata(bool master, uint16_t portBase)
: cmdPort(portBase + 0x206) {
    this->master = master;
}
Sata::~Sata(){}

void Sata::Identify() {
    cmdPort.Write(0xEC);
}

void Sata::Read28(uint32_t sector, uint32_t perSector) {
    cmdPort.Write(0x20);
}

void Sata::Write28(uint32_t sector, uint8_t data, uint32_t perSector) {
    cmdPort.Write(0x30);
}

void Sata::Flush() {
    cmdPort.Write(0xE7);
}