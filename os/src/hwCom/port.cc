#include <hwCom/port.h>

using namespace os::common;
using namespace os::hwCom;

Port::Port(uint16_t port) {this->port = port;}
Port::~Port() {}

Port8Bit::Port8Bit(uint16_t port) : Port(port) {}
Port8Bit::~Port8Bit() {}
uint8_t Port8Bit::Read() {return Read8(port);}
void Port8Bit::Write(uint8_t data) {Write8(port, data);}

Port8BitSlow::Port8BitSlow(uint16_t port) : Port8Bit(port) {}
Port8BitSlow::~Port8BitSlow() {}
void Port8BitSlow::Write(uint8_t data) {Write8Slow(port, data);}

Port16Bit::Port16Bit(uint16_t port) : Port(port) {}
Port16Bit::~Port16Bit() {}
uint16_t Port16Bit::Read() {return Read16(port);}
void Port16Bit::Write(uint16_t data) {Write16(port, data);}

Port32Bit::Port32Bit(uint16_t port) : Port(port) {}
Port32Bit::~Port32Bit() {}
uint32_t Port32Bit::Read() {return Read32(port);}
void Port32Bit::Write(uint32_t data) {Write32(port, data);}