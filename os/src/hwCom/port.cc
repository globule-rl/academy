#include <hwCom/port.h>

using namespace os::common;
using namespace os::hwcom;

Port::Port(uint16_t portnum) {this->portnum = portnum;}
Port::~Port() {}

Port8Bit::Port8Bit(uint16_t portnum) : Port(portnum) {}
Port8Bit::~Port8Bit() {}
uint8_t Port8Bit::Read() {return Read8(portnum);}
void Port8Bit::Write(uint8_t data) {Write8(portnum, data);}

Port8BitSlow::Port8BitSlow(uint16_t portnum) : Port8Bit(portnum) {}
Port8BitSlow::~Port8BitSlow() {}
void Port8BitSlow::Write(uint8_t data) {Write8Slow(portnum, data);}

Port16Bit::Port16Bit(uint16_t portnum) : Port(portnum) {}
Port16Bit::~Port16Bit() {}
uint16_t Port16Bit::Read() {return Read16(portnum);}
void Port16Bit::Write(uint16_t data) {Write16(portnum, data);}

Port32Bit::Port32Bit(uint16_t portnum) : Port(portnum) {}
Port32Bit::~Port32Bit() {}
uint32_t Port32Bit::Read() {return Read32(portnum);}
void Port32Bit::Write(uint32_t data) {Write32(portnum, data);}