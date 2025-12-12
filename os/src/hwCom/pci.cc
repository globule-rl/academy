#include <hwCom/pci.h>

using namespace os::common;
using namespace os::drivers;
using namespace os::hwCom;

/* read data
        >>(8*0x0E%4)=8*14%4=16 shift 16-bit, 16-31, %4/every 4-bit*8=32 total
        extract upper 16-bit actual size
            lower -> type/size
    numFuncs
        &(1<<7): mask bit 7/lsb -> numFuncs
            0001 shift 7 bit to left/10000000 2^7 128
        bool: 0->1-func, non-zero->multiple funcs
            > hasFunc or no
    bar
        &0x7F: get lower 7bits
        headerTypes: header0/endpoint->6bars, header1/bridge->2bars
            maxbar: 
                (6-4*type) 6-4*0=6 6-4*1=2 bars
        if bar >= max 6, not within bound 0-5, no reading bar, early ret
        else read 0x10 0x14 ...
            decode type 1 i/o addr bits31-2 ~0x3/0xC clear bit0-1, bits2+ intact -> addr
                        0 mem addr bit31-4, >>1&0x3, extract bit1-2
                            addr width, prefetchable flag
*/

Pcidd::Pcidd() {}
Pcidd::~Pcidd() {}

Pcic::Pcic()
: cmdPort(0xCF8), dataPort(0xCFC)  {}
Pcic::~Pcic() {}

uint32_t Pcic::Read(uint16_t bus, uint16_t device, uint16_t func, uint32_t regOffset) {
    uint32_t id = 0x1<<31 | ((bus&0xFF)<<16) | ((device&0x1F)<<11) | ((func&0x07)<<8) | (regOffset&0xFC);
    cmdPort.Write(id);
    uint32_t res = dataPort.Read() >> (8*(regOffset%4)); 
    return res;
}
bool Pcic::DeviceHasFuncs(uint16_t bus, uint16_t device) {
    return Read(bus, device, 0, 0x0E) & (1<<7);
}
Pcidd Pcic::GetDeviceDesc(uint16_t bus, uint16_t device, uint16_t func) {
    Pcidd res;
    res.irq=Read(bus, device, func, 0x3c);
    res.bus=bus;
    res.device=device;
    res.func=func;
    res.vendor_id=Read(bus, device, func, 0x00);
    res.device_id=Read(bus, device, func, 0x02);
    res.class_id=Read(bus, device, func, 0x0b);
    res.subclass_id=Read(bus, device, func, 0x0a);
    res.interface_id=Read(bus, device, func, 0x09);
    res.revision=Read(bus, device, func, 0x08); 
    return res;
}
BaseAddrReg Pcic::GetBar(uint16_t bus, uint16_t device, uint16_t func, uint16_t bar) {
    BaseAddrReg res;
    uint32_t headerType = Read(bus, device, func, 0x0E)&0x7F;
    int maxBar = 6-(4*headerType);
    if (bar >= maxBar) return res;
    uint32_t barVal = Read(bus, device, func, 0x10+4*bar);
    res.type = (barVal&0x01) ? InpOut : MemMapping;
    if (res.type == InpOut) {
        res.addr = (uint8_t*)(barVal&~0x03);
        res.prefetchable = false;
    } else {
        switch ((barVal>>1)&0x3) {
            case 0: //00 32-bit addr
            case 1: //01 20-bit/reserved
            case 2: //10 64-bit addr
            break;
        }
    }
    return res;
}
void Pcic::SelectDrivers(DrvManager* drvManager, IrqManager* irqs) {
    for (int bus=0; bus<8; bus++) {
        for (int device=0; device<32; device++) {
            int numFuncs = DeviceHasFuncs(bus, device) ? 8 : 1;
            for (int func=0; func<numFuncs; func++) {
                Pcidd pcid = GetDeviceDesc(bus, device, func);
                for (int baseAddrReg=0; baseAddrReg<6; baseAddrReg++) {
                    BaseAddrReg bar = GetBar(bus, device, func, baseAddrReg);
                    if (bar.addr && bar.type == InpOut) pcid.portBase = (uint32_t)bar.addr;
            }     
            }
        }
    }
}