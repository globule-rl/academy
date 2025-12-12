#include <drivers/amdEther.h>

using namespace os::common;
using namespace os::drivers;
using namespace os::hwCom;

/* 16-byte aligned: 16 multiples 0-15 16 bytes
    send/recv every 8 byte/1 buffer in 16k kilobytes
        sendBufDesc: in 32-bit 0xF -> 0xFFFFFFF0 8 bytes 1hex=4bit 2hex=8bit/1byte 
            16hex=8bytes, mask low 4-bit
    256: 0X100 0-255
        % reminder lower byte
        / quotient upper byte
    reg: 20->32-bit mode, 0->reset, lower+higher 16-bit
    regAddrPort = pcid->portBase+0x12
        initialize 
*/ 
// ---------------- amd :Driver ----------------------
Amd::Amd(Pcidd* pcid, IrqManager* irqs) 
: Driver(), 
    IrqHandler(pcid->irq+irqs->HwIrqOffset(), irqs),
    MacAddr0Port(pcid->portBase),
    MacAddr2Port(pcid->portBase+0x02),
    MacAddr4Port(pcid->portBase+0x04),
    regAddrPort(pcid->portBase+0x12),
    regDatPort(pcid->portBase+0x10),
    busCtlRegDatPort(pcid->portBase+0x16),
    resetPort(pcid->portBase+0x14) {
    this->rawDatHandler = 0;
    sendBufDesc = (BufDesc*)(((uint32_t)&sendBuffDesc[0] + 15) & ~((uint32_t)0x0F));
    recvBufDesc = (BufDesc*)(((uint32_t)&recvBuffDesc[0] + 15) & ~((uint32_t)0x0F));
    curSendBuf = 0;
    curRecvBuf = 0;
    for (uint8_t i=0; i<8; i++) {
        sendBufDesc[i].addr = ((uint32_t)&sendBuff[i]+15) &~((uint32_t)0xF);
        sendBufDesc[i].flag = 0x7FF | 0xF000;    
        recvBufDesc[i].addr = ((uint32_t)&recvBuff[i]+15) &~((uint32_t)0xF);
        recvBufDesc[i].flag = 0x7FF | 0xF000; 
    }

    initBlock.logicAddr = 0;
    initBlock.sendBufDescAddr = (uint32_t)sendBufDesc;
    initBlock.recvBufDescAddr = (uint32_t)recvBufDesc;
    initBlock.mode = 0x0000;

    uint16_t mac0Val = MacAddr0Port.Read();
    uint16_t mac2Val = MacAddr2Port.Read();
    uint16_t mac4Val = MacAddr4Port.Read();
    uint64_t Mac0 = mac0Val % 256;
    uint64_t Mac1 = mac0Val / 256;
    uint64_t Mac2 = mac2Val % 256;
    uint64_t Mac3 = mac2Val / 256;
    uint64_t Mac4 = mac4Val % 256;
    uint64_t Mac5 = mac4Val / 256;
    uint64_t Mac = Mac5<<40 | Mac4<<32 | Mac3<<24 | Mac2<<16 | Mac1<<8 | Mac0;
    initBlock.phyAddr = Mac;
    initBlock.reserved3 = 0;
    initBlock.reserved1 = 0;
    initBlock.numSendBuf = 3;
    initBlock.reserved2 = 0;
    initBlock.numRecvBuf = 3;

    regAddrPort.Write(20);
    busCtlRegDatPort.Write(0x102);
    regAddrPort.Write(0);
    regDatPort.Write(0x04);
    regAddrPort.Write(1);
    regDatPort.Write((uint32_t)&initBlock & 0xFFFF);
    regAddrPort.Write(2);
    regDatPort.Write((uint32_t)&initBlock>>16 & 0xFFFF);

}
Amd::~Amd() {}
void Amd::SetIPAddr(uint32_t ip) {
    initBlock.logicAddr = ip;
}
uint32_t Amd::GetIPAddr() {
    return initBlock.logicAddr;
}


// ---------------- rawDataHandler ----------------------
RawDataHandler::RawDataHandler(Amd* amd) {
    this->amd = amd;
}
RawDataHandler::~RawDataHandler() {}