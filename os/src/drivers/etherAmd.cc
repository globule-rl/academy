#include <drivers/etherAmd.h>

using namespace os::common;
using namespace os::drivers;
using namespace os::hwCom;

Amd::Amd(Pcidd* dev, IrqManager* irqs) 
: Driver(), IrqHandler(dev->irq+irqs->HwIrqOffset(), irqs) {
    initBlock.logicAddr = 0;
}
Amd::~Amd() {}

void Amd::SetIPAddr(uint32_t ip) {
    initBlock.logicAddr = ip;
}

RawDataHandler::RawDataHandler(Amd* amd) {

}
RawDataHandler::~RawDataHandler() {}