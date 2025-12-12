#include <net/etherFrame.h>

using namespace os::common;
using namespace os::drivers;
using namespace os::net;

// ---------------- etherFrameProvider :rawDataHandler/amd ----------------------
EtherFrame::EtherFrame(Amd* amd)
: RawDataHandler(amd) {}
EtherFrame::~EtherFrame() {}
uint32_t EtherFrame::GetIPAddr() {
    return amd->GetIPAddr();
}
void EtherFrame::Send(uint64_t dsiMac, uint8_t* buf, uint32_t size) {}


// ---------------- etherFrameHandler ----------------------
EtherFHandler::EtherFHandler(EtherFrame* etherFrame, uint16_t etherType) {
    this->etherFrame = etherFrame;
}
EtherFHandler::~EtherFHandler() {}
uint32_t EtherFHandler::GetIPAddr() {
    return etherFrame->GetIPAddr();
}
void EtherFHandler::Send(uint64_t dsiMac, uint16_t etherType, uint8_t* buf, uint32_t size) {}