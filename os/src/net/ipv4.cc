# include <net/ipv4.h>

using namespace os::common;
using namespace os::net;

// ---------------- ipv4 :etherFrame ----------------------
/* checksum: header+payload+checksum->0xFFFF
        /size: in pairs, switch lower and upper 8
        size odd: last byte -> upper 8
        over 16-bit: align 16-bit, + -> carry-over
    ret NOT/complement, switch upper and lower 8
*/
Ipv4::Ipv4(EtherFrame* etherFrame, Arp* arp, uint32_t gip, uint32_t subnet)
: EtherFHandler(etherFrame, 0x800) {
    for (int i=0; i<255; i++) ipv4Handlers[i]=0;
    this->arp = arp;
    this->gip = gip;
    this->subnet = subnet;
}
Ipv4::~Ipv4() {}
uint16_t Ipv4::Checksum(uint16_t* buf, uint32_t size) {
    uint32_t tmp=0;
    for (int i=0; i<size/2; i++) tmp += ((buf[i]&0xFF00)>>8 | (buf[i]&0x00FF)<<8);
    if (size%2) tmp += ((uint16_t)(char*)buf[size-1]) << 8;
    while (tmp&0xFFFF0000) tmp=(tmp>>16)+(tmp&0xFFFF);
    return ((~tmp&0xFF00)>>8) | ((~tmp&0x00FF)<<8); 
}
void Ipv4::Send(uint32_t dstIp, uint8_t ipProtocol, uint8_t* data, uint32_t size) {
    uint32_t segLen = sizeof(Ipv4Header) + size;
    uint8_t* buf = (uint8_t*)MemManager::activeMemManager->malloc(segLen);
    Ipv4Header* seg = (Ipv4Header*)buf;
    seg->srcIp = etherFrame->GetIPAddr();
    seg->segLen = segLen;
    uint8_t* datBuf = buf + sizeof(Ipv4Header);
    for (int i=0; i<size; i++) datBuf[i] = data[i];
    uint32_t routeIp = dstIp ? ((dstIp&subnet) == (seg->srcIp&subnet)) : gip;
    etherFrame->Send(arp->ResolveMac(routeIp), buf, segLen);
}



// ---------------- ipv4Handler ----------------------
Ipv4Handler::Ipv4Handler(Ipv4* ipv4, uint8_t ipProtocol) {
    this->ipv4 = ipv4;
    this->ipProtocol = ipProtocol;
    ipv4->ipv4Handlers[ipProtocol] = this;
}
Ipv4Handler::~Ipv4Handler() {}
void Ipv4Handler::Send(uint32_t dstIp, uint8_t* data, uint32_t len) {
    ipv4->Send(dstIp, ipProtocol, data, len);
}