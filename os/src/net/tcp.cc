#include <net/tcp.h>

using namespace os::common;
using namespace os::net;

// ---------------- socket ----------------------
TcpSocket::TcpSocket(Tcp* tcp) {
    this->tcp = tcp;
    state = CLOSED;
}
TcpSocket::~TcpSocket() {}
void TcpSocket::Send(uint8_t* data, uint16_t size) {
    while (state != ESTABLISHED) {}
    tcp->Send(this, data, size, PSH|ACK);
}


// ---------------- tcp :ipv4 ----------------------
/* 65535: 0xFFFF
    seg: header+data
     buf/start + sizeof(pseudo), after pseudo
    on recvd: srcPort->dstPort, dstPort->src
        switch case ack/fin/syn
    send: header
        socket/host/little -> network/big/ip
            srcport/local port -> network
            pseudo header len  -> network
        be32: seq, ack -> network
*/
Tcp::Tcp(Ipv4* ipv4) 
: Ipv4Handler(ipv4, 0x06){
    for (int i=0; i<65535; i++) sockets[i]=0;
    numSockets = 0;
    freePort = 1024;
}
Tcp::~Tcp() {}
TcpSocket* Tcp::Listen(int16_t port) {
    TcpSocket* socket = (TcpSocket*)MemManager::activeMemManager->malloc(sizeof(TcpSocket));
    if(socket != 0) {
        new (socket) TcpSocket(this);
        socket->state = LISTEN;
        socket->srcIp = ipv4->GetIPAddr();
        socket->srcPort = ((port&0xFF00)>>8) | ((port&0x00FF)<<8);
        sockets[numSockets++] = socket;
    }
    return socket;
}
bool Tcp::OnIPRevd(uint32_t srcIp_BE, uint32_t dstIp_BE, uint8_t* ipv4Payload, uint32_t size) {
    TcpHeader* seg = (TcpHeader*)ipv4Payload;
    uint16_t dstPort = seg->srcPort;
    uint16_t srcPort = seg->dstPort;
}
uint32_t Be32(uint32_t socket) {
    return ((socket&0x000000FF)<<24 | (socket&0x0000FF00)<<8 | (socket&0x00FF0000)>>8 | (socket&0xFF000000)>>24);
}
void Tcp::Send(TcpSocket* socket, uint8_t* data, uint16_t size, uint16_t flags) {
    uint16_t segLen = size + sizeof(TcpHeader);
    uint16_t total = segLen + sizeof(TcpPseudoHeader);
    uint8_t* buf = (uint8_t*)MemManager::activeMemManager->malloc(total);
    TcpPseudoHeader* pseudo = (TcpPseudoHeader*)buf;
    TcpHeader* seg = (TcpHeader*)(buf+sizeof(TcpPseudoHeader));
    uint8_t* datBuf = buf+sizeof(TcpHeader)+sizeof(TcpPseudoHeader);
    pseudo->srcIp = socket->srcIp;
    pseudo->dstIp = socket->dstIp;
    pseudo->ipProtocol = 0x0600;
    pseudo->segLen = (segLen&0x00FF)<<8 | (segLen&0xFF00)>>8;
    seg->srcPort = socket->srcPort;
    seg->dstPort = socket->dstPort;
    seg->seq = Be32(socket->seq); 
    seg->ack = Be32(socket->ack);
    seg->headerLenWord = sizeof(TcpHeader)/4;
    seg->flags = flags;
    seg->winSize = 0xFFFF;
    seg->urgPtr = 0;
    seg->reserved = 0;
    seg->options = ((flags & SYN)!=0) ? 0XB4050402 : 0;
    socket->seq += size;
    for (int i=0; i<size; i++) datBuf[i] = data[i];
    seg->checksum = 0;
    seg->checksum = Ipv4::Checksum((uint16_t*)buf, total);
    Ipv4Handler::Send(socket->dstIp, (uint8_t*)seg, segLen);
    MemManager::activeMemManager->free(buf);

}
void Tcp::Bind(TcpSocket* socket, TcpHandler* tcpHandler) {
}



