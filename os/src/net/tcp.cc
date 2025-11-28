#include <net/tcp.h>

using namespace os::common;
using namespace os::net;


Tcp::Tcp(Ipv4* ipv4) {

}
Tcp::~Tcp() {}

TcpSocket* Tcp::Listen(uint16_t port) {

}
void Tcp::Bind(TcpSocket* socket, TcpHandler* tcpHandler) {

}

TcpSocket::TcpSocket(Tcp* tcp) {

}
TcpSocket::~TcpSocket() {}

void TcpSocket::Send(uint8_t* data, uint16_t size) {

}