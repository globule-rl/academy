#pragma once

#include <common/types.h>
#include "ipv4.h"

/* tcp: transmission control protocol provider
    application layer: send data smtp, ftp, http, dhcp
        internal layer: ipv4 icmp echo/routing
            transport layer: tcp/udp create seg
                network layer: ip addr packet 
                    link layer/frame: ethe, fiber, satellite
                        tcp reassemble packet, ck err, ack
    3-way handsake
        client SYN 
            server SYN|ACK
                client ACK
    header
    status
        closed client syn
            listen server syn|ack
                established client ack
                    closed
                        active client close/fin
                        passive server fin/ack
    sequence /flow control sliding window
        initial
        data recvd, ackd, delivd
                            not delivd
                    not ackd
    checksum: + sum => 0xFFFF
    16-bit/2-byte: 1 word
    20 bytes: 5-15 32-bit/4-byte words -> 4 bit
    
*/

namespace os {
    namespace net {
        class Tcp;
        class TcpSocket;
        class TcpHandler {

        };
        class Tcp {
            public:
                Tcp(Ipv4* ipv4);
                ~Tcp();
                virtual TcpSocket* Listen(int16_t port);
                virtual void Bind(TcpSocket* socket, TcpHandler* tcpHandler);
        };
        class TcpSocket {
            public:
                TcpSocket(Tcp* tcp);
                ~TcpSocket();
                void Send(uint8_t* data, uint16_t size);
        };
    }
}