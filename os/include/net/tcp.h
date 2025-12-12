#pragma once

#include <common/types.h>
#include "ipv4.h"
#include <memManagement.h>

/* pipline/class inheritance:
    tcpSocket <- tcp 
        <- ipv4Handler <- ipv4 
            <- etherFHandler <- etherF
                <- rawDataHandler <- amd/raw data
                    <- pci port <- eax/edx reg write
    provider: friend class handler
            socket: friend class provider
            amd has pointer to raw_data 
        handler has access to all provider, then to next pipe
        provider has access to all socket, not the other way around
    Tcp: transmission control ipProtocol provider
    application layer: send data smtp, ftp, http, dhcp
        internal layer: ipv4 icmp echo/routing
            transport layer: tcp/udp create seg
                network layer: ip addr packet 
                    link layer/frame: ethe, fiber, satellite
                        tcp reassemble packet, ck err, ack
    ipProtocol
        3-way handsake: client SYN, server SYN|ACK, client ACK
        psh: push immediately, telnet/ssh/msg
        urg: urgent
    header/pseudoHeader: ports, seq, ack no., len, flag, win size, checksum...
        -> header len 32-bit words
        -> option mss: max seg size
        -> calc checksum: pseudo+header+data/payload+checksum = 0xFFFF
    status
        closed client syn, listen server syn|ack, established client ack,
            closed, active client close/fin, passive server fin/ack
    sequence /flow control sliding window
        initial
        data recvd, ackd, delivd, not delivd, not ackd
    16-bit/2-byte: 1 word
    20 bytes: 5-15 32-bit/4-byte words -> 4 bit
    
*/

namespace os {
    namespace net {
        enum TcpSocketState {
            LISTEN,
            CLOSED,
            ESTABLISHED
        };
        enum TcpFlag {
            FIN=1,
            SYN=2,
            RST=4,
            PSH=8,
            ACK=16
        };
        struct TcpPseudoHeader {
            uint32_t srcIp;
            uint32_t dstIp;
            // uint8_t reserved;
            uint8_t ipProtocol;
            uint16_t segLen;
        } __attribute((packed));
        struct TcpHeader {
            uint16_t srcPort;
            uint16_t dstPort;
            uint32_t seq;
            uint32_t ack;
            uint8_t headerLenWord:4;
            uint8_t flags;
            uint16_t winSize;
            uint16_t urgPtr;
            uint8_t reserved:4;
            uint32_t options;
            uint16_t checksum;
        } __attribute((packed));
        class Tcp;
        class TcpSocket {
            friend class Tcp;
            protected:
                TcpSocketState state;
                Tcp* tcp;
                uint32_t srcIp;
                uint16_t srcPort;
                uint16_t dstIp;
                uint16_t dstPort;
                uint32_t seq;
                uint32_t ack;
            public:
                TcpSocket(Tcp* Tcp);
                ~TcpSocket();
                virtual void Send(uint8_t* data, uint16_t size);
        };
        class TcpHandler;
        class Tcp: Ipv4Handler {
            protected:
                TcpSocket* sockets[65535];
                uint16_t numSockets;
                uint8_t freePort;
            public:
                Tcp(Ipv4* ipv4);
                ~Tcp();
                virtual TcpSocket* Listen(int16_t port);
                virtual bool OnIPRevd(uint32_t srcIp_BE, uint32_t dstIp_BE, uint8_t* ipPayload, uint32_t size);
                void Send(TcpSocket* socket, uint8_t* data, uint16_t size, uint16_t flags);
                virtual void Bind(TcpSocket* socket, TcpHandler* tcpHandler);
        };
        class TcpHandler {

        };
    }
}