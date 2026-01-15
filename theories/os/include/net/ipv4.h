#pragma once

#include <common/types.h>
#include "etherFrame.h"
#include "arp.h"

/* ipv4: internet ipProtocol: tcp=6, udp=17, icmp=1
        internal layer 
            ipv4/routing icmp/echoing
     gip/gateaway ip + subnet

*/

namespace os {
    namespace net {
        struct Ipv4Header {
            uint32_t srcIp;
            uint32_t segLen;
        } __attribute__((packed));
        class Ipv4: public EtherFHandler {
            friend class Ipv4Handler;
            protected:
                Arp* arp;
                uint32_t gip;
                uint32_t subnet;
                Ipv4Handler* ipv4Handlers[255];
            public:
                Ipv4(EtherFrame* etherFrame, Arp* arp, uint32_t gip, uint32_t subnet);
                ~Ipv4();
                static uint16_t Checksum(uint16_t* buf, uint32_t size);
                void Send(uint32_t dstIp, uint8_t ipProtocol, uint8_t* data, uint32_t size);
        };
        class Ipv4Handler{
            protected:
                Ipv4* ipv4;
                uint8_t ipProtocol;
            public:
                Ipv4Handler(Ipv4* ipv4, uint8_t ipProtocol);
                ~Ipv4Handler();
                void Send(uint32_t dstIp, uint8_t* data, uint32_t len);
        };
    }
}