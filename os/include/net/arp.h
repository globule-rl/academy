#pragma once

#include <common/types.h>
#include "etherFrame.h"

/* arp: address resolution protocol
        translate ip to mac   
    ethernet frame: local device
                        
*/

namespace os {
    namespace net {
        class Arp {
            public:
                Arp(EtherFrame* etherFrame);
                ~Arp();
                void BroadcastMacAddr(uint32_t gip);
                uint64_t ResolveMac(uint32_t ip);
        };
    }
}
