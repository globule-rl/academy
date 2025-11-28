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
                Arp(Efp* etherFrame);
                ~Arp();
                void BroadcastMacAddr(uint32_t gip);
        };
    }
}
