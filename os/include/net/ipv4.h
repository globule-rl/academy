#pragma once

#include <common/types.h>
#include "etherFrame.h"
#include "arp.h"

/* ipv4: internet protocol
    internal layer 
        ipv4/routing icmp/echoing
    gateaway ip + subnet
*/

namespace os {
    namespace net {
        class Ipv4 {
            public:
                Ipv4(Efp* etherFrame, Arp* arp, uint32_t gip, uint32_t subnet);
                ~Ipv4();
        };
    }
}