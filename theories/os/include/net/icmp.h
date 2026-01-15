#pragma once

#include <common/types.h>
#include "ipv4.h"

/* icmp: internet control msg protocol
                        
*/

namespace os {
    namespace net {
        class Icmp {
            public:
                Icmp(Ipv4* ipv4);
                ~Icmp();
                void ReqEchoReplay(uint32_t gip);
        };
    }
}