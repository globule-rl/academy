#pragma once

#include <common/types.h>
#include "ipv4.h"

/* udp: user datagram protocol
*/

namespace os {
    namespace net {
        class Udp {
            public:
                Udp(Ipv4* ipv4);
                ~Udp();
        };
    }
}