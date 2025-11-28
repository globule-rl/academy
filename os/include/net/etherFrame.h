#pragma once

#include <drivers/etherAmd.h>
#include <hwCom/interrupts.h>

/* ethernet: physical ethernet port/for internet
        efp: ethernet frame provider
    mac header 14 bytes
        dest mac addr
        src mac add
        etherType 
            payload 46-1500 bytes
                checksum 4 bytes 00  20 20 3A
                    -> total 64-1518 bytes
    */
namespace os {
    namespace net {
        class Efp: public drivers::RawDataHandler {
            public:
                Efp(drivers::Amd* amd);
                ~Efp();
        };
    }
}