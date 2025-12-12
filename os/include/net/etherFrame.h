#pragma once

#include <common/types.h>
#include <drivers/amdEther.h>
#include <hwCom/interrupts.h>

/* ethernet: physical ethernet port/for internet
        EtherFrame: ethernet frame provider
    mac header 14 bytes + presmble/sfd
        dest mac addr
        src mac add
        etherType: ipv4=0x0800, arp=0x0806
    payload 46-1500 bytes
    footer: 4 bytes, fcs/frame check seq
            crc32 checksum 
                     4 bytes 00 20 20 3A
        -> total 64-1518 bytes
    */
namespace os {
    namespace net {
        struct EtherHeader {
            uint64_t dstMac: 48;
            uint64_t srcMac: 48;
            uint16_t etherType;
        } __attribute__ ((packed));
        typedef uint32_t EtherFooter;
        class EtherFrame: public drivers::RawDataHandler {
            friend class EtherFHandler;
            protected:
                EtherFHandler* etherFHandler[65535];
            public:
                EtherFrame(drivers::Amd* amd);
                ~EtherFrame();
                uint32_t GetIPAddr();
                // uint64_t GetMacAddr();
                void Send(uint64_t dstMac, uint8_t* buf, uint32_t size);
        };
        class EtherFHandler {
            protected:
                EtherFrame* etherFrame;
                uint16_t etherType;
            public:
                EtherFHandler(EtherFrame* etherFrame, uint16_t etherType);
                ~EtherFHandler();
                uint32_t GetIPAddr();
                void Send(uint64_t dstMac, uint16_t etherType, uint8_t* buf, uint32_t size);
        };
    }
}