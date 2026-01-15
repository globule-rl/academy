#pragma once

#include <common/types.h>
#include <drivers/driver.h>
#include <hwCom/interrupts.h>
#include <hwCom/pci.h>
#include <hwCom/port.h>


/* amd: amd am79c973 ether card
  init block: 
    mem addr: logic/64, phy/48
    descriptor/32: send, recv
    config/16: mode, reserved
    buffer count/4: unused/reserved, num/store 0-15
        -> how many of each type
        -> pack small int into 16-bit bitfield
    mem: send/recvBuff 2d: (2*1024/2kb entries+15/overflow)*(8/each entry 8 bytes) 
                -> 16kb
            uint8_t: 8 bit per ele
            desc: 2kb, one byte per desc
    reg: control/status reg to config/report device behavior
    bus control: communication data flow/timing/priorities/arbitration
*/
namespace os {
    namespace drivers {
        class RawDataHandler;
        class Amd: public Driver, public hwCom::IrqHandler {
            struct BufDesc {
                uint32_t addr;
                uint32_t flag;
                uint32_t flag2;
                uint32_t avail;
            }__attribute__((packed));

            RawDataHandler* rawDatHandler;
            BufDesc* sendBufDesc;
            BufDesc* recvBufDesc;
            uint8_t curSendBuf;
            uint8_t curRecvBuf;
            uint8_t sendBuff[2*1024+15][8];
            uint8_t recvBuff[2*1024+15][8];
            uint8_t sendBuffDesc[2*1024+15];
            uint8_t recvBuffDesc[2*1024+15];

            struct InitBlock {
                uint64_t logicAddr;
                uint32_t sendBufDescAddr;
                uint32_t recvBufDescAddr;
                uint16_t mode;
                uint64_t phyAddr: 48;
                uint16_t reserved3;
                unsigned reserved1: 4;
                unsigned numSendBuf: 4;
                unsigned reserved2: 4;
                unsigned numRecvBuf: 4;
            }__attribute__((packed));
            InitBlock initBlock;

            hwCom::Port16Bit MacAddr0Port;
            hwCom::Port16Bit MacAddr2Port;
            hwCom::Port16Bit MacAddr4Port;
            hwCom::Port16Bit regAddrPort;
            hwCom::Port16Bit regDatPort;
            hwCom::Port16Bit busCtlRegDatPort;
            hwCom::Port16Bit resetPort;

            public:
                Amd(hwCom::Pcidd* pcid, hwCom::IrqManager* irqs);
                ~Amd();
                void Send(uint8_t buf, int cnt);
                void Recv();
                void SetIPAddr(uint32_t);
                uint32_t GetIPAddr();
        };
        class RawDataHandler {
            protected:
                Amd* amd;
            public:
                RawDataHandler(Amd* amd);
                ~RawDataHandler();
        };
    }
}