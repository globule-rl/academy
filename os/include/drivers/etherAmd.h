#pragma once

#include <common/types.h>
#include <drivers/driver.h>
#include <hwCom/interrupts.h>
#include <hwCom/pci.h>
#include <hwCom/port.h>


/* am79c973
    mac addr
*/
namespace os {
    namespace drivers {
        class Amd;
        class RawDataHandler {
            protected:
            public:
                RawDataHandler(Amd* amd);
                ~RawDataHandler();
        };
        class Amd: public Driver, public hwCom::IrqHandler {
            struct InitBlock {
                common::uint64_t logicAddr;
            }__attribute__((packed));
            InitBlock initBlock;
            public:
                Amd(hwCom::Pcidd* dev, hwCom::IrqManager* irqs);
                ~Amd();
                void SetIPAddr(common::uint32_t);
        };
    }
}