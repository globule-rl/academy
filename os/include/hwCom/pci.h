#pragma once

#include <drivers/driver.h>
#include <hwCom/interrupts.h>

/*  pci: peripheral component interconnect
        pcidd: pci device descriptor -> ids
        pcic: pci controller 
            -> bar: base address register 
                0 memMapping 1 inp/output
            -> drivers, irq
*/
namespace os {
    namespace hwCom {
        class Pcidd {
            public:
                common::uint32_t irq;
                Pcidd();
                ~Pcidd();
        };
        class Pcic {
            Pcic();
            ~Pcic();
            public:
                /* dev.device_id
                        case 0x1022 amd case 0x2000 am79c973
                        case 0x8086 intel
                    dev.class_id
                        case 0x03 graphics 0x00 vga
                */
                drivers::Driver* GetDriver(Pcidd dev, IrqManager* irqs);
        };
    }
}
