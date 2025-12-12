#pragma once

#include <drivers/driver.h>
#include <hwCom/interrupts.h>
#include <memManagement.h>

/*  pci: peripheral component interconnect, i/o port
        pcidd: pci device descriptor -> ids
        pcic: pci controller 
    bar: base address register 
            0 memMapping 1 inp/output
            -> drivers, irq
    bus: 0-7 
        device:0-31 phy slot 
        func: logical device, one bus/slot->8 funcs 0-7
        bar/base addr register: 6
            config/specify where i/o port/mem mapped
        
    pcid.device_id
        case 0x1022 amd 
            case 0x2000 am79c973
        case 0x8086 intel
    pcid.class_id
        case 0x03 graphics 0x00 vga
*/
namespace os {
    namespace hwCom {
        enum BarType {
            MemMapping = 0,
            InpOut = 1
        };
        class BaseAddrReg {
            public:
                bool prefetchable;
                uint8_t* addr;
                uint32_t size;
                BarType type;
        };
        class Pcidd {
            public:
                uint32_t irq;

                uint16_t bus;
                uint16_t device;
                uint16_t func;
                uint32_t portBase;

                uint16_t vendor_id;
                uint16_t device_id;
                uint16_t class_id;
                uint16_t subclass_id;
                uint16_t interface_id;
                uint16_t revision;

                Pcidd();
                ~Pcidd();
        };
        class Pcic {
            private:
                Port32Bit cmdPort;
                Port32Bit dataPort;
            public:
                Pcic();
                ~Pcic();
                uint32_t Read(uint16_t bus, uint16_t device, uint16_t func, uint32_t regOffset);
                bool DeviceHasFuncs(uint16_t bus, uint16_t device);
                Pcidd GetDeviceDesc(uint16_t bus, uint16_t device, uint16_t func);
                BaseAddrReg GetBar(uint16_t bus, uint16_t device, uint16_t func, uint16_t bar);
                void SelectDrivers(DrvManager* drvManager, IrqManager* irqs);
                Driver* GetDriver(Pcidd pcid, IrqManager* irqs);
        };
    }
}
