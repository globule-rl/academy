#pragma once

#include <common/types.h>
#include <hwCom/port.h>

/* ata: advanced technology attachment
    sata: series
    pata: parallel
*/
namespace os {
    namespace drivers {
        class Sata {
            protected:
                bool master;
                hwCom::Port8Bit cmdPort;
            public:
                Sata(bool master, common::uint16_t portBase);
                ~Sata();
                void Identify();
                void Read28(common::uint32_t sector, common::uint32_t perSector);
                void Write28(common::uint32_t sector, common::uint8_t data, common::uint32_t perSector);
                void Flush();
        };
    }
}