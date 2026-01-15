#pragma once

namespace os {
    namespace drivers {
        class Driver {
            public:
                Driver();
                ~Driver();
                virtual void Activate();
                virtual int Reset();
                virtual void Deactivate();
        };
        class DrvManager {
            public:
                Driver* drivers[265];
                int numDrvs;
            public:
                DrvManager();
                void AddDriver(Driver* drv);
                void ActivateAll();
        };
    }
}