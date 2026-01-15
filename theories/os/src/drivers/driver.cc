#include <drivers/driver.h>

using namespace os::drivers;

Driver::Driver() {}
void Driver::Deactivate() {}
Driver::~Driver() {}
void Driver::Activate() {}
int Driver::Reset() {}

DrvManager::DrvManager() {numDrvs=0;}
void DrvManager::AddDriver(Driver* drv) {
    drivers[numDrvs] = drv;
    numDrvs++;
}
void DrvManager::ActivateAll() {
    for (int i=0; i<numDrvs; i++) drivers[i]->Activate();
}