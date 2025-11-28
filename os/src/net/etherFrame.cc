#include <net/etherFrame.h>

using namespace os::common;
using namespace os::drivers;
using namespace os::net;

Efp::Efp(Amd* amd)
: RawDataHandler(amd) {

}
Efp::~Efp() {}