#include <sysCalls.h>

using namespace os;
using namespace os::common;
using namespace os::hwCom;

/* irq+offset: adjusted
        map syscall irq: 
            0x80 -> hw pic iqr0 2
*/
SysCallHandler::SysCallHandler(uint8_t irq, IrqManager* irqs)
: IrqHandler(irq+irqs->HwIrqOffset(), irqs) {}
SysCallHandler::~SysCallHandler() {}

void printf(char*);
/* esp: stack ptr
        eax: syscall num
        ebx/file descriptor/str to print 
        ecx/buffer ptr, edx/length -> param
            eax 4: sys_write
*/
uint32_t SysCallHandler::HandleIrq(uint32_t esp) {
    CPUState* cpu = (CPUState*)esp;
    switch(cpu->eax) {
        case 4: printf((char*)cpu->ebx); break;
        default: break;
    }
    return esp;
}
