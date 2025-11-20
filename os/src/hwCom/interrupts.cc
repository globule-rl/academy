#include <hwCom/interrupts.h>

using namespace os;
using namespace os::common;
using namespace os::hwcom;

void printf(char* str);
void printfHex(uint8_t);

IrqManager* IrqManager::ActiveIrqManager = 0;
IrqManager::GateDescriptor IrqManager::idt[256];

/* 0xFFFF = 2^16 - 1  >> 16  -> take higher 16 bits
    0x11 2^4 16, 17
    0x20 2^5 32, 
    0xA0 2^7=128 2^8=256 160 
    0x80 2^7 128
    3 binary 10
    256: 2^8
    gatedescriptor idt[256]:
        & 0xFFFF: low 16-bit/0x20 
        (>>16)& 0xFFFF: higher 16-bit/0xA0
        access: desc_present descriptor present/valid | dpl/0-1 shift to 5-6 | type 0x0E 32-bit irq 0x0F trap
            bit pos: 7 present 5-6 dpl 0-4 type+reserved
            common: 0x8E 142
*/
void IrqManager::SetIdtEntry(
                uint8_t irq, 
                uint16_t codeSeg,
                void (*handler)(),
                uint8_t DPL,
                uint8_t DescriptorType) {
    idt[irq].handlerAddrLowBits = ((uint32_t) handler) & 0xFFFF;
    idt[irq].handlerAddrHighBits = ((uint32_t) handler>>16) & 0xFFFF;
    idt[irq].gdt_codeSegSelector = codeSeg;
    const uint8_t IDT_PRESENT = 0x80;
    idt[irq].access = IDT_PRESENT | ((DPL & 3) << 5) | DescriptorType;
    idt[irq].reserved = 0;
}
uint8_t IrqManager::HwIrqOffset() {return hwIrqOffset;}
/* HandleIrq: global entry point asm/assembly calls it
*/
uint32_t IrqManager::HandleIrq(os::common::uint8_t irq, os::common::uint32_t esp) {
    if (ActiveIrqManager) return ActiveIrqManager->DoHandleIrq(irq, esp);
    return esp;
}
/* 
    hwIrqOffset: 0x20 irq 0-15 -> remapping to 0x20-0x2F
    irq 0x20: timer schedule
        master 0x20 -> port 0x20 
            call register handler first/keyboard
    eoi: end of irq, irq >= 0x28, hwIrq+8
        slave 0x20 -> port 0xA0
*/
uint32_t IrqManager::DoHandleIrq(os::common::uint8_t irq, os::common::uint32_t esp) {
    if (handlers[irq]) {esp = handlers[irq]->HandleIrq(esp);}
    else if (irq != hwIrqOffset) {printf("UNHANDLED IRQ 0x"); printfHex(irq);}
    if (irq == hwIrqOffset) {esp = (uint32_t)taskManager->Schedule((CPUState*)esp);}
    if (irq >= hwIrqOffset && irq < hwIrqOffset+16) {
        picMasterCmdPort.Write(0x20);
        if (irq >= hwIrqOffset+8) picSlaveCmdPort.Write(0x20);
    }
    return esp;
}

/* port8bitslow: pic master 0x20 slave 0xA0
    0x0E: 1110 14 four lower bits in 32-bit
        entry: irq, code, handler, dpl, type-gate
            -> idt[256] low high access/present/dpl/type/reserved
                -> set gate/idt as type
                    full: 10001110 
    python write 00 to 0F/13 in hex
        hex => {i:02X}
            for i in range(16): print(f"SetIdtEntry(hwIrqOffset+{i:02X}, CodeSeg, &HandleIrq{i:02X}, 0, IDT_IRQ_GATE);")
            for i in range(20): print(f"SetIdtEntry({i:02X}, CodeSeg, &HandleException{i:02X}, 0, IDT_IRQ_GATE);")
    pic 4-bit: init cmd, remapping offset->IRO0 8, data->IRO2, EOI, mask all IRQs/unmask later w/ 0xFD/0xFF
        icw: initialization control word
        8-bit/1-byte each step x5
    limit 256*sizeof: 256 entries/0-255/intel x 8 bytes = 2048 bytes
        protected mode 8 bytes in 32-bit sys: 
            0-1 handler offset low, 2-3 selector, 4 zero, 5 access, 6-7 handler offset high
    pic I/0 port/0x20/0x21/0xA0/0xA1: 8-bit each write, 8bitslow
        idt entry 32-bit: 8 bytes per entry to cpu/mem/idt table
*/
IrqManager::IrqManager(os::TaskManager* taskManager, os::common::uint8_t hwIrqOffset, os::Gdt* gdt)
        : picMasterCmdPort(0x20),
          picMasterDataPort(0x21),
          picSlaveCmdPort(0xA0),
          picSlaveDataPort(0xA1) {
    this->taskManager = taskManager;
    this->hwIrqOffset= hwIrqOffset;
    uint32_t CodeSeg = gdt->CodeSegSelector();

    const uint8_t IDT_IRQ_GATE = 0x0E;
    for (uint8_t i=255; i>0; --i) {
        SetIdtEntry(i, CodeSeg, &IrqIgnore, 0, IDT_IRQ_GATE);
        handlers[i] = 0;
    }
    SetIdtEntry(0, CodeSeg, &IrqIgnore, 0, IDT_IRQ_GATE);
    handlers[0] = 0;
    
    SetIdtEntry(00, CodeSeg, &HandleException00, 0, IDT_IRQ_GATE);
    SetIdtEntry(01, CodeSeg, &HandleException01, 0, IDT_IRQ_GATE);
    SetIdtEntry(02, CodeSeg, &HandleException02, 0, IDT_IRQ_GATE);
    SetIdtEntry(03, CodeSeg, &HandleException03, 0, IDT_IRQ_GATE);
    SetIdtEntry(04, CodeSeg, &HandleException04, 0, IDT_IRQ_GATE);
    SetIdtEntry(05, CodeSeg, &HandleException05, 0, IDT_IRQ_GATE);
    SetIdtEntry(06, CodeSeg, &HandleException06, 0, IDT_IRQ_GATE);
    SetIdtEntry(07, CodeSeg, &HandleException07, 0, IDT_IRQ_GATE);
    SetIdtEntry(08, CodeSeg, &HandleException08, 0, IDT_IRQ_GATE);
    SetIdtEntry(09, CodeSeg, &HandleException09, 0, IDT_IRQ_GATE);
    SetIdtEntry(0A, CodeSeg, &HandleException0A, 0, IDT_IRQ_GATE);
    SetIdtEntry(0B, CodeSeg, &HandleException0B, 0, IDT_IRQ_GATE);
    SetIdtEntry(0C, CodeSeg, &HandleException0C, 0, IDT_IRQ_GATE);
    SetIdtEntry(0D, CodeSeg, &HandleException0D, 0, IDT_IRQ_GATE);
    SetIdtEntry(0E, CodeSeg, &HandleException0E, 0, IDT_IRQ_GATE);
    SetIdtEntry(0F, CodeSeg, &HandleException0F, 0, IDT_IRQ_GATE);
    SetIdtEntry(10, CodeSeg, &HandleException10, 0, IDT_IRQ_GATE);
    SetIdtEntry(11, CodeSeg, &HandleException11, 0, IDT_IRQ_GATE);
    SetIdtEntry(12, CodeSeg, &HandleException12, 0, IDT_IRQ_GATE);
    SetIdtEntry(13, CodeSeg, &HandleException13, 0, IDT_IRQ_GATE);

    SetIdtEntry(hwIrqOffset+00, CodeSeg, &HandleIrq00, 0, IDT_IRQ_GATE);
    SetIdtEntry(hwIrqOffset+01, CodeSeg, &HandleIrq01, 0, IDT_IRQ_GATE);
    SetIdtEntry(hwIrqOffset+02, CodeSeg, &HandleIrq02, 0, IDT_IRQ_GATE);
    SetIdtEntry(hwIrqOffset+03, CodeSeg, &HandleIrq03, 0, IDT_IRQ_GATE);
    SetIdtEntry(hwIrqOffset+04, CodeSeg, &HandleIrq04, 0, IDT_IRQ_GATE);
    SetIdtEntry(hwIrqOffset+05, CodeSeg, &HandleIrq05, 0, IDT_IRQ_GATE);
    SetIdtEntry(hwIrqOffset+06, CodeSeg, &HandleIrq06, 0, IDT_IRQ_GATE);
    SetIdtEntry(hwIrqOffset+07, CodeSeg, &HandleIrq07, 0, IDT_IRQ_GATE);
    SetIdtEntry(hwIrqOffset+08, CodeSeg, &HandleIrq08, 0, IDT_IRQ_GATE);
    SetIdtEntry(hwIrqOffset+09, CodeSeg, &HandleIrq09, 0, IDT_IRQ_GATE);
    SetIdtEntry(hwIrqOffset+0A, CodeSeg, &HandleIrq0A, 0, IDT_IRQ_GATE);
    SetIdtEntry(hwIrqOffset+0B, CodeSeg, &HandleIrq0B, 0, IDT_IRQ_GATE);
    SetIdtEntry(hwIrqOffset+0C, CodeSeg, &HandleIrq0C, 0, IDT_IRQ_GATE);
    SetIdtEntry(hwIrqOffset+0D, CodeSeg, &HandleIrq0D, 0, IDT_IRQ_GATE);
    SetIdtEntry(hwIrqOffset+0E, CodeSeg, &HandleIrq0E, 0, IDT_IRQ_GATE);
    SetIdtEntry(hwIrqOffset+0F, CodeSeg, &HandleIrq0F, 0, IDT_IRQ_GATE);

    SetIdtEntry(          0x80, CodeSeg, &HandleIrq80, 0, IDT_IRQ_GATE);

    picMasterCmdPort.Write(0x11);
    picSlaveCmdPort.Write(0x11);
    picMasterDataPort.Write(hwIrqOffset);
    picSlaveDataPort.Write(hwIrqOffset+8);
    picMasterDataPort.Write(0x04);
    picSlaveDataPort.Write(0x02);
    picMasterDataPort.Write(0x01);
    picSlaveDataPort.Write(0x01);
    picMasterDataPort.Write(0x00);
    picSlaveDataPort.Write(0x00);

    IdtPtr idt_ptr;
    idt_ptr.limit = 256*sizeof(GateDescriptor)-1;
    idt_ptr.base = (uint32_t)idt;
    asm volatile("lidt %0" : : "m" (idt_ptr));
}
IrqManager::~IrqManager() {Deactivate();}
/* cli: clear irq flag, disable maskable hw irq if=0, safe state
    sti: set irq flag, enable, renable before iret/optional if handler 
*/
void IrqManager::Deactivate() {
    if (ActiveIrqManager == this) {ActiveIrqManager=0; asm("cli");}
}
void IrqManager::Activate() {
    if (ActiveIrqManager) ActiveIrqManager->Deactivate();
    ActiveIrqManager = this;
    asm("sti");
}

IrqHandler::IrqHandler(IrqManager* irqManager, os::common::uint8_t irqNum) {
        this->irqNum = irqNum;
        this->irqManager = irqManager;
        irqManager->handlers[irqNum] = this;
}
IrqHandler::~IrqHandler() {
    if (irqManager->handlers[irqNum] == this) irqManager->handlers[irqNum] = 0;
}
uint32_t IrqHandler::HandleIrq(uint32_t esp) {return esp;}
