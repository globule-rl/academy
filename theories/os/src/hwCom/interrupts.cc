#include <hwCom/interrupts.h>

using namespace os::common;
using namespace os::hwCom;

void printf(char* str);
void printfHex(uint8_t);

IrqManager* IrqManager::activeIrqManager = 0;
IrqManager::GateDescriptor IrqManager::idt[256];

/* 0xFFFF = 2^16 - 1  >> 16  -> take higher 16 bits
    0x11 2^4 16, 17
    0x20 2^5 32, 
    0xA0 2^7=128 2^8=256 160 
    0x80 2^7 128
    3 binary 10
    256: 2^8
    protected mode 8 bytes in 32-bit sys: 
        0-1 idtHandler offset low, 2-3 selector, 4 always zero, 5 access, 6-7 idtHandler offset high
    gatedescriptor idt[256]:
        & 0xFFFF: idtHandler low 16-bit/0x20 
        (>>16)& 0xFFFF: idtHandler high 16-bit/0xA0
        access: desc_present descriptor present/valid | dpl/0-1 shift to 5-6 | type 0x0E 32-bit irq 0x0F trap
            bit pos: 7 present 5-6 dpl 0-4 type+reserved
                common usage: 0x8E 142
*/
void IrqManager::SetIdtEntry(uint8_t irq, uint16_t codeSeg, void (*idtHandler)(), uint8_t DPL, uint8_t DescriptorType) {
    idt[irq].idtHandlerAddrLowBits = ((uint32_t) idtHandler) & 0xFFFF;
    idt[irq].idtHandlerAddrHighBits = ((uint32_t) idtHandler>>16) & 0xFFFF;
    idt[irq].gdt_codeSegSelector = codeSeg;
    const uint8_t IDT_PRESENT = 0x80;
    idt[irq].access = IDT_PRESENT | ((DPL & 3) << 5) | DescriptorType;
    idt[irq].reserved = 0;
}
uint8_t IrqManager::HwIrqOffset() {return hwIrqOffset;}
// HandleIrq: global entry point asm/assembly calls it
uint32_t IrqManager::HandleIrq(uint8_t irq, uint32_t esp) {
    if (activeIrqManager) return activeIrqManager->DoHandleIrq(irq, esp);
    return esp;
}
/* 
    hwIrqOffset: 0x20 irq 0-15 -> remapping to 0x20-0x2F
    irq 0x20: timer schedule
        master 0x20 -> port 0x20 
            call register irqHandler first/keyboard
    eoi: end of irq, irq >= 0x28, hwIrq+8
        slave 0x20 -> port 0xA0
*/
uint32_t IrqManager::DoHandleIrq(uint8_t irq, uint32_t esp) {
    if (irqHandlers[irq]) {esp = irqHandlers[irq]->HandleIrq(esp);}
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
        entry: irq, code, idtHandler, dpl, type-gate
            -> idt[256] low high access/present/dpl/type/reserved
                -> set gate/idt as type
                    full: 10001110 
    python write 00 to 0F/13 in hex
        hex => {i:02X}
            for i in range(20): print(f"SetIdtEntry(0x{i:02X}, CodeSeg, &HandleException0x{i:02X}, 0, IDT_IRQ_GATE);")
            for i in range(16): print(f"SetIdtEntry(hwIrqOffset+0x{i:02X}, CodeSeg, &HandleIrq0x{i:02X}, 0, IDT_IRQ_GATE);")
    pic 4-bit: init cmd, remapping offset->IRO0 8, data->IRO2, EOI, mask all IRQs/unmask later w/ 0xFD/0xFF
        icw: initialization control word
        8-bit/1-byte each step x5
    limit 256*sizeof: 256 entries/0-255/intel x 8 bytes = 2048 bytes
        protected mode 8 bytes in 32-bit sys: 
            0-1 idtHandler offset low, 2-3 selector, 4 zero, 5 access, 6-7 idtHandler offset high
    pic I/0 port/0x20/0x21/0xA0/0xA1: 8-bit each write, 8bitslow
        idt entry 32-bit: 8 bytes per entry to cpu/mem/idt table
*/
IrqManager::IrqManager(TaskManager* taskManager, uint8_t hwIrqOffset, Gdt* gdt)
: picMasterCmdPort(0x20), picMasterDataPort(0x21), picSlaveCmdPort(0xA0), picSlaveDataPort(0xA1) {
    this->taskManager = taskManager;
    this->hwIrqOffset= hwIrqOffset;
    uint32_t CodeSeg = gdt->CodeSegSelector();

    const uint8_t IDT_IRQ_GATE = 0x0E;
    for (uint8_t i=255; i>0; --i) {
        SetIdtEntry(i, CodeSeg, &IrqIgnore, 0, IDT_IRQ_GATE);
        irqHandlers[i] = 0;
    }
    SetIdtEntry(0, CodeSeg, &IrqIgnore, 0, IDT_IRQ_GATE);
    irqHandlers[0] = 0;
    
    SetIdtEntry(0x00, CodeSeg, &HandleException0x00, 0, IDT_IRQ_GATE);
    SetIdtEntry(0x01, CodeSeg, &HandleException0x01, 0, IDT_IRQ_GATE);
    SetIdtEntry(0x02, CodeSeg, &HandleException0x02, 0, IDT_IRQ_GATE);
    SetIdtEntry(0x03, CodeSeg, &HandleException0x03, 0, IDT_IRQ_GATE);
    SetIdtEntry(0x04, CodeSeg, &HandleException0x04, 0, IDT_IRQ_GATE);
    SetIdtEntry(0x05, CodeSeg, &HandleException0x05, 0, IDT_IRQ_GATE);
    SetIdtEntry(0x06, CodeSeg, &HandleException0x06, 0, IDT_IRQ_GATE);
    SetIdtEntry(0x07, CodeSeg, &HandleException0x07, 0, IDT_IRQ_GATE);
    SetIdtEntry(0x08, CodeSeg, &HandleException0x08, 0, IDT_IRQ_GATE);
    SetIdtEntry(0x09, CodeSeg, &HandleException0x09, 0, IDT_IRQ_GATE);
    SetIdtEntry(0x0A, CodeSeg, &HandleException0x0A, 0, IDT_IRQ_GATE);
    SetIdtEntry(0x0B, CodeSeg, &HandleException0x0B, 0, IDT_IRQ_GATE);
    SetIdtEntry(0x0C, CodeSeg, &HandleException0x0C, 0, IDT_IRQ_GATE);
    SetIdtEntry(0x0D, CodeSeg, &HandleException0x0D, 0, IDT_IRQ_GATE);
    SetIdtEntry(0x0E, CodeSeg, &HandleException0x0E, 0, IDT_IRQ_GATE);
    SetIdtEntry(0x0F, CodeSeg, &HandleException0x0F, 0, IDT_IRQ_GATE);
    SetIdtEntry(0x10, CodeSeg, &HandleException0x10, 0, IDT_IRQ_GATE);
    SetIdtEntry(0x11, CodeSeg, &HandleException0x11, 0, IDT_IRQ_GATE);
    SetIdtEntry(0x12, CodeSeg, &HandleException0x12, 0, IDT_IRQ_GATE);
    SetIdtEntry(0x13, CodeSeg, &HandleException0x13, 0, IDT_IRQ_GATE);

    SetIdtEntry(hwIrqOffset+0x00, CodeSeg, &HandleIrq0x00, 0, IDT_IRQ_GATE);
    SetIdtEntry(hwIrqOffset+0x01, CodeSeg, &HandleIrq0x01, 0, IDT_IRQ_GATE);
    SetIdtEntry(hwIrqOffset+0x02, CodeSeg, &HandleIrq0x02, 0, IDT_IRQ_GATE);
    SetIdtEntry(hwIrqOffset+0x03, CodeSeg, &HandleIrq0x03, 0, IDT_IRQ_GATE);
    SetIdtEntry(hwIrqOffset+0x04, CodeSeg, &HandleIrq0x04, 0, IDT_IRQ_GATE);
    SetIdtEntry(hwIrqOffset+0x05, CodeSeg, &HandleIrq0x05, 0, IDT_IRQ_GATE);
    SetIdtEntry(hwIrqOffset+0x06, CodeSeg, &HandleIrq0x06, 0, IDT_IRQ_GATE);
    SetIdtEntry(hwIrqOffset+0x07, CodeSeg, &HandleIrq0x07, 0, IDT_IRQ_GATE);
    SetIdtEntry(hwIrqOffset+0x08, CodeSeg, &HandleIrq0x08, 0, IDT_IRQ_GATE);
    SetIdtEntry(hwIrqOffset+0x09, CodeSeg, &HandleIrq0x09, 0, IDT_IRQ_GATE);
    SetIdtEntry(hwIrqOffset+0x0A, CodeSeg, &HandleIrq0x0A, 0, IDT_IRQ_GATE);
    SetIdtEntry(hwIrqOffset+0x0B, CodeSeg, &HandleIrq0x0B, 0, IDT_IRQ_GATE);
    SetIdtEntry(hwIrqOffset+0x0C, CodeSeg, &HandleIrq0x0C, 0, IDT_IRQ_GATE);
    SetIdtEntry(hwIrqOffset+0x0D, CodeSeg, &HandleIrq0x0D, 0, IDT_IRQ_GATE);
    SetIdtEntry(hwIrqOffset+0x0E, CodeSeg, &HandleIrq0x0E, 0, IDT_IRQ_GATE);
    SetIdtEntry(hwIrqOffset+0x0F, CodeSeg, &HandleIrq0x0F, 0, IDT_IRQ_GATE);

    SetIdtEntry(            0x80, CodeSeg, &HandleIrq0x80, 0, IDT_IRQ_GATE);

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
/* cli: clear irq flag, disable maskable hw irq if=0, safe state
    sti: set irq flag, enable, renable before iret/optional if irqHandler 
*/
void IrqManager::Deactivate() {
    if (activeIrqManager == this) {activeIrqManager=0; asm("cli");}
}
IrqManager::~IrqManager() {Deactivate();}

void IrqManager::Activate() {
    if (activeIrqManager) activeIrqManager->Deactivate();
    activeIrqManager = this;
    asm("sti");
}

IrqHandler::IrqHandler(uint8_t irq, IrqManager* irqs) {
        this->irq = irq;
        this->irqs = irqs;
        irqs->irqHandlers[irq] = this;
}
IrqHandler::~IrqHandler() {
    if (irqs->irqHandlers[irq] == this) irqs->irqHandlers[irq] = 0;
}
uint32_t IrqHandler::HandleIrq(uint32_t esp) {return esp;}
