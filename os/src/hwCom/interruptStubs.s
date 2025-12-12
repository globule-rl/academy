

.set IRQ_BASE, 0x20

.section .text

/* h unsigned char, j unsigned int 
     IrqHandler(uint8_t irq, IrqManager* irqs);
    stub: 
        fix the stack: cpu exception push err, hw irq push nothing
            -> stub dummy 0 + vector num
        save register
        safe state/cli
        call irqHandler
        glue
*/
.extern _ZN2os5hwcom10IrqManager10HandlerIrqEhj

.macro HandleException num
.global _ZN2os5hwcom10IrqManager19HandleException\num\()Ev
_ZN2os5hwcom10IrqManager19HandleException\num\()Ev:
    movb $\num, $irq
    jmp int_bottom
.endm

.macro HandleIrq num
.global _ZN2os5hwcom10IrqManager10HandleIrq\num\()Ev
_ZN2os5hwcom10IrqManager10HandleIrq\num\()Ev:
    movb $\num + IRQ_BASE, $irq
    pushl $0
    jmp int_bottom
.endm

/* for i in range(20): print(f"HandleException 0x{i:02X};")
    for i in range(16): print(f"HandleIrq 0x{i:02X};")
*/
HandleException 0x00;
HandleException 0x01;
HandleException 0x02;
HandleException 0x03;
HandleException 0x04;
HandleException 0x05;
HandleException 0x06;
HandleException 0x07;
HandleException 0x08;
HandleException 0x09;
HandleException 0x0A;
HandleException 0x0B;
HandleException 0x0C;
HandleException 0x0D;
HandleException 0x0E;
HandleException 0x0F;
HandleException 0x10;
HandleException 0x11;
HandleException 0x12;
HandleException 0x13;

HandleIrq 0x00;
HandleIrq 0x01;
HandleIrq 0x02;
HandleIrq 0x03;
HandleIrq 0x04;
HandleIrq 0x05;
HandleIrq 0x06;
HandleIrq 0x07;
HandleIrq 0x08;
HandleIrq 0x09;
HandleIrq 0x0A;
HandleIrq 0x0B;
HandleIrq 0x0C;
HandleIrq 0x0D;
HandleIrq 0x0E;
HandleIrq 0x0F;

HandleIrq 0x31;
HandleIrq 0x80;

/* general registers: 
        eax: accumulator registor/arthmetic
        ebx: base registor/data ptr
        ecx: counter register/loop counter
        edx: data register/data op/extensions
    index n ptrs:
        esi: source index/str mem op ptr
        edi: destination index/str dest ptr
        ebp: stack base pointer/stack frame ptr
        eip: instruction pointer/next instruction addr
        esp: stack pointer/stack top ptr
    seg registers:
        cs: code seg
        ds: data seg
        es: extra seg/video mem
        fs: extra seg/video mem
        gs: extra seg/video mem
        ss: stack seg/data
    eflags: flag register/status control sys flags 
        zf: zero flag
        cf: carry flag
        tf: trap flag
        if: irq flag 
    pusha: push all general purpose register -> stack
            save state/register
        cld: clear direction flag, go forward/safety
            increment register index
                load ring 0 -> seg register 0x10/data segselector in gdt
                    es ds critical
                        -> c++ code can safely access global var/stack
         irqHandler declaration
            push: HandleIrq(irq, esp)
        call mangled c++ method IrqManager
        add $8, %esp: rm 2 arg irq esp/8 bytes pushed
        mov , %esp: return new esp/after task sw/push/pop
            restore correct stack ptr 
*/
int_bottom:
    pusha
    pushl %ds
    pushl %es
    pushl %fs
    pushl %gs

    pushl %ebp
    pushl %edi
    pushl %esi

    pushl %edx
    pushl %ecx
    pushl %ebx
    pushl %eax

    cld
    #mov $0x10, %eax
    #mov %eax, %ds
    #mov %eax, %es

    pushl %esp
    push $irq
    call _ZN2os5hwcom10IrqManager10HandlerIrqEhj
    add $8, %esp
    mov %eax, %esp

    popl %eax
    popl %ebx
    popl %ecx
    popl %edx

    popl %esi
    popl %edi
    popl %ebp
    pop %gs
    pop %fs
    pop %es
    pop %ds
    #popa

.global _ZN2os5hwcom10IrqManager09IrqIgnoreEv 
_ZN2os5hwcom10IrqManager09IrqIgnoreEv:
    iret

