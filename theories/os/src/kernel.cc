#include <common/types.h>
#include <memManagement.h>
#include <gdt.h>
#include <multiTasking.h>
#include <hwCom/interrupts.h>
#include <sysCalls.h>

#include <drivers/driver.h>
#include <drivers/keyboard.h>
#include <drivers/mouse.h>
#include <drivers/vga.h>
#include <drivers/sata.h>
#include <gui/desktop.h>
#include <gui/window.h>

#include <drivers/etherAmd.h>
#include <net/arp.h>
#include <net/etherFrame.h>
#include <net/icmp.h>
#include <net/ipv4.h>
#include <net/tcp.h>
#include <net/udp.h>

// #define GRAPHICSMODE

using namespace os;
using namespace os::common;
using namespace os::hwCom;
using namespace os::drivers;
using namespace os::gui;
using namespace os::net;

/* 0xb8000: x86 protected text mode base addr
            80colx25chars, each 2 bytes(char+attr)
        &0xFF00: mask preserve high/attr byte
        | str[i]: update (x, y) to str[i]
        | ' ': set each char at offset/80*y+x to space/clear line 
    python create '0123456789ABCDEF'
            0: ascii 48
            A: ascii 65
        "".join(chr(48+i) if i<10 else chr(65+i-10) for i in range(16))
    hex: 4 bits
        hex[i]: lookup table i index, hex[15] -> 'F'
*/
void printf(char* str) {
    static uint16_t* baseAddr = (uint16_t*)0xb8000;
    static uint8_t x=0, y=0;
    for (int i=0; str[i]!='\0'; ++i) {
        switch(str[i]) {
            case '\n': x =0; y++; break;
            default: baseAddr[80*y+x] = (baseAddr[80*y+x] & 0xFF00) | str[i]; x++; break;
        }
        if (x >= 80) {x=0; y++;}
        if (y >= 25) {
            for (y=0; y<25; y++)
                for (x=0; x<80; x++) baseAddr[80*y+x] = (baseAddr[80*y+x] & 0xFF00) | ' '; x=0; y=0;
            }
    }
}
void printfHex(uint8_t i) {
    char* foo = "00";
    char* hex = "0123456789ABCDEF";
    foo[0] = hex[(i>>4) & 0xF];
    foo[1] = hex[i & 0xF];
    printf(foo);
}
void sysprintf(char* str) { asm("int $0x80" : : "a" (4), "b" (str));}

typedef void (*ctor)();
extern "C" ctor start_ctors;
extern "C" ctor end_ctors;
extern "C" void callCtors() {
    for (ctor* i=&start_ctors; i!=&end_ctors; i++) (*i)();
}

void taskA() { while(true) sysprintf("taskA");}
void taskB() { while(true) sysprintf("taskB");}


/* kernel mem:
        BIOS/multiboot: firmware bootloader
        kernel code/,text: executable instructions 
        kernel data/.data: initialize globals
        kernel bss/.bss: uninitialized globals/zeroed
        kernel stack: per-cpu stacks
        vmalloc: hight mem/dynamic alloc
        modules: var/loadable mods
        user space: processes/virtual
    start at 1MB post-boot 
        paging -> high mem mapping
    multiboot: firmware header
        esp eax/magic no. ebx/data info struct
    10*1024*1024: 10 mega bytes 0xA00000 25bit 
        1024: 0x400 2^10
        1 kb = 1024 bytes 10*1024 10kb/0x2800 reserve/guard 
        1 mb = 1024*1024 bytes
    (*i)(): deref call each as func to run global/static init 
    +8: add 8 bytes/offset to mem_upper kb
        ->memupper/total usable ram/0x90000
                    -heap/gc -os struct
                -> stack/auto mem
    mem: start from heap 0x100000
        size 10kb reserve: prevent overflow 
            dynamic allc malloc/free within bounds
    >> 24 & 0xFF: get hex val
        shift 24 bit & preserve the highest two bits
    0x20: hwIrqOffset
    0x80: irq+hwIrqOffset

*/
extern "C" void kernelMain(const void* multiboot_structure, uint32_t /*multiboot_magic*/) {
    printf("hello world");

    uint32_t* memupper = (uint32_t*)(((size_t_32)multiboot_structure) + 8);
    size_t_32 heap = 10*1024*1024;
    MemManager mem(heap, (*memupper)*1024 - heap - 10*1024);
    printf("heap: 0x");
    printfHex((heap >> 24) & 0xFF);
    printfHex((heap >> 16) & 0xFF);
    printfHex((heap >> 8) & 0xFF);
    printfHex((heap     ) & 0xFF);

    void* allocd = mem.malloc(1024);
    printf("\nallocd: 0x");
    printfHex(((size_t_32)allocd >> 24) & 0xFF);

    Gdt gdt;
    TaskManager taskManager;
    Task task1(&gdt, taskA);
    Task task2(&gdt, taskB);

    IrqManager irqs(&taskManager, 0x20, &gdt);
    SysCallHandler syscalls(0x80, &irqs);
    #ifdef GRAPHICSMODE
        Desktop desktop(320, 200, 0x00, 0x00, 0xA8);
    #endif
    DrvManager drvManager;
        #ifdef GRAPHICSMODE
            KeyboardDriver keyboard(&desktop, &irqs);
        #else
            KeyboardEvHandler kEvHandler;
            KeyboardDriver keyboard(&kEvHandler, &irqs);
        #endif
        drvManager.AddDriver(&keyboard);
        #ifdef GRAPHICSMODE
            MouseDriver mouse(&desktop, &irqs);
        #else
            MouseEvHandler mEvHandler;
            MouseDriver mouse(&mEvHandler, &irqs);
        #endif
        drvManager.AddDriver(&mouse);
        #ifdef GRAPHICSMODE
            Vga vga;
        #endif
        drvManager.ActivateAll();
    #ifdef GRAPHICSMODE
        vga.SetMode(320, 200, 0);
        Window win1(&desktop, 10,10,20,20, 0xA8,0x00,0x00);
        desktop.Attach(&win1);
    #endif

    Sata sata0m(true, 0x1F0);
    sata0m.Identify();
    Sata sata0s(false, 0x1F0);
    sata0s.Identify();
    sata0s.Write28(0, (uint8_t)"https://draw.io", 25);
    sata0s.Flush();
    sata0s.Read28(0, 25);
    Sata sata1m(true, 0x170);
    sata1m.Identify();
    Sata sata1s(false, 0x170);
    sata1s.Identify();

    Amd* eth0 = (Amd*)(drvManager.drivers[2]);
    uint8_t ip1=10, ip2=0, ip3=2, ip4=15;
    uint32_t ip_be = (uint32_t)ip1 | (uint32_t)ip2<<8 | (uint32_t)ip3<<16 | (uint32_t)ip4<<24;
    eth0->SetIPAddr(ip_be);
    Efp etherFrame(eth0);
    Arp arp(&etherFrame);

    uint8_t gip1=10, gip2=0, gip3=2, gip4=2;
    uint32_t gip_be = (uint32_t)gip1 | (uint32_t)gip2<<8 | (uint32_t)gip3<<16 | (uint32_t)gip4<<24; 
    uint8_t subnet1=255, subnet2=255, subnet3=255, subnet4=0;
    uint32_t subnet_be = (uint32_t)subnet1 | (uint32_t)subnet2<<8 | (uint32_t)subnet3<<16 | (uint32_t)subnet4<<24; 
    Ipv4 ipv4(&etherFrame, &arp, gip_be, subnet_be);
    Icmp icmp(&ipv4);
    Tcp tcp(&ipv4);
    Udp udp(&ipv4);

    irqs.Activate();
    arp.BroadcastMacAddr(gip_be);
    TcpHandler tcpHandler;
    TcpSocket* tcpSocket = tcp.Listen(1234);
    tcp.Bind(tcpSocket, &tcpHandler);
    tcpSocket->Send((uint8_t*)"hi tcp", 10);
    icmp.ReqEchoReplay(gip_be);

    while (1) {
        #ifdef GRAPHICSMODE
            desktop.Draw(&vga);
        #endif
    }
}