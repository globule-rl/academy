/* flags: bitmask for avail fields
    0 mem, 1 bootdev, 2 cmdline, 3 mods,
    4 aout_syms, 5 elf_secs, 6 mmap, 7 drives,
    8 config, 9 loader_name, 10 apm

    uint32_t
    mem_lower/upper: than 1MB
    bootdev: boot drive
    cmdline: cmdline ptr
    mods_count/addr: module count, struct arr ptr
    syms/elf: var excutable/linkable format, symbol/section tables
    mmap: multiboot_memory_map_t arr(size, base_addr, len, type)
    apm_table: advanced pwr management suspend/standby
        BIOS info(major/minor version, flags, entry point) table ptr 

    grub: grand unified bootloader, load kernels from filesyms(ext2 extended/FAT file allocation table)
*/
.set MAGIC, 0x1badb002
.set FLAGS, (1<<0 | 1<<1)
.set CHECKSUM, -(MAGIC + FLAGS)

.section .multiboot
    .long MAGIC
    .long FLAGS
    .long CHECKSUM

.section .text
.extern kernalMain
.extern callCtors
.global loader

/* multiboot header
    esp: stack ptr
    eax: accumulator/num register, parser magic no.
    ebx: base/data register, struct, parse multiboot_info_t via reg
*/
loader:
    mov $kernel_stack, $esp
    call callCtors
    push %eax
    push %ebx
    call kernalMain

_stop:
    cli
    hlt
    jump _stop

.section .bss
.space 2*10241024;
kernel_stack: