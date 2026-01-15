# intel 32-bit i386 x86
    registers:
        - main: eax/accumulator, ecx/count, edx/data, ebx/base
        - index: esp/stack_ptr, ebp/base_ptr, esi/src_index, edi/dest_index
        - program counter: eip/instruction_ptr
        - seg selectors: cs/code_seg, ds/data_seg, es/extra_seg, fs/f_seg, gs/g_seg, ss/stack_seg
        - status: EFlags
# local test/qemu
    make run

## pkg
    - sudo apt-get install g++ binutils libc6-dev-i386
    - sudo apt-get install VirtualBox grub-legacy xorriso
    - sudo apt-get install qemu-system-i386
    - sudo apt-get install qemu-arch-extra

## makefile
    - .s -> .o assembly multiboot header, kernel entry/stack constructor
    - .cpp -> .o obj/ data
    - .ld -> link two .o
    -   -> .bin -> .iso -> clean up