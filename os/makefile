# sudo apt-get install g++ binutils libc6-dev-i386
# sudo apt-get install VirtualBox grub-legacy xorriso

# .s -> .o assembly multiboot header, kernel entry(stack constructor)
#	2Mib mebibyte 1Mib 1,048,576 bytes 1Mb 1,000,000 bytes
# .cpp -> .o obj/ data
# .ld -> link two .o
# -> .bin -> .iso -> clean up

GCCPARAMS = -m32 -Iinclude -fno-use-cxa-atexit -nostdlib - fno-builtin \ 
			-fno-exceptions -fno-leading-underscore -Wno-write-strings
ASPPARAMS = --32
LDPARAMS = -melf_i386

objects = obj/loader.o \
		  obj/gdt.o \
		  obj/memory_management.o \
		  obj/drivers/driver.o \
		  obj/hardware_communication/port.o \
		  obj/hardware_communication/interruptstubs.o \
		  obj/hardware_communication/interrupts.o \
		  obj/syscalls.o \
		  obj/multitasking.o \
		  obj/drivers/amd_am79c973.o \
		  obj/hardware_communication/pci.o \
		  obj/drivers/keyboard.o \
		  obj/drivers/vga.o \
		  obj/drivers/ata.o \
		  obj/gui/widget.o \
		  obj/gui/window.o \
		  obj/gui/desktop.o \
		  obj/net/etherframe.o \
		  obj/net/arp.o \
		  obj/net/icmp.o \
		  obj/net/ipv4.o \
		  obj/net/udp.o \
		  obj/net/tcp.o \
		  obj/kernel.o   
run: mykernel.iso
	(killall VirtualBox && sleep 1) || true
	VirtualBox --startvm "My Operating System" &

obj/%.o: src/%.cpp
	mkdir -p $(@D)
	gcc $(GCCPARAMS) -c -o $@ $<
obj/%.o: src/%.s
	mkdir -p $(@D)

mykernel.bin: linker.ld $(objects)
	ld $(LDPARAMS) -T $< -o $@ $(objects)

mykernel.iso: mykernel.bin
	mkdir iso
	mkdir iso/boot
	mkdir iso/boot/grub
	cp mykernel.bin iso/boot/mykernel.bin
	echo 'set timeout=0' 				   > iso/boot/grub/grub.cfg
	echo 'set default=0'				  >> iso/boot/grub/grub.cfg
	echo ''								  >> iso/boot/grub/grub.cfg
	echo 'menuentry "My Operating Sys" {' >> iso/boot/grub/grub.cfg
	echo '	multiboot /boot/mykernel.bin' >> iso/boot/grub/grub.cfg
	echo '	boot'						  >> iso/boot/grub/grub.cfg
	echo '}'							  >> iso/boot/grub/grub.cfg
	grub-mkrescue --output=mykernel.iso iso
	rm -rf iso
install: mykernel.iso
	sudo cp $< /boot/mykernel.bin

.PHONY: clean
clean:
	rm -rf obj mykernel.bin mykernel.iso