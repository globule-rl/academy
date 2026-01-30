.PHONY: all clean view

all: pwm.vvp
	vvp pwm.vvp
	gtkwave dump.vcd

pwm.vvp: pwm.v pwm_tb.v
	iverilog -o pwm.vvp pwm.v pwm_tb.v

clean:
	rm -f pwm.vvp 