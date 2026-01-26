# motor controller
## features
    - up/down
    - memory
## pkg
    sudo apt-get install iverilog gtkwave
        brew install icarus-verilog gtkwave
    iverilog -o pwm.vvp pwm.v pwm_tb.v
    vvp pwm.vvp
    gtkwave dump.vcd
        cat dump.vcd | head -50
        less dump.vcd

