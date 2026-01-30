# desk motor controller
## power
    motor wire 1.7mm 22awg 3a*30v=90w
        90w/24v=3.75a
    180w/0.8=225w 300w
    color wire 4mm pvc inside <-3mm
        thermal sensor wire 2.5mm <- 2mm
## pkg
    sudo apt-get install iverilog gtkwave
        brew install icarus-verilog gtkwave
    iverilog -o pwm.vvp pwm.v pwm_tb.v
    vvp pwm.vvp
    gtkwave dump.vcd
        cat dump.vcd | head -50
        less dump.vcd
## components
    inventory
        motor w/ hall 24v 7a
        esp32 5v/3.3v 500ma
            nano 5-12v 20mA/pin 500ma
        display 7-seg/digital
        usbc 5v->12v
        ac 220v -> 5v
        plug
    bts7960*3 ibt-2 motor drive 6-27v 43a
    acs712 current sensor 5v 30a
    220v -> 12v *16a 450w
        buck converter 12v->5v
    rj54 connector/wire
    fan connector
    fuse 15a
    wire
        3a 22awg 1.7mm to motor
            5a 20awg
            10a 16awg
        1.5a 26awg 1.5mm to esp
        600ma 30awg 0.8mm all colors to sensor
        length: 120-10-10=100-transformer/20=80cm
            3 rj45b plug+jack 0.5*2 1m
            



