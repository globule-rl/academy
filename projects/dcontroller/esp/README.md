# motor controller
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
## software
### platform io 
    static ck
    unit test
    lib freertos task 
    lib display lvgl touchscreen
        buf drv color btn stat event/ev         
### motor pwm/speed state hall current
    motor
        state id
            up down/-/low stop/0
    rtos tasks cmdq mutex/share
        xSemaphoreTake/block other, safe read, Give//release mutex
    init pin low_state
    motor/accel control rpm+10
        clamp min max
    hall - prev_cnt, irq
        3 wires/6 states, 000 111 invalid/fault protection
        noise/debounce
        sync motor
        speed rpm/angle/pos
        over current limit stop/safety
    cmd motor 0/1, reverse, testing
    telemetry/logging
        idle task
### interface
    btn digitalRead
        debounce
        prefs/eeprom store small vals, float, kv
        upd display clear size cursor height mm display
    web server 80
        wifi attempts 20
        html root style btn .motor .status cls
            /control post json.stringigy, sendcmd/speed,id 
                json doc -> cmd queue
            /status getEleById(0/1) upd status
                semaphore take, doc read, give
                    serialize doc to json
            send status 200/400/405
### unit test
    delta time
    mock
    hil
### opencode
    export PATH=/Users/stella/.opencode/bin:$PATH

## pin conn
    5-pin/purple-green-black-brown-yellow
        signal/3
        pwr black-brown
        #TODO multimeter verify
    2-pin/end-of-travel/thermal/safety sw/red-black -> 5+2pin board
    2-pin/red-white -> hall sensor/pulse?/overcurrent?

## debug
### llm
    opencode ./usage
    local lm studio
        lms server start --port 1234
        export ANTHROPIC_BASE_URL=http://localhost:1234
        export ANTHROPIC_AUTH_TOKEN=lmstudio
            claude --model openai/gpt-oss-20b
        /model
    openrouter
        "$schema": "https://opencode.ai/config.json",
            "model": "openrouter/anthropic/claude-sonnet-4.5",
        "plan": "model": "openrouter/anthropic/claude-opus-4.5",
    openclaw price drop
### src
   .platformio/packages/framework-espidf/components/esp_adc/include/esp_adc/adc_oneshot.h

### header files
    mv test/test_motor_wrapper.h ../../../../ref/dmotor/test
    cache
    /Users/stella/.platformio/.cache/tmp/test-list-21269.json

### hil 
    mock test 
        ./run_tests.sh mock 
        cmock
            mv test/test_motor.c ../../../../ref/dmotor/test
            mv run_tests.sh ../../../../ref/dmotor/test


