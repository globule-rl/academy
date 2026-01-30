module entity (
    input clk, // 12mhz clk sig
    input rst_n, // default high, active/pressed/! low -> trigger rst

    input hall_sensor,
    input encoder_a,
    input encoder_b,
    input cur_sensor,
    input temp_sensor,
    input pressure_sensor,

    input uart_rx,
    input lm_sw_top,
    input lm_sw_bot,
    input emerg_stop,

    output motor_ena,
    output motor_dir,
    output pwm_out,

    output uart_tx,
    output [7:0] status,
    output alarm,
    output [7:0] debug_bus,

    inout sda, // serial data
    inout scl // serial clk
);

    wire clk_sys, clk_pwm, clk_uart;
    wire pll_locked; // phase locked loop/synchronized out sig phase/freq w/ inp

    wire [15:0] cur_pos;
    wire [7:0] motor_speed_target; // motor control fsm finite state machine
    wire motor_running; // limit, pos, dir
    wire [7:0] pwm_duty; // pwm generator
    wire [15:0] pos_feedback; // encoder
    assign cur_pos = pos_feedback;
    wire [15:0] motor_rpm; // hall
    wire [9:0] temp_adc, cur_adc, pressure_adc;
    wire temp_warning, temp_critical;
    wire cur_limit_active;
    wire collision_detected; // temp, pressure

    wire [7:0] uart_rx_data;
    wire uart_rx_valid;
    wire [7:0] uart_tx_data;
    wire uart_tx_ready;
    wire [7:0] cmd_type;
    wire [7:0] cmd_speed;
    wire cmd_dir;
    wire cmd_valid; // uart, debug
    wire watchdog_timeout; // heartbeat, led ctrl

    wire rtc_valid; // i2c inter-integrated circuit protocol, real-time clock
    wire [7:0] rtc_hrs, rtc_mins; // sda scl

    wire safe_to_operate;
    assign safe_to_operate = ~emerg_stop & ~temp_critical & 
                            ~watchdog_timeout & ~collision_detected;
    assign motor_ena = (safe_to_operate & motor_running) ? 1'b1 : 1'b0;

    clk_manager clk_mgr (
        .clk_in(clk), // conn inp to sig clk
        .rst_n(rst_n),
        .clk_sys(clk_sys),
        .clk_pwm(clk_pwm), // 48mhz pwm
        .clk_uart(clk_uart), // 1.843mhz 115200 baud uart
        .pll_locked(pll_locked)
    );
endmodule

module clk_manager (
    input clk_in,
    input rst_n,
    output clk_sys,
    output clk_pwm,
    output clk_uart,
    output pll_locked
);
    reg [3:0] div_sys, div_pwm; // clk dividers
    reg [5:0] div_uart;

    assign clk_sys = clk_in;
    assign clk_pwm = clk_in;
    assign clk_uart = clk_in;
    assign pll_locked = 1'b1;
endmodule

module motor_control_fsm (
    input clk, rst_n,
    input [15:0] cur_pos,
    input limit_top, limit_bot,
    input cmd_valid,
    input [7:0] cmd_speed,
    input cmd_dir,
    
    output motor_dir,
    output motor_running,
    output alarm,
    output [2:0] debug
);
    reg [7:0] speed_reg;
    reg dir_reg, running;
    reg [2:0] state;

    parameter IDLE = 0, RAMP_UP = 1, RUNNING = 2, RAMP_DOWN = 3, STOPPED = 4;

    always @(posedge clk or negedge rst_n) begin // posedge mem/reg, state sequential respond to clk edge
       if (!rst_n) begin
            speed_reg <= 0; // non-blocking, execute parallel, right/read val => left/write
            dir_reg <= 0;
            running <= 0;
            state <= IDLE;
       end else begin
            case(state)
                IDLE: begin
                    if (cmd_valid && cmd_speed > 0) begin
                        dir_reg <= cmd_dir;
                        state <= RAMP_UP;
                    end
                end  
                RAMP_UP: begin
                    if (speed_reg<cmd_speed) speed_reg<=speed_reg+1;
                    else state <= RUNNING;
                    running <= 1;
                end
                RUNNING: begin
                    speed_reg <= cmd_speed;
                    if ((cmd_dir&&limit_bot) || (!cmd_dir&&limit_top)) state <= RAMP_DOWN;
                    if (cmd_speed == 0) state <= RAMP_DOWN;
                end
                RAMP_DOWN: begin
                    if (speed_reg > 0) speed_reg <= speed_reg-1;
                    else state <= STOPPED;
                end
                STOPPED: begin
                    running <= 0;
                    state <= IDLE;
                end
            endcase
        end
    end
    assign motor_speed = speed_reg;
    assign motor_dir = dir_reg;
    assign motor_running = running;
    assign alarm = (limit_top || limit_bot) ? 1'b1 : 1'b0;
    assign debug = state;
endmodule

module pwm_scurve_controller (
    input clk, rst_n,
    input [7:0] target_speed,
    input motor_running,
    
    output [7:0] pwm_duty
);
    localparam accel_rate = 8'd2;
    reg [7:0] cur_duty;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) cur_duty <= 0;
        else if (motor_running) begin
            if (cur_duty<target_speed) cur_duty <= cur_duty+accel_rate;
            else if (cur_duty>target_speed) cur_duty <= cur_duty-accel_rate;
        end else cur_duty <= 0;
    end
    assign pwm_duty = cur_duty;
endmodule

module pwm_generator #(
    parameter CLK_MHZ = 48, // clk 12* pll 4
    parameter FREQ_HZ = 2000
    )(
    input clk,
    input [7:0] duty,
    
    output pwm_out
);
    localparam PERIOD = CLK_MHZ*1_000_000/FREQ_HZ;
    reg [19:0] counter;
    always @(posedge clk) counter <= (counter>=PERIOD-1) ? 0 : counter+1; // 0-(PERIOD-1) cycle
    assign pwm_out = (counter<(duty*PERIOD/256)); // high when cnt<duty, 0-255 pulse width, higher duty, longer pulse, more pwr
endmodule

module quadrature_decoder (
    input clk, rst_n,
    input enc_a, enc_b,

    output [15:0] pos
);
    reg[15:0] pos_counter;
    reg [1:0] enc_prev;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pos_counter <= 0;
            enc_prev <= 0;
        end else begin
            enc_prev <= {enc_a, enc_b};
            if (enc_prev==2'b00 && {enc_a, enc_b}==2'b01) pos_counter<=pos_counter+1;
            if (enc_prev==2'b01 && {enc_a, enc_b}==2'b10) pos_counter<=pos_counter+1;
            if (enc_prev==2'b10 && {enc_a, enc_b}==2'b11) pos_counter<=pos_counter+1;
            if (enc_prev==2'b11 && {enc_a, enc_b}==2'b00) pos_counter<=pos_counter+1;
        end
    end
    assign pos = pos_counter;
endmodule

module speed_monitor (
    input clk, rst_n,
    input hall_inp,
    output reg [15:0] rpm
);
    reg [15:0] pulse_cnt;
    reg [19:0] timer;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pulse_cnt <= 0;
            timer <= 0;
        end else begin
            timer <= timer+1;
            if (timer == 12_000_000) begin // 12mhz/cycle 1 sec
                rpm <= pulse_cnt*60;
                pulse_cnt <= 0;
                timer <= 0;
            end
        end
    end
endmodule

module simple_adc #(
    parameter WIDTH=10, 
    CHANNELS=3
    )(
    input clk, rst_n,
    input ch0_in, ch1_in, ch2_in,
    output [WIDTH-1:0] ch0_out, ch1_out, ch2_out
);
    reg [WIDTH-1:0] ch0_reg, ch1_reg, ch2_reg;
    reg [1:0] ch_sel; // channel selector
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) ch_sel <= 0;
        else begin 
            ch_sel <= ch_sel+1;
            case(ch_sel)
                0: ch0_reg <= {ch0_reg[WIDTH-2:0], ch0_in};
                1: ch1_reg <= {ch1_reg[WIDTH-2:0], ch1_in};
                2: ch2_reg <= {ch2_reg[WIDTH-2:0], ch2_in};
            endcase
        end
    end
    assign ch0_out = ch0_reg;
    assign ch1_out = ch1_reg;
    assign ch2_out = ch2_reg;
endmodule

module load_detector (
    input clk,
    input [9:0] pressure_adc,
    input [9:0] cur_adc,
    output collision
);
    assign collision = (pressure_adc>10'd800) || (cur_adc>10'd900);
endmodule

module watchdog_timeout #(
    parameter TIMEOUT_MS=500
    )(
    input clk, rst_n,
    input heartbeat,
    output timeout
);
    reg[19:0] counter;
    localparam TIMEOUT_CYCLES = TIMEOUT_MS*12000/1000; // 12m cycles per s, 12000 per ms
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) counter <= 0;
        else if (heartbeat) counter <= 0;
        else counter <= counter+1;
    end
    assign timeout = (counter >= TIMEOUT_CYCLES); // timeout flag high when counter reaches TIMEOUT_CYCLES
endmodule