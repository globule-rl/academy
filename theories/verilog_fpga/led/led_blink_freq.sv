/* 2 led 4 states, 4 freqs
    The count val to generate diff frequencies
        counter register 32 bit
        toggle register: flip at frequencies
    25 kHz -> 100Hz 125 counter led_select 2'b00
        toggle every 125 cycles at 100hz 5ms
            25000/100/2 = 125 counter 50% duty 
            125/25000 = 5ms  time/period
    25 kHz -> 1Hz  12500 counter led_select 2b'11
        toggle every 12,500 cycles at 1hz 0.5s 500ms
            12500/25000 = 500ms
    r_counter starts from 0 */
module led_blink_frequency (
    i_clock,
    i_enable,
    i_switch_1,
    i_switch_2,
    o_led_drive
);
    input i_clock;
    input i_enable;
    input i_switch_1;
    input i_switch_2;
    output o_led_drive;
    parameter c_counter_100Hz = 125;
    parameter c_counter_50Hz = 250;
    parameter c_counter_10Hz = 1250;
    parameter c_counter_1Hz = 12500;
    reg [31:0] r_counter_100Hz = 0;
    reg [31:0] r_counter_50Hz = 0;
    reg [31:0] r_counter_10Hz = 0;
    reg [31:0] r_counter_1Hz = 0;
    reg r_TOGGLE_100Hz = 1'b0;
    reg r_TOGGLE_50Hz = 1'b0;
    reg r_TOGGLE_10Hz = 1'b0;
    reg r_TOGGLE_1Hz = 1'b0;

    reg r_LED_SELECT;
    wire w_LED_SELECT;

    begin
        always @ (posedge i_clock) begin
                if (r_counter_100Hz == c_counter_100Hz-1) begin
                        r_TOGGLE_100Hz <= !r_TOGGLE_100Hz;
                        r_counter_100Hz <= 0;
                    end
                else
                    r_counter_100Hz <= r_counter_100Hz + 1;
            end

        always @ (posedge i_clock) begin
                if (r_counter_50Hz == c_counter_50Hz-1) begin
                        r_TOGGLE_50Hz <= !r_TOGGLE_50Hz;
                        r_counter_50Hz <= 0;
                    end
                else
                    r_counter_50Hz <= r_counter_50Hz + 1;
            end

        always @ (posedge i_clock) begin
            if (r_counter_10Hz == c_counter_10Hz-1) begin
                    r_TOGGLE_10Hz <= !r_TOGGLE_10Hz;
                    r_counter_10Hz <= 0;
                end
            else
                r_counter_10Hz <= r_counter_10Hz + 1;
        end

        always @ (posedge i_clock) begin
                if (r_counter_1Hz == c_counter_1Hz-1) begin
                        r_TOGGLE_1Hz <= !r_TOGGLE_1Hz;
                        r_counter_1Hz <= 0;
                    end
                else
                    r_counter_1Hz <= r_counter_1Hz + 1;
            end

        always @ (*) begin
                case ({i_switch_1, i_switch_2})
                    2'b11 : r_LED_SELECT <= r_TOGGLE_1Hz;
                    2'b10 : r_LED_SELECT <= r_TOGGLE_10Hz;
                    2'b01 : r_LED_SELECT <= r_TOGGLE_50Hz;
                    2'b00 : r_LED_SELECT <= r_TOGGLE_100Hz;
                endcase
            end 
        assign o_led_drive = r_LED_SELECT & i_enable;
    end 
endmodule
