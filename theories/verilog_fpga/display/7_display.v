/* 4 7-seg led_out[6:0]:
    each: common anode high, 7 cathodes[6:0] low to illuminate '8'
    one_sec_counter[26:0]: 2^27 - 1 = 134m > 100m
        generate 1s enable pulse
    human eye refresh rate: every 1ms-16ms, 4ms each 7-seg
        1000ms/1ms = 1 khz, num of ms update img in 1s
        1000ms/16ms = 60hz
        10.5ms/4 = 2.6ms each, not the same time, cycle every 10.5ms
            refresh_counter[19:0]: can count to 1,048,575, counter increment by 1 every clock cycle/edge
                    100mhz 100m cycles in 1s
                    1/100mhz = 10 ns per tick, counter increase once every 20ns, whole counter [19] 21 ms
                    100,000,000/(2^19-1) = 190hz 
                led_activating_counter[1:0]: 
                    first msb lower 2-bit for 4 signals [18] 10.4ms
                        cycling thru 00/01/10/11, 2.6ms digit period each at 100mhz every 2^18 clock cycles
                    fixed, continuously activating updating                                     
    decoder: cathode[6:0] low for illuminated, high for un-illuminated
        '0': A-F low G high 7'b000000
            displayed_num [15:0] 
                    4 digits, 4 bit per digit
                    displayed_num/1000 (displayed_num % 1000)/100 displayed_num % 10
                    thousands hundreds tens units
                led_bcd[3:0]: binary coded decimal
                    0-9 10 nums in 2^4 = 16 bits */
module Seven_seg_LED_Display_Controller (
    input clock_100Mhz,
    input reset,
    output reg [3:0] Anode_Activate,
    output reg [6:0] LED_out 
    );

    reg [26:0] one_sec_counter;
    wire one_sec_enable;
    reg [19:0] refresh_counter;
    wire [1:0] led_activating_counter;
    reg [15:0] displayed_num;
    reg [3:0] led_bcd;

    always @(posedge clock_100Mhz or posedge reset)
    begin
        if (reset == 1)
            one_sec_counter <= 0;
        else begin
            one_sec_counter <= one_sec_counter + 1;
        end
    end
    assign one_sec_enable = (one_sec_counter==99999999)?1:0;

    always @(posedge clock_100Mhz or posedge reset)
    begin
        if (reset == 1)
            refresh_counter <= 0;
        else begin
            refresh_counter <= refresh_counter + 1;
        end
    end
    assign led_activating_counter = refresh_counter[19:18];

    always @(posedge clock_100Mhz or posedge reset)
    begin
        if (reset == 1)
            displayed_num <= 0;
        else if (one_sec_enable == 1)
            displayed_num <= displayed_num + 1;
    end

    always @(*)
    begin
        case (led_activating_counter)
        2'b00: begin
            Anode_Activate = 4'b0111;
            led_bcd = displayed_num[15:12];
        end
        2'b01: begin
            Anode_Activate = 4'b1011;
            led_bcd = displayed_num[11:8];
        end
        2'b10: begin
            Anode_Activate = 4'b1101;
            led_bcd = displayed_num[7:4];
        end
        2'b11: begin
            Anode_Activate = 4'b1110;
            led_bcd = displayed_num[3:0];
        end
        endcase
    end
    always @(*)
    begin
        case(led_bcd)
        4'b0000: LED_out = 7'b0000001;
        4'b0001: LED_out = 7'b1001111;
        4'b0010: LED_out = 7'b0010010;
        4'b0011: LED_out = 7'b0000110;
        4'b0100: LED_out = 7'b1001100;
        4'b0101: LED_out = 7'b0100100;
        4'b0110: LED_out = 7'b0100000;
        4'b0111: LED_out = 7'b0001111;
        4'b1000: LED_out = 7'b0000000;
        4'b1001: LED_out = 7'b0000100;
        default: LED_out = 7'b0000001;
        endcase
    end
endmodule