/* tb: testbench 
    timescale: time unit 1s
        precision 1 ps for delay modeling
   uut: instantiate dut
   #10: 10ns half period 20ns/2 from 50mhz
        clock generation
    #100: release rst after 100ns delay
        deassert reset after initial hold
    #200_000_000: run 200ms sim time
            ns -> us -> ms */
`timescale 1ns/1ps
module tb_led_blink;
    reg clk = 0;
    reg rst = 1;
    wire led;
    led_blink uut (
        .clk(clk),
        .rst(rst),
        .led(led)
    );
    always #10 clk = ~clk;
    initial begin
        $dumpfile("led_blink.vcd");
        $dumpfile(0, tb_led_blink);
        #100 rst = 0;
        #200_000_000 $finish;
    end
endmodule

