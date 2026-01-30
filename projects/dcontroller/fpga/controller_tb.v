`timescale 1ns/1ps

module tb;
    reg clk;
    reg rst_n;

    wire motor_dir;
    wire motor_ena;
    wire pwm_out;
    entity dut (
        .clk(clk),
        .rst_n(rst_n),

        .motor_dir(motor_dir),
        .motor_ena(motor_ena),
        .pwm_out(pwm_out)
    );
    initial begin
        clk = 0;
        forever #41.667 clk = ~clk; // 24->12mhz, half period=1/24e6/2
    end
    initial begin
        $dumpfile("dump.vcd");
        $dumpvars(0, testbench); // depth/recursive 0
    end
    initial begin
        rst_n = 0;

        $display("=== Testbench ===");
        #200 rst_n = 1; // after 200 ns, rst
        $display("%0t: sys rst", $time);

        #1000;
        $display("motor ena %b", motor_ena); // in binary
        $display("");
    end
endmodule