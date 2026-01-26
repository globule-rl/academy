`timescale 1us/1ns

module pwm_tb;
    reg clk;
    reg [7:0] duty;  // 64x4=256
    wire pwm_out;
    
    pwm dut(.clk(clk), .duty(duty), .pwm_out(pwm_out));
    
    initial begin
        $dumpfile("dump.vcd");
        $dumpvars(0, pwm_tb);
        
        clk = 0;
        duty = 64;  // 25% duty cycle, 64/256
        
        repeat(2000) #0.5 clk = ~clk;
        
        #500 duty = 128;  // 50% 128/256
        repeat(2000) #0.5 clk = ~clk;
        
        #500 duty = 192;  // 75% 192/256
        repeat(2000) #0.5 clk = ~clk;
        
        $finish;
    end
endmodule