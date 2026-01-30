module pwm(
    input clk,
    input [7:0] duty,
    output reg pwm_out
);
    reg [7:0] counter;
    
    always @(posedge clk) begin
        counter <= counter + 1;
        pwm_out <= (counter < duty) ? 1 : 0;
    end
endmodule
