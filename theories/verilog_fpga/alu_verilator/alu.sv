/* verilog: verification and logic
	operation_t: verilator public -> tx transaction
			make it pubic in Valu___024unit.h
		2'h1: set 2-bit address signal to 1 01
		2'h2: 2 10
	input [5:0]: 6-bit input port
	logic [5:0]: 6-bit logic vector 3 state
		[3:0][7:0]: packed arr, 4 words of 8 bits each, packed into 32-bit vector
			4-stae vals(0, 1, X, Z), wire/reg 
				enable modeling for buses/signals
	always_ff: flip flop, oup on clk edge
	out_valid <= 1'b1: always be 1, should be in_valid_r */
typedef enum logic [1:0] {
	add = 2'h1,
	sub = 2'h2,
	nop = 2'h0
} operation_t 

module alu #(
	parameter WIDTH = 6
)(
	input clk,
	input rst,
	input operation_t op_in,
	input [WIDTH-1:0] a_in,
	input [WIDTH-1:0] b_in,
	input 		  in_valid,
	output logic [WIDTH-1:0] out,
	output logic 		 out_valid
);
	operation_t op_in_r;
	logic [WIDTH-1:0] a_in_r;
	logic [WIDTH-1:0] b_in_r;
	logic 		  in_valid_r;
	logic [WIDTH-1:0] result;

	always_ff @ (posedge clk, posedge rst) begin
		if (rst) begin
			op_in_r <= nop;
			a_in_r <= '0;
			b_in_r <= '0;
			in_valid_r <= '0;
		end else begin
			op_in_r <= op_in;
			a_in_r <= a_in;
			b_in_r <= b_in;
			in_valid_r <= in_valid;
		end
	end
	
	always_comb begin
		result = '0;
		if (in_valid_r) begin
			case (op_in_r)
				add: result = a_in_r + b_in_r;
			    sub: result = a_in_r + (~b_in_r+1'b1);
				default: result = '0;
			endcase
		end
	end

	always_ff @ (posedge clk, posedge rst) begin
		if (rst) begin
			out 	<= '0;
			out_valid <= '0;
		end else begin
			out	 <= result;
			out_valid <= in_valid_r;
		end
	end
endmodule;





			
