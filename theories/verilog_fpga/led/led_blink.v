/* wire: physical conn two ports, carry signal, gates
        val updated as driver changes
        continuous assignments/assign, combinatorial, gates
    reg: register, mem ele, procedural assignments, op later
        val updated every clock cycle
        always, initial, task, function blocks
            sequential/@posedge
    A B modules [3:0][3:0] 8 wires -> two wires A add  B sub
            input wire [3:0] a,
            input wire [3:0] b,
            output reg c
        A first inp -> conn to gnd, gnd, vcc, gnd
            two inps 0010 0011
            assign c = a & b
                assign: make conn outside of 'always'
            clocked
                always @ (posedge clk)
                    c <= a & b
        always: sensitivity
            clk synthesize as latches/retain state from inp until reset by signal
        rst: active high
    <= : no blocking assignment
        every line executed in parallel
    26 bit counter 50mhz/2^26 = 0.75Hz blink
        2^26 cycles/period 2^26/50m = 1.342s time/period
    counter == 26'd50_000_000: 1 sec delay at 50mhz */
module led_blink (
    input wire clk,
    input wire rst,
    output reg led
);
    reg [25:0] counter;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
                counter <= 0;
                led <= 0;
        end else begin
            counter <= couter+1;
            if (counter == 26'd50_000_000) begin
                counter <= 0;
                led <= ~led;
            end
        end
    end
endmodule
