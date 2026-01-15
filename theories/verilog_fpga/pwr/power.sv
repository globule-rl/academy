/* dut: pwr: 5v
        diode: 1n4007
            polarity, blown shorted/conducts
        load: esp32/resistor 120mA/80-500mA, wifi 480mA, overcurrent/shorted 2000mA
        fuse: 500mA, 1us/1000ns delay to blow
    no pwr/protection 
            <- diode blown not shorted, reverse polarity
                fuse blown
    pwr <- correct polarity 0, diode blown shorted/conducts
            fuse not blown-conducts
*/
`timescale 1ns/1ps

module power (
    input  logic pwr,
    input  logic polarity,
    input  logic diode_blown,
    input  logic diode_shorted,
    input  logic short_circuit,
    input  logic wifi_mode,
    input  logic force_fuse_blown,

    output logic powered,
    output logic fuse_blown,
    output real  current_ma
);

    logic diode_conducts;
    logic fuse_conducts;
    real load_current;
    logic fuse_blown_internal = 0;
    real overcurrent_time = 0.0;

    parameter real FUSE_RATING_MA = 500.0;
    parameter real FUSE_BLOW_TIME_NS = 1000;

    always @(*) begin
        if (diode_blown) begin
            if (diode_shorted == 1'b1)
                diode_conducts = 1'b1;
            else
                diode_conducts = 1'b0;
        end else begin
            diode_conducts = (polarity == 1'b0) ? 1'b1 : 1'b0;
        end
    end

    always @(*) begin
        if (short_circuit)
            load_current = 2000.0;
        else if (wifi_mode == 1'b1)
            load_current = 480.0;
        else
            load_current = 120.0;
    end

    always @(pwr or diode_conducts or load_current or force_fuse_blown) begin
            if (force_fuse_blown) begin
                fuse_blown_internal = 1'b1;
                overcurrent_time = 0.0;
            end else if (pwr && diode_conducts && load_current > FUSE_RATING_MA) begin
                #1 overcurrent_time = overcurrent_time + 1.0;
                if (overcurrent_time >= FUSE_BLOW_TIME_NS) begin
                    fuse_blown_internal = 1'b1;
                    $display("  [%0t] Overcurrent: %.1f mA", $time, load_current);
                end
            end else begin
                overcurrent_time = 0.0;
            end
        end

    assign fuse_blown = fuse_blown_internal;
    assign fuse_conducts = ~fuse_blown_internal;
    assign powered = pwr & diode_conducts & fuse_conducts;
    always @(*) begin
        if (pwr && diode_blown && !diode_conducts)
            current_ma = 0.001;
        else if (pwr && !fuse_conducts)
            current_ma = 0.0;
        else if (powered && fuse_conducts)
            current_ma = load_current;
        else if (pwr && diode_blown && diode_shorted && fuse_conducts)
            current_ma = load_current;
        else
            current_ma = 0.0;
    end


endmodule