module tb_pwr;
    logic pwr;
    logic polarity;
    logic diode_blown;
    logic diode_shorted;
    logic short_circuit;
    logic wifi_mode;
    logic force_fuse_blown;

    logic powered;
    logic fuse_blown;
    real  current_ma;
    
    int test_num = 0;
    int passed = 0;
    int failed = 0;
    
    power dut (
        .pwr(pwr),
        .polarity(polarity),
        .diode_blown(diode_blown),
        .diode_shorted(diode_shorted),
        .short_circuit(short_circuit),
        .wifi_mode(wifi_mode),
        .force_fuse_blown(force_fuse_blown),
        .powered(powered),
        .fuse_blown(fuse_blown),
        .current_ma(current_ma)
    );
    
    task run_test(
        input string test_name,
        input logic pwr, wifi_mode, polarity, diode_blown, diode_shorted, short_circuit, force_fuse_blown
        input logic expected_fuse_blown, input logic expected_powered, input real expected_current_min, input real expected_current_max
    );
        test_num++;
        pwr = pwr;
        wifi_mode = wifi_mode; 
        polarity = polarity;
        diode_blown = diode_blown;
        diode_shorted = diode_shorted;
        short_circuit = short_circuit;
        force_fuse_blown = force_fuse_blown;
        
        #10;
        
        if (powered === expected_powered && fuse_blown === expected_fuse_blown && current_ma >= expected_current_min && current_ma <= expected_current_max) begin
            $display("✓ Test %0d PASSED: %s", test_num, test_name);
            $display("  Device: %s, Fuse: %s, Current: %.2f mA", powered ? "POWERED" : "OFF", fuse_blown ? "BLOWN" : "OK", current_ma);
            passed++;
        end else begin
            $display("✗ Test %0d FAILED: %s", test_num, test_name);
            $display("  Expected: Device=%b, Fuse=%b, Current=%.1f-%.1f mA", expected_powered, expected_fuse_blown, expected_current_min, expected_current_max);
            $display("  Got: Device=%b, Fuse=%b, Current=%.2f mA", powered, fuse_blown, current_ma);
            failed++;
        end
        $display("");
    endtask
    
    task run_overcurrent_test(
        input string test_name,
        input logic pwr, polarity, diode_blown, diode_shorted, short_circuit
        input int wait_time_ns
    );
        test_num++;
        pwr = pwr;
        polarity = polarity;
        diode_blown = diode_blown;
        diode_shorted = diode_shorted;
        short_circuit = short_circuit;
        force_fuse_blown = 0;
        
        $display(" Overcurrent Test %0d: %s", test_num, test_name);
        $display("  Waiting %0d ns for fuse response...", wait_time_ns);
        
        #(wait_time_ns);
        
        if (powered === 1'b0 && fuse_blown === 1'b1) begin
            $display("✓ Test %0d PASSED: %s", test_num, test_name);
            $display("  Fuse correctly blew, Device protected");
            passed++;
        end else begin
            $display("✗ Test %0d FAILED: %s", test_num, test_name);
            $display("  Expected: Fuse blown and Device off");
            $display("  Got: Fuse=%b, Device=%b", fuse_blown, powered);
            failed++;
        end
        $display("");
    endtask
    
    initial begin       
        pwr = 0;
        polarity = 0;
        diode_blown = 0;
        diode_shorted = 0;
        wifi_mode = 0;
        short_circuit = 0;
        force_fuse_blown = 0;

        #5;
        // pwr, wifi_mode, polarity, diode_blown, diode_shorted, short_circuit, force_fuse_blown 
        // expected_fuse_blown, expected_power, expected_current_min, expected_current_max
        $display("--- NORMAL OPERATION TESTS ---\n");
        run_test("Normal operation - idle mode", 
                 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 100.0, 150.0);       
        run_test("Normal operation - WiFi transmission/max fuse limit", 
                 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 450.0, 500.0);
        
        $display("--- PROTECTION TESTS/DIODE/FUSE ---\n");        
        run_test("Reverse polarity -> diode protects -> no power", 
                 1'b1, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 0.0, 0.01);                         
        run_test("Blown/Failed diode/Open -> no power", 
                 1'b1, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 0.0, 0.01);        
        run_test("Blown/Failed diode/shorted -> no protection -> has power", 
                 1'b1, 1'b1, 1'b0, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b1, 100.0, 150.0);        
        run_test("Reverse polarity + Shorted diode -> no protection -> has power", 
                 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b1, 100.0, 150.0);      
        run_test("Pre-blown fuse -> no power", 
                 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 0.0, 0.01);
        
        // pwr, polarity, diode_blown, diode_shorted, short_circuit, wait_time_ns
        $display("--- OVERCURRENT TESTS ---\n");        
        run_overcurrent_test("Short circuit -> fuse blow -> no pwr",
                            1'b1, 1'b0, 1'b0, 1'b0, 1'b1, 1500);        
        pwr = 0;
        short_circuit = 0;
        #100;
        run_overcurrent_test("Shorted diode + short circuit -> fuse blow/protects -> no pwr",
                            1'b1, 1'b0, 1'b1, 1'b1, 1'b1, 1500);
        
        pwr = 0;
        short_circuit = 0;
        diode_blown = 0;
        #100;        
        run_test("Power restored after repairs", 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 100.0, 150.0);
        
        $display("\n========================================");
        $display("Test Summary:");
        $display("  Total:  %0d", test_num);
        $display("  Passed: %0d", passed);
        $display("  Failed: %0d", failed);
        $display("========================================\n");
        
        if (failed == 0)
            $display("✓ All tests passed! Circuit power protection verified.\n");
        else
            $display("✗ Some tests failed!\n");
        
        $finish;
    end
    
    always @(fuse_blown) begin
        if (fuse_blown && $time > 0)
            $display("  [%0t] ⚡ FUSE STATUS CHANGED: %s", $time, fuse_blown ? "BLOWN" : "OK");
    end
    
    initial begin
        $dumpfile("power.vcd");
        $dumpvars(0, tb_pwr);
    end
    
endmodule