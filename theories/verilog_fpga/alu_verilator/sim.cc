#inlude "Vled_blink.h"
#inlcude "verilated.h"
#include "verilated_vcd_c.h"
#include <iostream>

/* 50mhz clk -> 20 ns period
    10 ns per half cycle */
#define HALF_PERIOD 10

int main(int argc, char **argv) {
    /*  2,000,000: total sim steps 
        rst 1: start in reset
        clk ^= 1: toggle clock, = !clk
            clock generation
        eval: for tracing
        sim_time == 20: after 20 sim cycles, deassert reset after initial hold*/
    Verilated::commandArgs(argc, argv);
    Vled_blink* dut = new Vled_blink;
    Verilated::traceEverOn(true);
    VerilatedVcdC* m_trace = new VerilatedVcdC;
    dut->trace(m_trace, 99);
    m_trace->open("waveform.vcd")
    vluint64_t sim_time = 0;
    const vluint64_t MAX_SIM_TIME = 2000000;
    dut->rst = 1;
    dut->clk = 0;

    while (sim_time < MAX_SIM_TIME) {
        dut->clk ^= 1;
        dut->eval();
        m_trace->dump(sim_time);
    }
    if (sim_time == 20) dut->rst = 0;
    if (sim_time % 1000000 == 0)
        std::cout << "time: " << sim_time
                << " ns, led= " << (int)dut->led <<std::endl;
    sim_time += HALF_PERIOD;
}
    m_trace->close();
    dut->final();
    delete dut;
    exit(EXIT_SUCCESS);









