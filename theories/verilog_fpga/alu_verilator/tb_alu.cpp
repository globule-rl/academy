#include <stdlib.h>
#include <iostream>
#include <verilated.h>
#include <verilated_vcd_c.h>
#include "Valu.h"
#include "Valu___024unit.h"
#include <cstdlib>

/* starttime: after reset */
#define MAX_SIM_TIME 300
#define VERIF_START_TIME 7 
vluint64_t sim_time = 0;

class AluInTx {
	public:
		uint32_t a;
		uint32_t b;
		enum Operation {
			add = Valu___024unit::operation_t::add,
			sub = Valu___024unit::operation_t::sub,
			nop = Valu___024unit::operation_t::nop
		} op;
};

class AluOutTx {
	public:
		uint32_t out;
};

class AluInDrv {
	/* transactions drive to inp interface pins
		always_comb begin
			dut.in_valid =1'b0;
			if (tx item exists && tx item operation != NOP) begin
				dut.in_valid = 1'b1;
			end
		end */
	private:
		Valu *dut;
	public:
		AluInDrv(Valu *dut) {
			this->dut = dut;
		}
		void drive(AluInTx *tx) {
			dut->in_valid = 0;
			if (tx != NULL) {
				if (tx->op != AluInTx::nop) { 	
					dut->in_valid = 1;
					dut->op_in = tx->op;
					dut->a_in = tx->a;
					dut->b_in = tx->b;
				}
				delete tx;
			}
		}
};

class AluInMon {
	private:
		Valu *dut;
		AluScb *scb;
	public:
		AluInMon(Valu *dut, AluScb *scb) {
			this->dut = dut;
			this->scb = scb;
		}
		void monitor(){
			if (dut->in_valid == 1) {
				AluInTx *tx = new AluInTx();
				tx->op = AluInTx::Operation(dut->op_in);
				tx->a = dut->a_in;
				tx->b = dut->b_in;
				scb->writeIn(tx);
			}
		}
};

class AluOutMon {
	private:
		Valu *dut;
		AluScb *scb;
	public:
		AluOutMon(Valu *dut, AluScb *scb) {
			this->dut = dut;
			this->scb = scb;
		}
		void monitor() {
			if (dut->out_valid == 1) {
				AluOutTx *tx = new AluOutTx();
				tx->out = dut->out;
				scb->writeOut(tx);
			}
		}
}

class AluScb {
	private:
		std::deque<AluInTx*> in_q;
	public:
		void writeIn(AluInTx *tx) {
			in_q.push_back(tx);
		}

		void writeOut(AluOutTx *tx) {
			if (in_q.empty()) {
				std::cout << "Fatal error in AluScb: empty AluInTx queue" << std::endl;
				exit(1);
			}

			AluInTx *in;
			in = in_q.front();
			in_q.pop_front();
			switch (in->op) {
				case AluInTx::nop:
					std::cout << "Fatal error in Aluscb, received NOP on input" << std::en
					exit(1);
					break;
				case AluInTx::add:
					if (in->a + in->b != tx->out) {
						std::cout << std::endl;
						std::cout << "AluScb: add mismatch" << std::endl;
						std::cout << " Expected: " << in->a + in->b
								  << " Actual: " << tx->out << std::endl;
						std::cout << " Simtime: " << sim_time << std::endl;
					}
					break;
				case AluInTx::sub:
					if (in->a - in->b != tx->out) {
						std::cout << std::endl;
						std::cout << "AluScb: sub mismatch" << std::endl;
						std::cout << " Expected: " << in->a - in->b
								  << " Actual: " << tx->out << std::endl;
						std::cout << " Simtime: " << sim_time << std::endl;
					}
					break;
			}
			delete in;
			delete tx;
		}
};

void dut_reset(Valu *dut, vluint64_t &sim_time){
	/* always_comb begin
			dut.rst = 1'b0;
			if (sim_time >= 3 && sim_time < 6) begin
				dut.rst = 1'b1;
			end
	 	end */ 
	dut->rst = 0;
	if(sim_time > 1 && sim_time < 5){
		dut->rst = 1;
		dut->a_in = 0;
		dut->b_in = 0;
		dut->op_in = 0;
		dut->in_valid = 0;
	}
}

AluInTx* rndAluInTx() {
	/* transaction generator tx
		%5: 20% chance generating a transaction
			%3: 0 1 2
			a: 10-20
			b: 0-5 
		else: not generating or state-based-opposite time-based sequence */ 
	if (rand() % 5 == 0) { 
		AluInTx *tx = new AluInTx();
		tx->op = AluInTx::Operation(ran() % 3) 
		tx->a = rand() % 11 + 10; 
		tx->b = rand() % 6; 
		return tx;
	} else {
		return NULL; 
	}
}
	
int main(int argc, char** argv, char** env) {
	/* 	generate transaction items
	 		-> driver -> inp/out monitor -> scoreboard
	 	clk == 1: high 
		alu: arithmetic logic unit
		dut: device under test */
	srand (time(NULL));
	Verilated::commandArgs(argc, argv);
	Valu *dut = new Valu;
	Verilated:traceEverOn(true);
	VerilatedVcdC *m_trace = new VerilatedVcdC;
	dut->trace(m_trace, 5);
	m_trace->open("waveform.vcd");
	AluInTx *tx;
	AluInDrv *drv	= new AluInDrv(dut);
	AluMon 	 *inMon = new AluInMon(dut, scb);
	AluOutMon *outMon = new AluOutMon(dut,scb);
	AluScb	 *scb	= new AluScb();

	while (sim_time < MAX_SIM_TIME) {
		dut_reset(dut, sim_time);
		dut->clk ^= 1;
		dut->eval();
		if (dut->clk == 1) {
			if (sim_time >= VERIF_START_TIME) {
				tx = rndAluInTx();
				drv->drive(tx);
				inMon->monitor();
				outMon->monitor();
			}
		}
		m_trace->dump(sim_time);
		sim_time++;
	}
	
	m_trace->close();
	delete dut;
	delete inMon;
	delete outMon;
	delete scb;
	delete drv;
	exit(EXIT_SUCCESS);
}	
