## TTL transistor-transistor-logic
## binary
    two orders: 0+1+1 -> binary 10 carryout+sum
        carry-in    B   A carry-out sum
        0           1   1   1       0
        (A+B) & (A XOR B) never both 1, max bound to three orders 100
            no double carry in OR, (A+B)=1->10, (A XOR B)=0
## fan-out: no. of logic inps another single logic gate outp can drive
    400mV margin
## logic design
    RTL NOR gate: 1->inp, saturate transistor, outp->0 0 0 ->1
        0.2->0 0.9->1 
        fan-out 4 5 parellel current hogging -> resistor
        delay: switching speed 50ns
        noise immunity/margin 0.5v 0.2v
    RCTL: leading/trailing edge bypass
        power dissipation per gate
    DTL: NAND
        inp 1/vcc 4v -> no cur, resistence, hold saturation 
            collector low 0 outp
        inp 0/gnd -> voltage drop max 0.7v -> not enough
            transistor off, collector Vcc
            0.2v
            1 4v
        fan-out: 8
        delay 25ns
        noise margin: 1.2v series threshold 0.7x2=1.4v
    TTL: emitter-based diode -> inp
            base-collector diode -> series diode
        diffused: collector region <- base <- emitter
        outp stage: active pull-up transistor
    ECL: emitter ref transistor T4
        common emitter resistor high: constant-current source
        inps 0 -> T off, no current, collector vcc, T5 outp 1
        inps higher > ref vbb, cur, collector potential fall -> outp 0
        prevent transistor saturation, fast 
            logic transistor current up, ref T cur down
        n ns
        power dissipation: 50mW
        fan-out: 25
    lowest cost per gate: as many gates as possible on one chip
        12 99 >100 gates
        reliability from minimized interconnection
        89% reduction in pcb
        reduced: connector, design time, assembly labor, test 
            inventory, stockning, handling, negotiations
## TI 54/74
    5v 0-0.2v 1-3v noise: 1v  
    high speed: less resistence, low outp impedence
        clamping diodes for fast transmission line effect
            darlington transistor pair
            6ns per gate
    low power: more resistence, reduction in power, 
        1/10 1mw 33ns speed per gate
    schottky: high speed low power
        high speed: unsaturated logic/ecl
            no minority carries/stored charge
            switching time less
            lower voltage drop than silicon p-n junction
        clamp excess current, prevent saturation
            max current/low voltage drop/closed sw Q2 Q3
                T4 off collector vcc 1 outp
        3ns 20mw
## NAND
    all inp 1 -> Q1 inverse Q2 conduct/voltage drop -> v>1.3 -> outp 0
    transition period: 0.7-1.3 Q2(R2/R3) & Q4 on -> Q3 Q2 Q4 on/current spike
         -> Q4 off -> 0
    any 0 -> v < 0.7 -> outp 1
        base-emitter forward-biased/conducts/saturates
        Q1 collector == Q2 base -> Q1 saturates 
            -> fast removal Q2 stored base charge
            faster turn-off time
        multi-emitter: small, low cost/capcitance, fast switching
        outp Q3 4 low source impedence, allow high capacitance load/outp
            Q4 drive current into load, impeded only by Q3
## worst-senario params test
    pos: current flow towards IH high
        neg: current flow away from IL low
    inpuot voltage vs current 
        temp vcc
        source current OH
        sink current OL
    IL 
        test vcc 4.5v for IL V<0.8v -> normal Vbe drop & diode V drop
            -> test fail: OL greater than max 0.4v sink 1.6mAx10=16mA 0
                vcc max all unused inps 1 -> outp 0
    IH
        emitter-base reverse-biased/off collector-base forward-biased/conducts
            emitter->collector collector->emitter
                inverse current gain of Q1 as inp current
            -> multi emitter inverse in between/leakage instead
                IH split current
                -> test leakage: V IH 5.5v>2v IH<1mA max 5.5mW
                    inp 0, vcc max->max current 
                        unused inps grounded 
        IH>2v at OH min 2.4v
            -> test OH min 2.4v 40uA 10
    outp 
        OL: lower temp/vcc 
                -> OL V higher, more difficult to keep in saturation
            -> test OL max 0.4v 16mA
                all inps 1 2v 
        OH: lower temp/vcc
                -> OH V lower 
            -> test OH min 2.4v source |-400uA| 10
                min inp 0.8v, hold Q3 off
                    unused vcc
            -> test outp short to ground, R4 Q4 D
                OH V = 0
                    all inps ground, max vcc
                -> test |-20mA| < current < |-55mA|
    switch speed 
        propagation delay time outp
         1 -> 0 HL 15ns 7ns
         0 -> 1 LH  22ns 11ns
        higher capacitance, more delay
        HL lower temp/higher vcc, more delay
            higher temp, less delay
        LH higher temp, more delay
            lower temp/higher vcc, less delay
        avg higher vcc, less delay
        more fan-outs, more delay/less voltage <- increased capacitance
    supply current
        average per-gate supply current same 
            0 Icc L max 5.5mA 3mA 2.4v
            1 Icc H max 2mA 1mA 0.4v
        nominal vcc 5v 91% worst-case val
            increase current draw -> charge internal capacitance if no load
            charge discharge
                LH charge by increased current up til outp short current
        lower freq>50khz/higher cap, more current
            higher freq>50khz/lower cap, less current
    dc noise margin
        400 mV -> 1v
        0 <0.4v outp + noise = 0.8v inp 0.2v tolerate 1.15v
        1 >2.4v - noise = 2v 3.3v, tolerate 1.5v
        higer temp, less margin
            less temp, more margin
    ac noise margin
        pulse width shorter than propagation time
            higher-amplitude pulse to outp change 
                then no pulse be propagated thru
            change inp to a different stage long enough
                -> sharp knee curve
        pulse amplitude 
            propagation delay: inp into opposite state
            capacitive storage: store pulse, sequence trigger gate response
                time between pulses > pulse width, no buildup
                not short high-amplitude pulses
        0-to-1 worse than 1-to-0
            immunity NAND gate less: long propagation time to 1 state
    noise rejection
            triggerable storage eles
            fast noise as signal
        circuit impedance <- coupling impedence/stray cap <- noise source
            load impedence by gates
        tight coupling: source effect 
            cant assess, improve noise rejection -> ignored
        ramp inp: dv/dt 0.4v/ns-0.8v/ns 
                time constant T: RC
                eo = T*(1-e(-t/T))
            pulse 4v rising at 0.4v/ns gate 2, 70ohm impedence, 70pF cap
                T = 4.9ns rise time = 4v/0.4/ns = 10ns 
                    norminal val multiple curve by 10, voltage by 4 
                    -> curve T=0.5ns -> peak 0.41v x 4 = 1.64v
                        curve pulse width 50% 
                            = 2.4ns/2 = 1.2ns x 10 = 12ns
                        -> 2v noise immunity, not affected
                1kohm impedence, T = 70ns
                    -> curve T=7ns -> peak 0.9v x 4 = 3.6v
                        pulse width 50% = 12/2 x 10 = 60ns
                            -> cause interference
# specialized gates
    open-collector gates
        wire-AND: single gate 0, others 1, less, 16mA OL
            totem-pole: 0 require sink current from other 1s
                degrade 0 voltage level, high current 55mA one gate short
        Rl max lower logic 1 voltage
            in sink current = (vcc - voh)/(n*Ioh+N*Iih)
             = (5-2.4)/(4*1mA+3*120uA) = 2321ohm
             impedance significantly higher 
                -> restriction for speed, noise, high cap
        Rl min higher logic 0 limit max sink-current 16mA
            = (5-0.4)/(16mA-3*1.6mA) = 410ohm
        n increase, Rl min decrese
    AND Gates
        inverting: delay less 4ns, invert at low power
            eliminate stray cap/inductance from outside
            5mW increase in power
    NOR; res invert NAND
        quadraple 2-inp: Q2 A or Q2 B conducts Q3 on Q4 off -> 0
    AND-OR-INVERT/expander
        one-stage: 15ns max delay, full fan-out 10
        5ns HL T 8ns LH T addition of expander 
            -> node resistance/stray capacitance
                 half delay increase 2.5ns 4ns
            1.6kohm collector resistor 1ns per pF -> 8pF per expander
        introduce delay: add cap to gnd 0.5-1 ns/pF
            change rise/fall time of outp
        IL 2mA
    schmitt
        timing, instability
            slow sources -> square up -> pos feedback to circuit
        high gain hysteresis 800mV
                /poor waveform fed directly to logic devices
            double triggering
            inp current -1.6mA-40uA low-high
                0: source voltage>-1v impedence <(-1-0.8)/1.6=1125ohm
                1: (Voh-Vih)/Iih
        stable hysteresis
            propagation delay 
                inp threshold 0.9 1.7v <- outp 1.5v
                delay HL 30ns max LH 35ns
    normalized fan-out
        1H: less current load/IH 2.4v IL0
                ->OH off/no current collector vcc
            direction: inp/source -> load
        0L: more current sink/IL 0.4v IH1->OL on/saturate collector 0 
            direction: load -> gnd
    unused inps tied to used
        multi emitters same inp 
            1 IL multi IH -> only IH increase
    mix standards: speed power fan-out
        use actual current val <-> generalized fan-out
        generalized loading factor -> double
## flip-flops
### 99% D type for simplicity, J-K rarely used
    one amplifier/transistor collector outp -> another inp 
        one on another off
    latches: add in base resistors
        voltage swing vcc-vce = 5-0.2/0.4=4.6v
            resistor divider/load current drawn -> 3.8v/3.5v
    R-S asychronous control
        reset/clear->Q set/preset 
                0 0 -> 1 1, 1 1 -> no change
            0 0.2v 
            1 3.3v
        S 0, R X -> Q 0 
        R 0, S X -> Qn 1, Qn+1 1
            cant R S both 1 -> unpredictable
    synchronous/clk inps
        d-c/edge-triggered
            pos/neg transition -> particular voltage
                high-speed clking 
                    independent of clk rise/fall times
        a-c coupled: DTL
            respond to a transition time
                clk capacitively coupled internally 
                    -> latching
                    dependent on clk rise/fall <200ns
        master-slave
            two latches in serial
                isolation outp & inp
        clk skew
            propagation delay/gate delay 10ns 
                - hold stable time/clk pulse below 50%/1.5v
                    before: clk setup time
                        activate outp
            capacitive delay tc <- wire routing/lengths
                if tc>10ns, nth change state 
                    before (n+1)st flip-flop clked
    D: a single data & clk inp
        asynchronous/synchronous
        preset/clear: overriding controls 0 1
            D inp state -> clk -> D outp state 
        edge-triggering: d-type latch
            clk high: inp coupled to outp
            clk fall: Q outp hold D state
                -> clk: enable inp to latch
            temp data storage
    T: data T & clk inp
        toggling: T 0, not change
            T 1, clked/outp change state
        counter, divide-by-2
            one cycle for two inp cycles
    J-K: two data & one clk inp
        j/k 1: clked/revert/change state
            X: less gates
        J 0 K X -> Qn 0 Qn+1 0
        J 1 K X -> Qn 0 Qn+1 1
        K 0 J X -> Qn 1 Qn+1 1
        k 1 J X -> Qn 1 Qn+1 0
            J/K 1 -> change state
        NAND Q-K 0 Q' 1 clk Z1
            clk 1, Q 0 -> Z1 outp 1
            clk 1, clear 1, Q' 1
                Q->gated/->Z1 -> k X
                    J -> Qn+1
        master-slave j-k
            setup time >= clk pulse width
                inp held stable until clk falls
                    wait til whole cycle done
                master latch 'remember' 1 state
                    k 0-1-0 clk 1
                        z1 1-0 
                        -> change z2/3/master state
                        z1 1, clk 0, remember 1
                        -> Qn+1 0, Qn+1' 1
    interchangeable
        d->t: Q' outp gated/-> T data inp 
            two AND
        rs->jk: outp gated/-> inp AND
        rs->d: inverter r s conn
        jk->d: inverter k j conn
        jk->t: j k conn
    dc coupled: no capacitive isolation/ac filter out dc
    j-k pos edge triggered
        preset/clear only clk low
    j-k neg 
        High spped 50mhz
    synchronizing
        edge-triggered: stable around clk edge 
            cant be narrower than setup time
        master-slave: all inp width/>20ns
            max 250ns: low power 6 inverter delays
        generate same width outp pulse


## cache line 
    system
            l1 instruction cache l1 data cache
            l2 cache
        l3 cache
        main memory
    alloc cache line to same sized region in main mem
        hit/miss, cache line fill, replacement, writeback
    locality: avoid pollution/use infrequently/not at all
        temporal: used more than once in a short period of time
        spatial: adjacent address data
    prefetch

## ripple counter
    asynchronous/not under one clk pulse
    8-4-2-1 bcd decade counter: cycle length differing from 2^N
        0-9 count in 4-digit binary 
            recycle for every 10 clk pulses
        reset logic 0 bcd 9
    NAND gate: not and 
        0 0 -> 1 
        0 1 -> 1
        1 1 -> 0
    strobe pulse: eliminate decode spike/delay 
        decode occur only after all flip-flops in counter stable
    clk pulse max f
    connect NAND gate outp to reset/preset inp of counter
        2^(n-1) < N < 2^n   2^3 < 10 < 2^4 N=10 0101 
        Q 0 at N-1/Q 1 at N, reset to 0
    eliminate reset propagation time
        latch/delay until stable/feedback loop
## synchronous counter
    parallel carry
    serial/ripple carry: clk speed reduce 50ns -> 100ns
    count up/down
        ignore unused x state
## three modes
     A not internally connected
    bcd
    symmerical divide-by-10
    divide-by-5
## binary divider
    two counters, snd is the a relative prime of the fst

## shift register: cascaded flip-flops
    serial-outp asynchoronous parallel load inp 40mhz
        load inp 1 -> parallel inp, serial outp op inhibited
        load inp 0 
            -> synchronous/serial outp op resumed w/ preset data loaded in
    left/parellel-load/mc 1/clk 2 
    right/mc 0/clk 1: 
        serial inp
        pos/rising edge: L->H, outp rising edge
        neg: H->Lpower efficiency, outp falling edge
    8 4 5 bit
        clear first/set all to 0
    1 clk pulse parallel load inp
        pulse 0, parallel data clked into shift register
    ring counter
        n initiate preset 1, clear others
        feedback -> 4 clk pulses -> valid states
    johnson
        2n states 2^n-1n unused states
            2n-1 cycle
        feedback
            gating
            skip-all-1 Q feedback gates
            self starting Q' feedback gates
            double rail inp/outp for first stage, no last
    linear generator
        long cycle length
        exlusive-OR +'/modulo-2 feedback of the last two stages
        synchronous counter
        2^n-1
        maximum-length subclass mls
            feedback equation 
                jump term A'BCD' 
                + MIS feedback C +' D
                + A'B'C'D' Exit 0000 state 
            jump state 15 - 8 = 7
            self start: 0000 -> 1
    word generator/read-only mem
        state sequence
        6 7 8 9 16-bit 5-stage ABCDE
            start from lsb 9 01001
            transition map/next-state map
    frequency divider 
        n: 1-16 low power
        transition code conversion
            programmable down counter
        n 1 -> 1110
