#![no_std]
#![no_main]

/* hardware f3
    run on QEMU using semihosting
    !: no ret
    debug::exit: exit QEMU */ 
use panic_halt as _;
use cortex_m_rt::entry;
use cortex_m_semihosting::{debug, hprintln};
#[entry]
fn main() -> ! {
    hprintln("hello world").unwrap();
    debug::exit(debug::EXIT_SUCCESS);
    loop {}
}


/* mircro-architecture 
    Cortex-M SysTick peripheral, 24-bit sys timer
    peripherals: only one SYST struct
    !has_wrapped: delay for x millisecs, continue until overflows/wrap around */ 
use cortex_m::peripheral::{syst, Peripherals};
use cortex_m_rt::entry;
use panic_halt as _;
#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let mut systick = peripherals.SYST;
    systick.set_clock_source(syst::SystClkSource::Core);
    systick.set_reload(1_000);
    systick.clear_current();
    systick.enable_counter();
    while !systick.has_wrapped() {
        // loop
    }
    loop{}
}

/* PAC: peripheral access crate
    off-chip peripheral mem mapped registers
    tm4c123 80MHz Cortex-M4 256KiB flash 
    ctl: control register
    globalsync0(): bit 0 update pwm generator 0 on slice 0
    clear_bit(): set bit to 0, disable global
    _2_ctl: mode 1 count up/down mode
        528 cycles(264 up & down) 4 loops per video line 2112 cycles
    unsafe: 
        svd(xml) doeant say if e.g. 32-bit has a special meaning
        bits(): raw bit writes w/o register field validation
        0-263->264 ticks cmpa: compare duty cycle 64/264=24.24%
    pwm freq = clk/(LOAD+1)*div = clk/528 = 16 MHz/528 = 30 kHz 
    modify(|r, w|) in c: write 2 temps
        uint32_t temp = pwm0.ctl.read();
        temp |= PWM0_CTL_GLOBALSYNC0;
        pwm0.ctl.write(temp);
        uint32_t temp2 = pwm0.enable.read();
        temp2 |= PWM0_ENABLE_PWM4EN;
        pwm0.enable.write(temp2); */ 
use panic_halt as _;
use cortex_m_rt::entry;
use tm4c123x;
#[entry]
pub fn init() -> (Delay, Leds) {
    let cp = cortex_m::Peripherals::take().unwrap();
    let p = tm4c123x::Peripherals::take().unwrap();
    let pwm = p.PWM0;
    pwm.ctl.write(|w| w.globalsync0().clear_bit());
    pwm._2_ctl.write(|w| w.enable().set_bit().mode().set_bit());
    pwm._2_gena.write(|w| w.actcmpau().zero().actcmpad().one());
    pwm._2_load.write(|w| unsafe {w.load().bits(263)});
    pwm._2_cmpa.write(|w| unsafe {w.compa().bits(64)});
    pwm.enable.write(|w| w.pwm4en().set_bit());
    if pwm.ctl.read().globalsync0().is_set() {
    }
    pwm.ctl.modify(|r, w| w.globalsync0().clear_bit());
}

/* HAL: hardware abstract layer, api 
    write_byte GPIO, Serial, I2C, SPI, timer, analog-digital
    constrain(): wrap up SYSCTL struct into higher-layer api obj
    oscillator settings
    Pll: phase-locked loop, output high-freq phase fixed to inp low-freq phase
    clocks: use the sc settings above
    p.GIO_PORTA: wrap up GPIO_PORTA struct to higher-layer api obj
        power_control: power up GPIO automatically
    Serial::uart0 args: activate uart serial
        pa1: transmit pin, port a pin 1 -> AF1 alternate function
        push_pull: high & low
        pa0: receive pin
        115200: baud rate
        output handling */ 
use panic_halt as _;
use cortex_m_rt::entry;
use tm4c123x_hal as hal;
use tm4c123x_hal::prelude::*;
use tm4c123x_hal::serial::{NewlineMode, Serial};
use tm4c123x_hal::sysctl;
#[entry]
fn main() -> ! {
    let p = hal::Peripherals:take().unwrap();
    let cp = hal::CorePeripherals::take().unwrap();
    let mut sc = p.SYSCTL.constrain();
    sc.clock_setup.oscillator = sysctl::Oscillator::Main(
        sysctl::CrystalFrequency::_16mhz,
        sysctl::SystemClock::UsePll(sysctl::PllOutputFrequency::_80_00mhz),
    );
    let clocks = sc.clock_setup.freeze();
    let mut porta = p.GPIO_PORTA.split(&sc.power_control);
    let uart = Serial::uart0(
        p.UART0,
        porta
            .pa1
            .into_af_push_pull::<hal::gpio::AF1>(&mut porta.control),
        porta
            .pa0
            .into_af_push_pull::<hal::gpio::AF1>(&mut porta.control),
        (),
        (),
        115200_u32.bps(),
        NewlineMode::SwapLFtoCRLF,
        &clocks,
        &sc.power_control,
    );
    loop {
        writeln!(uart, "hi\r\n").unwrap();
    }
}