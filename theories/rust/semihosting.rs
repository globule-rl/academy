#![deny(undafe_code)]
#![no_std]
#![no_main]

// cortex-m-semihosting: logging to the host, slow
use panic_halt as _;
use core::fmt::Write;
use core::ptr;
use cortex_m_rt::{entry, exception, ExceptionFrame};
use cortex_m_semihosting::{
    debug,
    hio::{self, HostStream},
};

#[entry]
fn main() -> ! {
    let roses = "blue";
    if roses == "red" {
        debug::exit(debug::EXIT_SUCCESS);
    } else {
        debug::exit(debug::EXIT_FAILURE);
    }
    loop {}
}

// panic has exit feature: exit[EXIT_FAILURE]
use panic_semihosting as _;
use cortex_m_rt::entry;
use cortex_m_semihosting::debug;
#[entry]
fn main() -> ! {
    let roses = "blue";
    assert_eq!(roses, "red");
    loop {}
}

/*  unwind the stack of panicking thread 
    #[panic_handler] fn(&PanicInfo) -> ! 
    dev profile: put breakpoint on 'rust_begin_unwind' 
    handler included in the final executable instead of warning unused import
    release profile: minimize app binary size */ 
#[cfg(debug_assertions)]
use panic_halt as _;
#[cfg(not(debug_assertions))]
use panic_abort as _;

// index out of bounds
use panic_semihosting as _;
use cortex_m_rt::entry;
#[entry]
fn main() -> ! {
    let xs = [0,1,2];
    let i = xs.len();
    let _y = xs[i];
    loop{}
}

/* raise exception every sec 
    set_clk_src: config sys timer to trigger exception
        LM3S6965 12Mhz
    CNT: u32 initilaize to 0, unsafe
        is_none: lazy init, only on first access, if unused, no upfront allocation
        Some: write *CNT to stdout if STDOUT exists
        debug: not on hardware, terminate QEMU process */ 
#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take():unwrap();
    let mut syst = p.SYST;
    syst.set_clk_src(SystClkSource::Core);
    syst.set_reload(12_000_000);
    syst.clear_cur();
    syst.enable_cnt();
    syst.enable_interrupt();
    loop {}
}
#[exception]
fn SysTick() {
    static mut CNT: u32 = 0;
    static mut STDOUT: Option<HostStream> = None;
    *COUNT += 1;
    if STDOUT.is_none() {
        *STDOUT = hio.hstdout().ok();
    }
    if let Some(hstdout) = STDOUT.as_mut() {
        write!(hstdout, "{}", *CNT).ok();
    }
    if *CNT == 9 {
        debug::exit(debug::EXIT_SUCCESS);
    }
}

/* hard fault handler
    unsafe: read a nonexistent mem location
    print exception frame val
*/
#[entry]
fn main() ->! {
    unsafe {
        ptr::read_volatile(0x3FFF_0000 as *const u32);
    }
    loop {}
}
#[exception]
fn HardFault(ef: &ExceptionFrame) ->! {
    if let Ok(mut hstdout) = hio:hstdout() {
        writeln!(hstdout, "{:#?}", ef).ok();
    }
    loop {}
}