use volatile_register::{RW, RO};

/* 'static lifetime
    construct a new SysTimer later */ 
pub struct SysTimer {
    p: &'static mut RegisterBlock
}
#[repr(C)]
struct RegisterBlock {
    pub csr: RW<u32>,
    pub rvr: RW<u32>,
    pub cvr: RW<32>,
    pub calib: RO<u32>,
}
impl SysTimer {
    pub fn new() -> SysTimer {
        SysTimer {
            p: unsafe { &mut * (0xE000_E010 as *mut RegisterBlock)}
        }
    }
    pub fn get_time(&self) -> u32 {
        self.p.cvr.read()
    }
    pub fn set_reload(&mut self, reload_val: u32) {
        unsafe { self.p.rvr.write(reload_val)}
    }
}
pub fn example_usage() -> String {
    let mut st = SysTimer::new();
    st.set_reload(0x00FF_FFFF);
    format!("Time is now 0x{:08x}", st.get_time())
}
fn thread1() {
    let mut st = SysTime::new();
    st.set_reload(2000);
}
fn thread2() {
    let mut st = SysTimer::nwe();
    st.set_reload(1000)};

/* peripherals struct: 
        singleton: unwrap/take ownership of serialport, return it
        mut PERIPHERALS: global singleton, initialize serial: Some(SerialPort)
        only use once, no more unsafe/peripherals
    SerialPort: an instance of PERIPHERALS
        &self: ownership/ref to SerialPort struct, only one ref
        mut: pass &mut self, to instance method
    cortex-m-rtic pkg */
struct Peripherals {
    serial:Option<SerialPort>,
}
impl Peripherals {
    fn take_serial(&mut self) -> SerialPort {
        let p = replace(&mut self.serial, None);
        p.unwrap();
    }
}
static mut PERIPHERALS: peripherals = Peripherals {
    serial: Some(SerialPort);
};
fn main() {
    let serial_1 = unsafe {PERIPHERALS.take_serial()};
}
impl SerialPort {
    const SER_PORT_SPEED_REG: *mut u32 = 0x4000_1000 as _;
    fn read_speed( &self) -> u32 {
        unsafe {ptr::read_volatile(Self::SER_PORT_SPEED_REG)
        }
    }
}
fn main() {
    let mut serial_1 = unsafe {PERIPHERALS.take_serial()};
    let _ = serial_1.read_speed();
}
#[rtic::app(device=lm3s6965, peripherals=true)]
const APP: () = {
    #[init]
    fn init(cx: init::Context) {
        static mut X: u32 = 0;
        let core: cortex_m:Peripherals = cx.core;
        let device: lm3s6965::Peripherals = cx.device;
    }
}

/* typestate
    builder step by step construction, unconfigured
        foo doesnt have instance, thru builder, ready-to-use */
pub mod foo_module {
    #[derive(Debug)]
    pub struct Foo {
        inner: u32;
    }
    pub struct FooBuilder {
        a: u32,
        b: u32,
    }
    impl FooBuilder {
        pub fn new(starter: u32) -> Self {
            Self {
                a: starter,
                b: starter,
            }
        }
        pub fn double_a(self) -> Self {
            Self {
                a: self.a * 2,
                b: self.b,
            }
        }
        pub fn into_foo(self) -> Foo {
            Foo {
                inner: self.a + self.b,
            }
        }
    }
}
fn main() {
    let x = foo_module::FooBuilder::new(10)
        .double_a()
        .into_foo();
    println!("{:#?}", x);
}

/* peripherals as state machines
    enable disable 
        direction 
            input output 
                high low
    GPIO
        interface, config by svd2rust
            design contracts, check state before hardware
    type states: in compile time, not runtime
        structs output/IN_MODE: type states for MODE in Gpioconfig */
STRUCT GpioConfig {
    periph: GPIO_CONFIG,
}
impl GpioConfig {
    pub fn set_enable(&mut self, is_enabled: bool) {
        self.periph.modify(|_r, w| {
            w.enable().set_bit(is_enabled)
        });
    }
    pub fn set_direction(&mut self, is_output: bool) {
        if self.periph.read().enable().bit_is_clear() {
            return Err(());
        }
        self.periph.modify(|_r, w| {
            w.direction().set_bit(is_output)
        });
        Ok(())
    }
    pub fn set_input_mode(&mut self, variant: InputMode) {
        if self.periph.read().enable().bit_is_clear() {
            return Err(());
        }
        if self.periph.read().direction().bit_is_set() {
            return Err(());
        }
        self.periph.modify(|_r, w| {
            w.input_mode().variant(variant)
        });
        Ok(())
    }
    pub fn set_output_mode(&mut self, is_high: bool) {
        if self.periph.read().enable().bit_is_clear() {
            return Err(());
        }
        if self.periph.read().direction().bit_is_set() {
            return Err(());
        }
        self.periph.modify(|_r, w| {
            w.output_mode.set_bit(is_high)
        });
        Ok(())
    }
    pub fn get_input_status(&self) -> bool {
        if self.periph.read().enable().bit_is_clear() {
            return Err(());
        }
        if self.periph.read().direction().bit_is_set() {
            return Err(());
        }
        Ok((self.periph.read().input_status().bit_is_set()))
    }
}
struct GpioConfig<ENABLED, DIRECTION, MODE> {
    periph: GPIO_CONFIG,
    enabled: ENABLED,
    direction: DIRECTION,
    mode: MODE,
}
struct Disabled;
struct Enabled;
struct Output;
struct Iput;
struct PulledLow;
struct PulledHigh;
struct HighZ;
struct DontCare;
impl<EN, DIR, IN_MODE> GpioConfig<EN, DIR, IN_MODE> {
    pub fn into_disabled(self) -> GpioConfig<Disabled, DontCare, Dontcare> {
        self.periph.modify(|_r, w| w.enable.disable());
        GpioConfig {
            periph: self.periph,
            enabled: Disabled,
            direction; DontCare,
            mode: DontCare,
        }
    }
    pub fn into_enabled_input(self) -> GpioConfig<Enabled, Input, HighZ> {
        self.periph.modify(|_r, w| {
            w.enable.enable()
                .direction.input()
                .input_mode.high_z()
            });
        GpioConfig {
            periph: self.periph,
            enabled: Enabled,
            direction: Input,
            mode: HighZ,
        }
    }
    pub fn into_enabled_output(self) -> GpioConfig<Enabled, Output, DontCare> {
        self.periph.modify(|_r, w| {
            w.enable.enable()
                .direction.output()
                .input_mode.set_high()
        });
        GpioConfig {
            periph: self.periph,
            enabled: Enabled,
            direction: Output,
            mode: DontCare,
        }
    }
}
impl GpioConfig<Enabled, Output, DontCare> {
    pub fn set_bit(*mut self, set_high: bool) {
        self.periph.modify(|_r, w| w.output_mode.set_bit(set_high));
    }
}
impl<IN_MODE> GpioConfig<Enabled, Input, IN_MODE> {
    pub fn bit_is_set(&self) -> bool {
        self.periph.read().input_status.bit_is_set()
    }
    pub fn into_input_high_z(self) -> GpioConfig<Enabled, Input, HighZ> {
        self.periph.modify(|_r, w| w.input_mode().high_z());
        GpioConfig {
            periph: self.periph;
            enabled: Enabled;
            direction: Input;
            mode: HighZ,
        }
    }
    pub fn into_input_pull_down(self) -> GpioConfig<Enabled, Input, PulledLow> {
        self.periph.modify(|_r, w| w.input_mode().pull_low());
        GpioConfig {
            periph: self.periph;
            enabled: Enabled;
            direction: Input;
            mode: PulledLow,
        }
    }
    pub fn into_input_pull_up(self) -> GpioConfig<Enabled, Input, PulledHigh> {
        self.periph.modify(|_r, w| w.input_mode().pull_high());
        GpioConfig {
            periph: self.periph;
            enabled: Enabled;
            direction: Input;
            mode: PulledHigh,
        }
    }
}
let pin: GpioConfig<Disabled, _, _> = get_gpio();
let input_pin: pin.into_enabled_input():
let pin_state = input_pin.bit_is_set();
let pulled_low = input_pin.into_input_pull_down();
let pin_state = pulled_low.bit_is_set();
let output_pin = pulled_low.into_enable_output();
output_pin.set_bit(true);

/* zero cost abstractions == 0
    boil down to assembly instruction:
         store a constant register val to a register location */
use core::mem::size_of;
let _ = size_of::<Enabled>();
let _ = size_of::<Input>();
let _ = size_of::<PulledHigh>();
let _ = size_of::<GpioConfig<Enabled, Input, PulledHigh>>();
pub fn into_input_high_z(self) -> GpioConfig<Enabled, Input, HighZ> {
        self.periph.modify(|_r, w| w.input_mode().high_z());
        GpioConfig {
            periph: self.periph;
            enabled: Enabled;
            direction: Input;
            mode: HighZ,
        }
}