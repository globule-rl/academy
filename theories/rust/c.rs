/* 2025 2032 2042 2052 2072
    preprocessor
        #ifdef compile-time code selection
            cargo[features]
                lib.rs top-level cfg: conditional compilation based on flag -> [features]
                cargo feature for each component
                    fir: finite impulse resopnse iir: infinite impulse response 
                    signal processing primitives compile slow/ large table of constants */
#[cfg(features="FIR")]
pub mod fir;
#[cfg(features="IIR")]          
pub mod iir;
[features]
FIR = []
IIR = []

/* compile time arr sizes/computations
    const fn: evaluable at compile time
        buf: buffer, store byte in mem */
const fn arr_size() -> usize {
    #[cfg(feature="use_more_ram")]
    { 1024 }
    #[cfg(not(feature="use_more_ram"))]
    { 128 }
}
static BUF: [u32; arr_size()] = [0u32; arr_size()];

/* c array access
        slow bounds check
    rust iterators 
        mem safety: check out-of-bound access on muanual arr 
        chaining, enumerating, zipping, find min/max, summing
    c default mutable, explicit const
        rust default const explicit mut
            *ele: &u16, deref to get the val
            &e: &&u16, point to the addr */
int16_t arr[16];
int i;
for (i=0; i<sizeof(arr)/sizeof(arr[0]); i++) {
    process(arr[i]);
}
let arr = [0u16; 16];
for ele in arr.iter() {
    process(*ele);
}

/* volatile c
    rust core::ptr::write_volatile
            &mut T converts to *mut T
        isr: interrupt service routine, save cur state, handle interrupt, restore
            sleep until signaled
            reset signal indicator
            tasks waiting for interrupt */
volatile bool signaled = false;
void ISR() {
    signaled = true;
}
void driver() {
    while(true) {
        while(!signaled) { WFI(); }
        signaled = false;
        run_task();
    }
}
static mut SIGNALED: bool = false;
#[interrupt]
fn ISR() {
    unsafe {core::ptr::write_volatile(&mut SIGNALED, true)};
}
fn driver() {
    loop {
        while unsafe { !core::ptr::read_volatile(&SIGNALED)} {}
        unsafe { core::ptr::write_volatile(&mut SIGNALED, false)};
        run_task();
    }
}

/* c packed struct & aligned var type 
    rust repr(C) to guarantee like C struct order, padding, size
        ordering of struct might change -> x, z, y to improve packing
            0x7fffd0d84c60 0x7fffd0d84c62 0x7fffd0d84c64
        packed version 
            std::ptr::addr_of!(): refs always aligned, get addr of struct fields/raw ptr
            0x7fffd33598490 0x7fffd33598492 0x7fffd33598493
                no padding betwen y z, now z is unaligned
                    set type alignment to 1
        specify alignment
            repr(align(n)) num of bytes to align to, power of 2 
            0x7fffec909a000 0x7fffec909a002 0x7fffec909a004
            0x7fffec909a000 0x7fffec909a002 0x7fffec909a004
                000: both placed on 4096 alighments */
#[repr(C)]
struct Foo {
    x: u16,
    y: u8,
    z: u16,
}
fn main() {
    let v = Foo { x: 0, y: 0, z: 0};
    println!("{:p} {:p} {:p}", &v.x, &v.y, &v.z);
}
#[repr(packed)]
struct Foo {}
fn main() {
    let px = std::ptr::addr_of!(v.x);
}
#[repr(C)]
#[repr(align(4096))]
struct Foo {}
fn main() {
    let v = Foo { x: 0, y: 0, z: 0};
    let u = Foo { x: 0, y: 0, z: 0};
    println!("{:p} {:p} {:p}", &v.x, &v.y, &v.z);
    println!("{:p} {:p} {:p}", &u.x, &u.y, &u.z);
}

/* primitives:
        const char *, &str -> Cstr
        char *, String ->CSting 
        unsigned int, u32 -> c_uint
    wrap func/datatypes
        c -> rust, *mut raw ptr, not &mut unsafe
    build.rs 
    cc crate compile c as dependency to a static lib  */
fn foo(num: u32) {
    let c_num: c_uint = num;
    let r_num: u32 = c_num;
}
typedef struct CoolStruct {
    int x;
    int y;
} CoolStruct;
void cool_function(int i, char c, CoolStruct* cs);
#[repr(C)]
pub struct CoolStruct {
    pub x: cty::c_int,
    pub y: cty::c_int,
}
extern "C" {
    pub fn cool_function(
        i: cty::c_int,
        c: cty::c_char,
        cs: *mut Coolstruct
    )
}
fn main() {
    cc::Build::new()
        .file("src/foo.c")
        .compile("foo");
}

/* c in rust */
[lib]
name = "your_crate"
crate-type = ["cdylib"]
#[no_mangle]
pub extern "C" vn rust_runc() {}

/* optimizing dependencies cargo.toml
        dev profile: debug
        no opt for cortex-m-rt 
        opt-level 
            for size z s increase inline treshold
            for speed
                1 
                2 3 -> speed at the expense of size 
                    3 more vectorization inling */
[package]
name = "app"
[profile.dev.package.cortex-m-rt]
opt-levl = 0
[profile.dev.package."*"]
opt-level = "z"
[targer.'cfg(all(target_arch = "arm", target_os = "none"))]
rustflags = [
    "-C", "inline-threshold=123",
]
[profile.dev.package."*"]
codegen-units = 1
opt-level = "z"

/* math 
    std lib
    libm */
fn main() {
    let float: f32 = 4.82832;
    let floored_float = float.floor();
    let sqrt_of_four = floored_float.sqrt();
    let sinus_of_four = floored_float.sin();
    let exp_of_four = floored_float.exp();
}
#![no_main]
#![no_std]
use panic_halt as _;
use cortex_m_rt::entry;
use cortex_m_semihosting::{debug, hprintln};
use libm::{exp, floorf, sin, sqrtf};
#[entry]
fn main() -> ! {
    let float = 4.82832;
    let floored_float = floor(float);
    let sqrt_of_four = sqrtf(floored_float);
    let sinus_of_four = sin(floored_float.into());
    let exp_of_four = sin(floored_float.into());
    hprintln(float, floored_float).unwrap();
    hprintln(exp_of_four).unwrap();
    loop {}
}
