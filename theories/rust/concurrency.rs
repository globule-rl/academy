/* count rising edges of input signal
    no concurrency */ 
#[entry]
fn main() {
    let peripherals = setup_peripherals();
    loop {
        let inputs = read_inputs(&peripherals);
        let outputs = process(inputs);
        write_outputs(&peripherals, outputs);
    }
}

/* static mut: unsafe global data, statically allocated mutable memory
    data races: unsafe { CNT += 1 } 
        critical ensures synchronised access to CNT */
static mut CNT: u32 = 0;
#[entry]
fn main() -> ! {
    set_timer_1hz();
    let mut last_state = false;
    loop {
        let state = read_signal_level();
        if state && !last_state {
            cortex_m::interrupt::free(|_| {
                unsafe { CNT += 1};
            });
        }
        last_state = state;
    }
}
#[interrupt]
fn timer() {
    unsafe { CNT = 0;}
}


/* atomic access: 
    compare and swap instruction 
        even multiple core
        require unsafe
    fetch_add(): atomically add 1 to CNT 
    CNT.store(): write 0 directly to CNT */   
use core::sync::atomic::{AtomicUsize, Ordering};
static CNT: AtomicUsize = AtomicUsize::new(0);
#[entry]
fn main() -> ! {
    set_timer_1hz();
    let mut last_state = false;
    loop {
        let state = read_signal_level();
        if state && !last_state {
            CNT.fetch_add(1, Ordering::Relaxed);
        }
        last_state = state;
    }
}
#[interrupt]
fn timer() {
    CNT.store(0, Ordering::Relaxed)
}

/* abstraction 
    a safe interface: critical-section cnt 
    wrapper around UnsafeCell<u32> 
        CNT static, not static mut, can still mut val
        inside critical: can use unsafeCell:get 
    sync: static CSCnt, shared between multiple threads 
        interrupt::free.increment: no need unsafe, CNT no longer mut, use interior mutability
    interrupt::free.reset: enter critical to obtain cs token */
use core::cell::UnsafeCell;
use cortex_m::interrupt;
struct CSCnt(UnsafeCell<u32>);
const CS_CNT_INIT: CSCnt = CSCnt(UnsafeCell::new(0));
impl CSCnt {
    pub fn reset(&self, _cs: &interrupt::CriticalSection) {
        unsafe { *self.0.get() = 0 };
    }
    pub fn increment(&self, _cs: &interrupt::CriticalSection) {
        unsafe { *self.0.get() += 1};
    }
}
unsafe impl Sync for CSCnt {}
static CNT: CSCnt = CS_CNT_INIT;
#[entry]
fn main() -> ! {
    set_timer_1hz();
    let mut last_state = false;
    loop {
        let state = read_signal_level();
        if state && !last_state {
            interrupt::free(|cs| CNT.increment(cs));
        }
        last_state = state;
    }
}
#[interrupt]
fn timer() {
    interrupt::free(|cs| CNT.reset(cs));
}

/* mutexes u32 type
    lock the mutex, when thread is done, unlock 
    Cell/Send: UnsafeCell with safe interface, not Sync
    Mutex<T> implement Sync, safe: interrupt in critical section */
use core::cell::Cell;
use cortex_m::interrupt::Mutex;
static CNT: Mutex<Cell<u32>> = Mutex::new(Cell::new(0));
#[entry]
fn main() -> ! {
    set_timer_1hz();
    let mut last_state = false;
    loop {
        let state = read_signal_level();
        if state && !last_state {
            interrupt::free(|cs|
                CNT.borrow(cs).set(CNT.borrow(cs).get() + 1));
        }
        last_state = state;
    }
}
#[interrupt]
fn timer() {
    interrupt::free(|cs| CNT.borrow(cs).set(0));
}

/* sharing peripherals: refs not copies, peripheral struct type
    only one instance exists at a time
    RefCell runtime check
    move peripheral into shared var after initialised in main
        Option None->instance later
    dp: obtain peripheral singletons, configure PA0->input, PA1->output
    borrow(): on mutex, critical section, ref to RefCell, replace() move new val to RefCall
        MY_GPIO: safe and concurrent, no longer access gpioa/dp.GPIOA, only thru mutex 
            loop:
                as_ref(): RefCell keeps tracks of how long &Option<> remains borrowed
                    convert GPIOA to &Option<&GPIOA>, unwrap(), obtain &GPIOA. modify peripheral
                read(): read state as input
                modify|w|: write PA1 high as seeing !last_state rising edge 
    interrupt: clear PA0 
        unwrap(): interrupt enabled after MY_GPIO set set_timer_1hz(), otherwise handle None */
use core::cell::RefCell;
use cortex_m::interrupt::{self, Mutex};
use stm32f4::stm32f405;
static MY_GPIO: Mutex<RefCell<Option<stm32f405::GPIOA>>> = 
                    Mutex::new(RefCell::new(None));
#[entry]
fn main() -> ! {
    let dp = stm32f405::peripherals::take().unwrap();
    let gpioa = &dp.GPIOA;
    configure_gpio(gpioa);
    interrupt::free(|cs| MY_GPIO.borrow(cs).replace(Some(dp.GPIOA)));
    set_timer_1hz();
    let mut last_state = false;
    loop {
        let state = interrupt::free(|cs| {
            let gpioa = MY_GPIO.borrow(cs).borrow();
            gpioa.as_ref().unwrap().idr.read().idr0().bit_is_set()
        });
        if state && !last_state {
            interrupt::free(|cs| {
                let gpioa = MY_GPIO.borrow(cs).borrow();
                gpioa.as_ref().unwrap().odr.modify(|_, w| w.ordr1().set_bit());
            });
        }
        last_state = state;
    }
}
#[interrupt]
fn timer() {
    interrupt::free(|cs| {
        let gpioa = MY_GPIO.borrow(cs).borrow();
        gpios.as_ref().unwrap().odr.modify(|_, w| w.odr1().clear_bit());
    });
}

/* TIM2 timer 
    muteable ref to a shared resource:
        borrow_mut deref_mut
    config(): NVIC interrupt, start timer 
    asm: miscellaneous assembly instructions
        wfi: wait for interrupt */
use core::cell::RefCell;
use core::ops::DerefMut:
use cortex_m::interrupt::{self, Mutex};
use cortex_m:asm::wfi;
use stm32f4::stm32f405;
static G_TIM: Mutex<RefCall<Option<Timer<stm32::TIM2>>>> = 
                Mutex::new(RefCall::new(None));
#[entry]
fn main() -> ! {
    let mut cp = cm::peripherals::take().unwrap();
    let dp = stm32f405::peripherals::take().unwrap();
    let tim = configure_timer_interrupt(&mut cp, dp);
    interrupt::free(|cs| {
        G_TIM.borrow(cs).replace(Some(tim));
    });
    loop {
        wfi();
    }
}
#[interrupt]
fn timer() {
    interrupt::free(|cs| {
        if let Some(ref mut tim) = G_TIM.borrow(cs).borrow_mut().deref_mut() {
            tim.start(1.hz());
        }
    });
}              

/* collections: dynamic data structures
    heap-allocated 
        bump pointer allocator 
            start: move start up to the next alignment boundary 
            ensure mem [0x2000_0100, 0x2000_0200] not used 
            out of mem error handling 
                bkpt: put the processor in debug state, breakpoint
            use collections in alloc
    heapless/fixed capacity
        U8: declare upfront capacity, 8 ele
        push: can fail, res indicating, alloc will realloc/increase capacity on heap
        inline store eles, stack/static var/heap Box<Vec<_, _>>  */
use core::alloc::{GlobalAlloc, Layout};
use core::cell::UnsafeCell;
use core::ptr;
use cortex_m::interrupt;
struct BumpPointerAlloc {
    head: UnsafeCell<usize>,
    end: usize,
}
unsafe impl Sync for BumpPointerAlloc {}
unsafe impl GlobalAlloc for BumpPointerAlloc {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        interrupt::free(|_| {
            let head = self.head.get()
            let size = layout.size();
            let align = layout.align();
            let align_mask = !(align-1);
            let start = (*head + align - 1) & align_mask;
            if start + size > self.end {
                ptr::null_mut()
            } else {
                *head = start + size;
                start as *mut u8
            }
        })
    }
    unsafe fn dealloc(&self, _: *mut u8, _: Layout) {
        // never dealloc mem
    }
}
#[global_allocator]
static HEAP: BumpPointerAlloc = BumpPointerAlloc {
    head: UnsafeCell::new(0x2000_0100),
    end: 0x2000_0200,
};
#![feature(alloc_error_handler)]
use cortex_m::asm;
#[alloc_error_handler]
fn on_oom(_layout: Layout) -> ! {
    asm::bkpt();
    loop {}
}
#[entry]
fn main() -> ! {
    let mut xs = Vec::new();
    xs.push(42);
    assert!(xs.pop(), Some(42));
    loop {}
}

use heapless::Vec;
use heapless::consts::*;
#[entry]
fn main() -> ! {
    let mut xs: Vec<_, U8> = Vec::new();
    xs.push(42).unwrap();
    assert_eq!(xs.pop(), Some(42));
    loop {}
}
