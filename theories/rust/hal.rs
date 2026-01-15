/* interoperability
    non-cpy wrapper provided by HAL: 
        free(), consume wrapper, ret raw peripheral 
        objs constructed(i/o pin), ret tuple */
pub struct Timer(TIMER0);
impl Timer {
    pub fn new(periph: TIMER0) -> Self {
        Self(periph)
    }
    pub fn free(self) -> TIMER0 {
        self.0
    }
}

/* GPIO interface exposed by hal
    split(): zero-sized/cost abstraction
                    no runtime cost for unused features
                        compile high-level features to machine code
                            generic func <T>, type-specific code
                when all pin assignments are known */
pub struct PA0;
pub struct PA1;
pub struct PortA;
impl PortA {
    pub fn split(self) -> PortApins {
        PortApins {
            pa0: PA0,
            pa1: PA1,
        }
    }
}
pub struct PortApins {
    pub pa0: PA0,
    pub pa1: PA1,
}

/* erase_pin(): 
        move properties from compile time to runtime
             more flexibility
        pin: u8: a pin on port A  
        PA0 -> PA -> Pin -> Port encoding
            store runtime data in struct/enums/trait/obj, avoid vtables       */
pub struct PA0;
impl PA0 {
    pub fn erase_pin(self) -> PA {
        PA { pin: 0}
    }
}
pub struct PA {
    pin: u8,
}
pub struct PA {
    pub fn erase_port(self) -> Pin {
        Pin {
            port: Port::A,
            pin: self.pin,
        }
    }
}
pub struct Pin {
    port: Port,
    pin: u8,
}
enum Port {
    A,
    B,
    C,
}

/* pin state: encoded as type parameters
    into_input, into_output, 
    with_input_state, with_output_state
        temp reconfig pin in diff state w/o moving it
    bound by sealed traits 
    pinstate outputstate inputstate: sealed
    phantomdata pinstate sealed 
        _p: marker act like &S ref/own, phantom types never used, but have the same lifetime
            ouputstate never used, output has the same lifetime as outputstate
        outputstate
            pushpull opendrain
        inputstate
            floating pullup pulldown
        pinstate PA1 */
mod sealed {
    pub trait Sealed {}
}
pub trait PinState: sealed::Sealed {}
pub trait OutputState: sealed::Sealed {}
pub trait InputState: sealed::Searled {}
pub struct Output<S: OutputState> {
    _p: PhantomData<S>,
}
impl<S: OutputState> PinState for Output<S> {}
impl<S: OutputState> sealed::Sealed for Output<S> {}
pub struct PushPull;
pub struct OpenDrain;
impl OutputState for PushPull {}
impl OutputState for OpenDrain {}
impl sealed::Sealed for PushPull {}
impl sealed::Sealed for OpenDrain {}
pub struct Input<S: InputState> {
    _p: PhantomData<S>,
}
impl<S: InputState> PinState for Input<S> {}
impl<S: InputState> sealed::Sealed for Input<S> {}
pub struct Floating:
pub struct PullUp;
pub struct PullDown;
impl InputState for Floating {}
impl InputState for Pullup {}
impl InputState for PullDown {}
impl sealed::Sealed for Floating {}
impl sealed::Sealed for Pullup {}
impl sealed::Sealed for PullDown {}
pub struct PA1<S: PinState> {
    _p: PhantomData<S>,
}
impl<S: Pinstate> PA1<S> {
    pub fn into_input<N: InputState>(self, input: N) -> PA1<Input<N>> {
        todo!()
    }
    pub fn into_output<N: OutputState>(self, output: N) -> PA1<Output<N>> {
        todo!()
    }
    pub fn with_input_state<N: InputState, R>(
        &mut self,
        input: N,
        f: impl FnOnce(&mut PA1<N>) -> R,
    ) -> R {
        todo!()
    }
    pub fn with_output_state<N: OutputState, R>(
        &mut self, 
        output: N,
        f: impl FnOnce(&mut PA1<N>) -> R, 
    ) -> R {
        todo!()
    } 
}