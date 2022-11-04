//! Program that takes throttle messages from CAN and outputs a voltage 0-3.1V from DAC, to
//! control the ESC.
//!
//! Author: Andrew Ealovega

#![no_main]
#![no_std]

use hal::dac::DacOut;

// global logger + panicking-behavior + memory layout
use phnx_throttle as _;

use stm32f7xx_hal as hal;
use stm32f7xx_hal::adc::Adc;
use stm32f7xx_hal::pac::ADC1;
use stm32f7xx_hal::prelude::_embedded_hal_digital_ToggleableOutputPin;

type Can1 = bxcan::Can<hal::can::Can<hal::pac::CAN1>>;

#[rtic::app(
device = crate::hal::pac,
dispatchers = [SDMMC1, DCMI]
)]
mod app {
    use super::hal;
    use crate::{ADC1, Can1};
    use stm32f7xx_hal::rcc::RccExt;
    use stm32f7xx_hal::dac::DacPin;
    use systick_monotonic::*;
    use systick_monotonic::fugit::RateExtU32;
    use bxcan::filter::Mask32;
    use bxcan::ExtendedId;

    use stm32f7xx_hal::{
        rcc::{HSEClock, HSEClockMode},
    };
    use stm32f7xx_hal::adc::Adc;
    use stm32f7xx_hal::gpio::{Output, Pin};
    use stm32f7xx_hal::pac::ADC_COMMON;
    use stm32f7xx_hal::adc::ChannelTimeSequence;

    #[monotonic(binds = SysTick, default = true)]
    type Mono = Systick<1000>;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        dac: hal::dac::C1,
        can: Can1,
        led: Pin<'B', 7, Output>,
        adc: Adc::<ADC1>,
        adc_comm: ADC_COMMON,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut rcc = cx.device.RCC.constrain();

        // To meet CAN clock accuracy requirements, an external crystal or ceramic
        // resonator must be used.
        let _clocks = rcc
            .cfgr
            .hse(HSEClock::new(8_000_000.Hz(), HSEClockMode::Bypass))
            .sysclk(216_000_000.Hz())
            .hclk(216_000_000.Hz())
            .freeze();

        defmt::info!("init");

        let systick = cx.core.SYST;
        let mono = Systick::new(systick, 216_000_000);

        let gpioa = hal::prelude::_stm327xx_hal_gpio_GpioExt::split(cx.device.GPIOA);
        let gpiob = hal::prelude::_stm327xx_hal_gpio_GpioExt::split(cx.device.GPIOB);

        let mut can = {
            // Use alternative pins on the pre-soldered headers.
            let rx = gpiob.pb8.into_alternate();
            let tx = gpiob.pb9.into_alternate();

            let can = hal::can::Can::new(cx.device.CAN1, &mut rcc.apb1, (tx, rx));

            bxcan::Can::builder(can)
                .set_bit_timing(0x001e0005) //1mb: 0x001e0002, 250k: 0x001e000, 500kb: 0x001e0005
                .leave_disabled()
        };
        can.enable_interrupt(bxcan::Interrupt::Fifo0MessagePending);

        // Accept only throttle messages
        let mut filters = can.modify_filters();
        filters.enable_bank(0, Mask32::frames_with_ext_id(ExtendedId::new(0x0000005).unwrap(), ExtendedId::MAX));
        core::mem::drop(filters);

        if can.enable_non_blocking().is_err() {
            defmt::info!("CAN enabling in background...");
        }

        // Configure DAC output
        let dac_pin = gpioa.pa4.into_analog();
        let dac = cx.device.DAC;
        let mut dac = hal::dac::dac(dac, dac_pin);
        dac.enable();

        // Configure ADC pedal in
        let mut adc: Adc::<ADC1> = {
            let adc1 = cx.device.ADC1;

            // Configure with right align, 56 cycles, and 12 bits
            Adc::<ADC1>::adc1(adc1, &mut rcc.apb2, &_clocks, 12, false, true)
        };
        adc.enable_eoc_interrupt();

        // Read ADC off of pin PA0
        let pa0 = gpioa.pa0;
        adc.start_cont_reads(pa0.into_analog());

        // Ye-old debug LED
        let mut led = gpiob.pb7.into_push_pull_output();
        led.set_high();

        // Pass in adc common registers to allow for reading reference voltage
        let adc_comm = cx.device.ADC_COMMON;

        (
            Shared {},
            Local { dac, can, led, adc, adc_comm },
            init::Monotonics(mono),
        )
    }

    use crate::read_can;
    use crate::write_throttle;
    use crate::read_pedal;

    //Extern tasks to make the autocomplete work
    extern "Rust" {
        #[task(local = [dac], capacity = 5, priority = 3)]
        fn write_throttle(_cx: write_throttle::Context, throttle: u8);

        #[task(binds = ADC, local = [adc, adc_comm], priority = 2)]
        fn read_pedal(_cx: read_pedal::Context);

        #[task(binds = CAN1_RX0, local = [can, led], priority = 1)]
        fn read_can(_cx: read_can::Context);
    }
}

/// Fired whenever the ADC has a new reading from the pedal, pressed or not
fn read_pedal(_cx: app::read_pedal::Context) {
    let adc = _cx.local.adc;
    let com = _cx.local.adc_comm;

    let reading = adc.read_cont_data();
    // Voltage reading in mv
    let vol = adc.bits_to_voltage(com, reading);

    if vol < 100 {
        return; // TODO calibrate resting voltage we can ignore
    }

    // Max voltage is 3.8V, so find percent and send it over to throttle write
    let percent = ((3.8 / (vol as f32 / 1000.0)) * 100.0) as u8;

    //TODO send auton kill message

    app::write_throttle::spawn(percent).unwrap();
}

/// Writes 0-3.1V to the ESC, with the percent passed to the task.
fn write_throttle(_cx: app::write_throttle::Context, throttle: u8) {
    // This should be a percent, so just throw out invalid values
    if throttle > 100 {
        defmt::error!("Ignoring invalid throttle percent");
        return;
    }

    let dac = _cx.local.dac;

    /*
    ESC uses differential voltage, so we need to invert our voltage.
    4092 is 3.1V, and the ESCs lowest value that will still move
    with no load is 3.8, so this should match roughly to the
    actual full range the ESC can be set to.
    */
    let out_val = 4092.0 - (throttle as f32 / 100.0) * 4092.0;

    defmt::trace!("Writing {} to DAC", out_val as u16);

    dac.set_value(out_val as u16);
}

/// Receives CAN frame on interrupt, then starts throttle write.
fn read_can(_cx: app::read_can::Context) {
    defmt::trace!("CAN interrupt fired");

    // How else could we know we got a frame?
    let led = _cx.local.led;
    led.toggle();

    let can = _cx.local.can;

    if let Ok(frame) = can.receive() {
        //Percent should be encoded in the first data byte
        let percent = u8::from_le_bytes(frame.data().unwrap()[..1].try_into().unwrap());

        defmt::trace!("Got frame percent: {}", percent);

        app::write_throttle::spawn(percent).unwrap();
    } else {
        defmt::error!("Lost a frame :(");
    }
}
