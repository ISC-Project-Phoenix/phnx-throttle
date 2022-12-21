#![no_main]
#![no_std]

use bxcan::{ExtendedId, Frame};
use hal::dac::DacOut;
use phnx_candefs::{AutonDisable, CanMessage, IscFrame, SetSpeed};
use rtic::mutex_prelude::TupleExt02;
use rtic::mutex_prelude::TupleExt03;

// global logger + panicking-behavior + memory layout
use phnx_throttle as _;

use stm32f7xx_hal as hal;
use stm32f7xx_hal::pac::ADC1;
use stm32f7xx_hal::prelude::_embedded_hal_adc_OneShot;
use systick_monotonic::fugit::Duration;

type Can1 = bxcan::Can<hal::can::Can<hal::pac::CAN1>>;

#[rtic::app(
device = crate::hal::pac,
dispatchers = [SDMMC1, DCMI]
)]
mod app {
    use super::hal;
    use crate::{Can1, Duration, ADC1};
    use bxcan::filter::Mask32;
    use bxcan::{ExtendedId, Fifo};
    use phnx_candefs::{SetSpeed, TrainingMode};
    use stm32f7xx_hal::dac::DacPin;
    use stm32f7xx_hal::rcc::RccExt;
    use systick_monotonic::fugit::RateExtU32;
    use systick_monotonic::*;

    use crate::hal::gpio::Analog;
    use phnx_candefs::IscFrame;
    use stm32f7xx_hal::adc::Adc;
    use stm32f7xx_hal::gpio::{Output, Pin};
    use stm32f7xx_hal::pac::ADC_COMMON;
    use stm32f7xx_hal::rcc::{HSEClock, HSEClockMode};
    use ukalman::*;

    #[monotonic(binds = SysTick, default = true)]
    type Mono = Systick<1000>;

    #[shared]
    struct Shared {
        can: Can1,
        /// If we are in training mode
        training_mode: bool,
        /// True if a pedal input has been received, and no new auton messages have been sent
        auton_disabled: bool,
    }

    #[local]
    struct Local {
        dac: hal::dac::C1,
        led: Pin<'B', 7, Output>,
        adc: Adc<ADC1>,
        adc_comm: ADC_COMMON,
        adc_chan: Pin<'A', 0, Analog>,
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
                .set_loopback(false)
                .leave_disabled()
        };
        can.enable_interrupt(bxcan::Interrupt::Fifo0MessagePending);

        // Filter for throttle and training messages
        let mut filters = can.modify_filters();
        filters.enable_bank(
            0,
            Fifo::Fifo0,
            Mask32::frames_with_ext_id(ExtendedId::new(SetSpeed::ID).unwrap(), ExtendedId::MAX),
        );
        filters.enable_bank(
            1,
            Fifo::Fifo0,
            Mask32::frames_with_ext_id(ExtendedId::new(TrainingMode::ID).unwrap(), ExtendedId::MAX),
        );
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
        let adc: Adc<ADC1> = {
            let adc1 = cx.device.ADC1;

            // Configure with right align, 56 cycles, 12 bits
            Adc::<ADC1>::adc1(adc1, &mut rcc.apb2, &_clocks, 12, false)
        };

        // Read ADC off of pin PA0
        let pa0 = gpioa.pa0.into_analog();

        defmt::info!("Enabled DAC and ADC");

        // Ye-old debug LED
        let mut led = gpiob.pb7.into_push_pull_output();
        led.set_high();

        // Pass in adc common registers to allow for reading reference voltage
        let adc_comm = cx.device.ADC_COMMON;

        read_pedal::spawn_after(Duration::<u64, 1, 1000>::millis(10u64)).unwrap();

        (
            Shared {
                can,
                training_mode: false,
                auton_disabled: false,
            },
            Local {
                dac,
                led,
                adc,
                adc_comm,
                adc_chan: pa0,
            },
            init::Monotonics(mono),
        )
    }

    use crate::read_can;
    use crate::read_pedal;
    use crate::write_throttle;

    //Extern tasks to make the autocomplete work
    extern "Rust" {
        #[task(local = [dac], capacity = 5, priority = 3)]
        fn write_throttle(_cx: write_throttle::Context, throttle: u8);

        #[task(capacity = 5, local = [adc, adc_comm, adc_chan, kalman: Kalman1D = Kalman1D::new(0.0, 100.0)], shared = [can, training_mode, auton_disabled], priority = 1)]
        fn read_pedal(_cx: read_pedal::Context);

        #[task(binds = CAN1_RX0, local = [led], shared = [can, training_mode, auton_disabled], priority = 2)]
        fn read_can(_cx: read_can::Context);
    }
}

/// Fired whenever the ADC has a new reading from the pedal, pressed or not
fn read_pedal(_cx: app::read_pedal::Context) {
    // poll full speed if testing raw voltage
    #[cfg(feature = "vol_out")]
    {
        app::read_pedal::spawn_after(Duration::<u64, 1, 1000>::nanos(1u64)).unwrap();
    }
    #[cfg(not(feature = "vol_out"))]
    {
        app::read_pedal::spawn_after(Duration::<u64, 1, 1000>::millis(10u64)).unwrap();
    }

    defmt::trace!("Polling pedal...");
    let adc = _cx.local.adc;
    let adc_chan = _cx.local.adc_chan;
    let com = _cx.local.adc_comm;

    let app::read_pedal::SharedResources {
        can,
        training_mode,
        auton_disabled,
    } = _cx.shared;

    // Read and normalize our voltage
    let adc_read = adc.read(adc_chan).unwrap();
    let mut vol_mv = adc.bits_to_voltage(com, adc_read);

    defmt::trace!("Raw voltage out ${}$mv", vol_mv);

    #[cfg(feature = "kalman")]
    {
        // Smooth reading with kalman filter.
        let kalman = _cx.local.kalman;
        vol_mv = kalman.filter(vol_mv as f32, 0.05, |x| x, |v| v + 0.001) as u16;
        defmt::trace!("Filter voltage out: %{}%", vol_mv);
    }

    #[cfg(not(feature = "vol_out"))]
    {
        // Filter values outside of ADC range
        if !(100..=2100).contains(&vol_mv) {
            defmt::trace!("Filtered ADC input");
            // If training mode, send 0 message
            (training_mode, can).lock(|tm, can| {
                if *tm {
                    let frame = SetSpeed { percent: 0 }.into_frame().unwrap();
                    let _ = can.transmit(&frame);
                }
            });
            return;
        }
    }

    // Convert to volts
    let vol = vol_mv as f32 / 1000.0;

    // Correct for non-linear voltage divider R2 with R1 = 7.5kohm, R2 = Throttle, Vo = measurement, Vi = 5v
    let throttle_resistence = -(7500.0 * vol / (vol - 5.0));

    // Throttle is linear and 5kohm at max
    let percent = ((throttle_resistence / 5000.0) * 100.0) as u8;
    defmt::trace!("per: {}", percent);

    (can, training_mode, auton_disabled).lock(|can, training_mode, auton_disabled| {
        // Echo pedal reads if in training mode
        if *training_mode {
            #[cfg(feature = "vol_out")]
            {
                let [vol1, vol2] = vol_mv.to_le_bytes();
                let frame = Frame::new_data(
                    ExtendedId::new(0x0000005).unwrap(),
                    [vol1, vol2, 0, 0, 0, 0, 0, 0],
                );
                let _ = can.transmit(&frame);
            }
            #[cfg(not(feature = "vol_out"))]
            {
                let frame = SetSpeed { percent }.into_frame().unwrap();
                let _ = can.transmit(&frame);
            }
        }

        // Disable auton if we are currently in auton and the driver has pressed the pedal
        if !*auton_disabled {
            defmt::info!("Disabling auton");
            let frame = AutonDisable {}.into_frame().unwrap();

            // If we couldn't send can message, it'll just retry on next ADC cycle
            if can.transmit(&frame).is_ok() {
                *auton_disabled = true;
            }
        }
    });

    app::write_throttle::spawn(percent as u8).unwrap();
}

/// Writes 3.1-0V to the ESC, with the percent passed to the task.
fn write_throttle(_cx: app::write_throttle::Context, throttle: u8) {
    // This should be a percent, so just throw out invalid values
    if throttle > 100 {
        defmt::error!("Ignoring invalid throttle percent: {}", throttle);
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

    let can = _cx.shared.can;
    let training = _cx.shared.training_mode;
    let auton_disabled = _cx.shared.auton_disabled;

    (can, training, auton_disabled).lock(|can, training_mode, auton_disabled| {
        if let Ok(frame) = can.receive() {
            if let Ok(conv) = CanMessage::from_frame(frame) {
                match conv {
                    CanMessage::SetSpeed(sp) => {
                        // If we receive a set throttle, we must be in auton
                        *auton_disabled = false;

                        defmt::trace!("Got frame percent: {}", sp.percent);

                        app::write_throttle::spawn(sp.percent).unwrap();
                    }
                    CanMessage::TrainingMode(_) => {
                        defmt::info!("Engaging Training Mode");
                        *training_mode = true;
                    }
                    _ => {
                        defmt::error!(
                            "Invalid CAN id received! This is an error in the filter or candefs."
                        )
                    }
                }
            }
        } else {
            defmt::error!("Lost a frame :(");
        }
    });
}
