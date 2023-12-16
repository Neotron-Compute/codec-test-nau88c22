//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use panic_probe as _;

use rp2040_hal::gpio::{FunctionPio1, Pin, PullNone};
// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    fugit::RateExtU32,
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

mod i2s;

// static CLIP: [u8; 403052] = *include_bytes!("test audio.raw");
static CLIP: [u8; 1024] = [0; 1024];

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // This is the correct pin on the Raspberry Pico board. On other boards, even if they have an
    // on-board LED, it might need to be changed.
    // Notably, on the Pico W, the LED is not connected to any of the RP2040 GPIOs but to the cyw43 module instead. If you have
    // a Pico W and want to toggle a LED with a simple GPIO output pin, you can connect an external
    // LED to one of the GPIO pins, and reference that pin here.
    let mut led_pin = pins.led.into_push_pull_output();
    let sda_pin = pins.gpio14.into_function();
    let scl_pin = pins.gpio15.into_function();

    let i2c = bsp::hal::I2C::i2c1(
        pac.I2C1,
        sda_pin,
        scl_pin,
        100.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    let _i2s_adc_data: Pin<rp2040_hal::gpio::bank0::Gpio22, FunctionPio1, PullNone> =
        pins.gpio22.reconfigure();
    let _i2s_dac_data: Pin<rp2040_hal::gpio::bank0::Gpio26, FunctionPio1, PullNone> =
        pins.gpio26.reconfigure();
    let _i2s_bit_clock: Pin<rp2040_hal::gpio::bank0::Gpio27, FunctionPio1, PullNone> =
        pins.gpio27.reconfigure();
    let _i2s_lr_clock: Pin<rp2040_hal::gpio::bank0::Gpio28, FunctionPio1, PullNone> =
        pins.gpio28.reconfigure();

    let mut player = i2s::init(pac.PIO1, &mut pac.RESETS);
    unsafe {
        cortex_m::interrupt::enable();
    }

    let mut codec = nau88c22::Codec::new(i2c, Some(12_000_000));

    let _ = codec.reset();

    for _ in 0..10 {
        delay.delay_ms(100);
        info!(
            "Device ID: {:?}",
            codec.check_device_id().map_err(clean_error)
        );
    }

    // R1 Bit 2, IOBUFEN, set to logic = 1
    // R1 Bit 8, DCBUFEN, set to logic = 1 if setting up for greater than 3.60V operation
    codec
        .modify_powermanagement1(|mut w| {
            w.iobufen_set(true);
            w.dcbufen_set(true);
            w
        })
        .unwrap();
    info!(
        "powermanagement1 = {:?}",
        defmt::Debug2Format(&codec.read_powermanagement1().map_err(clean_error))
    );

    // R1 Bits 1, Bit 0, REFIMP set to 80kΩ setting
    // R1 Bit 2, ABIASEN, set to logic = 1
    // Value to be written to R1 = 0x10D
    codec
        .modify_powermanagement1(|mut w| {
            w.refimp_set(1);
            w.abiasen_set(true);
            w
        })
        .unwrap();
    info!(
        "powermanagement1 = {:?}",
        defmt::Debug2Format(&codec.read_powermanagement1().map_err(clean_error))
    );

    // wait for caps to charge
    delay.delay_ms(250);

    loop {
        info!("play!");
        led_pin.set_high().unwrap();
        let mut count = 0;
        while count < CLIP.len() {
            count += player.play_samples_16bit_stereo(&CLIP);
        }
        led_pin.set_low().unwrap();
        info!("stop!");
        delay.delay_ms(1000);
    }
}

fn clean_error<T>(e: nau88c22::Error<T>) -> nau88c22::Error<()> {
    match e {
        nau88c22::Error::I2c(_) => nau88c22::Error::I2c(()),
        nau88c22::Error::WrongDeviceId => nau88c22::Error::WrongDeviceId,
    }
}

// End of file
