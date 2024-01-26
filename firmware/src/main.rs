//! NAU88C22 CODEC test
//!
//! Requires wiring up a Raspberry Pi Pico as follows:
//!
//! ```text
//!         +----------------+
//! UART TX<|GP0         VBUS|
//! UART RX>|GP1         VSYS|
//!         |GND          GND|
//!         |GP2       3V3_EN|
//!         |GP3          3V3|
//!         |GP4     ADC_VREF|
//!         |GP5         GP28|<I2S_FS
//!         |GND         AGND|
//!         |GP6         GP27|<I2S_BCLK
//!         |GP7         GP26|>I2C_DAC
//!         |GP8          RUN|
//!         |GP9         GP22|<I2S_ADC
//!         |GND          GND|
//!         |GP10        GP21|
//!         |GP11        GP20|
//!         |GP12        GP19|
//!         |GP13        GP18|
//!         |GND          GND|
//!    SDA<>|GP14        GP17|
//!     SCL<|GP15        GP16|
//!         +----------------+
//! ```
//!
//! * `I2S_FS` goes to J3 Pin 2
//! * `I2S_BCLK` goes to J3 Pin 4
//! * `I2S_ADC` goes to J3 Pin 6
//! * `I2S_DAC` goes to J3 Pin 8
//! * Leave J3 Pin 10 unconnected
//! * `SDA` goes to J4 Pin 6
//! * `SCL` goes to J4 Pin 4
//! * Fit a jumper across J4 Pin 7 and Pin 8, to select I2C mode
//! * Leave J4 Pin 2 unconnected
//!
//! Currently I am clocking out 0xABCD
//!
//! On the scope this looks like:
//!
//! 0b1010_1011_1100_1101 = ABCD
//!
//! But it's changing on rising clock.
//! LRCLK falls on a falling clock edge.
//! We first start outputting the data on the second rising edge after LRCLK.

#![no_std]
#![no_main]

use core::{fmt::Write, num::ParseIntError};

use bsp::entry;
use bsp::hal::{self, clocks::Clock, fugit::RateExtU32, gpio, pac, sio, uart, watchdog};
use rp_pico as bsp;

use defmt::*;
use defmt_rtt as _;
use embedded_hal::{digital::v2::ToggleableOutputPin, serial::Read};
use panic_probe as _;

mod i2s;

static CLIP: [u8; 403052] = *include_bytes!("test audio.raw");

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = watchdog::Watchdog::new(pac.WATCHDOG);
    let sio = sio::Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = hal::clocks::init_clocks_and_plls(
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

    // Use UART for the menu interface
    let tx_pin = pins.gpio0.into_function();
    let rx_pin = pins.gpio1.into_function();
    let uart = uart::UartPeripheral::new(pac.UART0, (tx_pin, rx_pin), &mut pac.RESETS);
    let config = uart::UartConfig::new(
        115200.Hz(),
        uart::DataBits::Eight,
        None,
        uart::StopBits::One,
    );
    let mut uart = uart
        .enable(config, clocks.system_clock.freq())
        .expect("UART config");

    // This is the correct pin on the Raspberry Pico board. On other boards, even if they have an
    // on-board LED, it might need to be changed.
    // Notably, on the Pico W, the LED is not connected to any of the RP2040 GPIOs but to the cyw43 module instead. If you have
    // a Pico W and want to toggle a LED with a simple GPIO output pin, you can connect an external
    // LED to one of the GPIO pins, and reference that pin here.
    let mut led_pin = pins.led.into_push_pull_output();
    let sda_pin = pins.gpio14.into_function();
    let scl_pin = pins.gpio15.into_function();

    let i2c = hal::I2C::i2c1(
        pac.I2C1,
        sda_pin,
        scl_pin,
        100.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    let _i2s_adc_data: gpio::Pin<gpio::bank0::Gpio22, gpio::FunctionPio1, gpio::PullNone> =
        pins.gpio22.reconfigure();
    let mut i2s_dac_data: gpio::Pin<gpio::bank0::Gpio26, gpio::FunctionPio1, gpio::PullNone> =
        pins.gpio26.reconfigure();
    i2s_dac_data.set_drive_strength(gpio::OutputDriveStrength::EightMilliAmps);
    i2s_dac_data.set_slew_rate(gpio::OutputSlewRate::Fast);
    let _i2s_bit_clock: gpio::Pin<gpio::bank0::Gpio27, gpio::FunctionPio1, gpio::PullNone> =
        pins.gpio27.reconfigure();
    let _i2s_lr_clock: gpio::Pin<gpio::bank0::Gpio28, gpio::FunctionPio1, gpio::PullNone> =
        pins.gpio28.reconfigure();

    let mut player = i2s::init(pac.PIO1, &mut pac.RESETS);
    unsafe {
        cortex_m::interrupt::enable();
    }

    let mut codec = nau88c22::Codec::new(i2c);

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

    // Turn on all the power bits, except the sleep bit
    codec
        .modify_powermanagement2(|mut w| {
            w.rhpen_set(true);
            w.lhpen_set(true);
            w.sleep_set(false);
            w.rbsten_set(true);
            w.lbsten_set(true);
            w.rpgaen_set(true);
            w.lpgaen_set(true);
            w.radcen_set(true);
            w.ladcen_set(true);
            w
        })
        .unwrap();
    info!(
        "powermanagement2 = {:?}",
        defmt::Debug2Format(&codec.read_powermanagement2().map_err(clean_error))
    );
    codec
        .modify_powermanagement3(|mut w| {
            w.auxout1en_set(true);
            w.auxout2en_set(true);
            w.lspken_set(true);
            w.rspken_set(true);
            w.rmixen_set(true);
            w.lmixen_set(true);
            w.rdacen_set(true);
            w.ldacen_set(true);
            w
        })
        .unwrap();
    info!(
        "powermanagement2 = {:?}",
        defmt::Debug2Format(&codec.read_powermanagement2().map_err(clean_error))
    );

    // Activate "Boost" mode because VDDSPK = VDDA * 1.5 (we used 5V supply)
    codec
        .modify_outputcontrol(|mut w| {
            w.tsen_set(true);
            w.aux1bst_set(true);
            w.aux2bst_set(true);
            w.spkbst_set(true);
            w
        })
        .unwrap();
    info!(
        "outputcontrol = {:?}",
        defmt::Debug2Format(&codec.read_outputcontrol().map_err(clean_error))
    );

    // Invert right speaker out for Bridge-Tied-Load (i.e. mono) mode
    codec
        .modify_rightspeakersubmix(|mut w| {
            w.rsubbyp_set(true);
            w
        })
        .unwrap();
    info!(
        "rightspeakersubmix = {:?}",
        defmt::Debug2Format(&codec.read_rightspeakersubmix().map_err(clean_error))
    );

    // Set to "clock output" mode
    codec
        .modify_clockcontrol1(|mut w| {
            // MCLK pin is clock source, not PLL
            w.clkm_set(false);
            // divide MCLK by 1 to get Fs = 48 kHz x 256 = 12.288 MHz
            w.mclksel_set(0);
            // lower the BCLK, 2 => BCLK = Fs ÷ 4 => BCLK = 3.072 MHz
            w.bclksel_set(2);
            // Set to be clock output (so called 'Master' mode)
            w.clkioen_set(true);
            w
        })
        .unwrap();
    info!(
        "clockcontrol1 = {:?}",
        defmt::Debug2Format(&codec.read_clockcontrol1().map_err(clean_error))
    );

    // Set to "I2S 16-bit" mode
    codec
        .modify_audiointerface(|mut w| {
            // I2S
            w.aifmt_set(2);
            // 16-bit
            w.wlen_set(0);
            w
        })
        .unwrap();
    info!(
        "audiointerface = {:?}",
        defmt::Debug2Format(&codec.read_audiointerface().map_err(clean_error))
    );

    writeln!(uart, "This is the UART!").unwrap();

    let mut buffer: heapless::Vec<u8, 64> = heapless::Vec::new();
    'outer: loop {
        if uart.uart_is_readable() {
            led_pin.toggle().unwrap();
            let mut ch = [0u8; 1];
            uart.read_full_blocking(&mut ch).unwrap();
            while !uart.uart_is_writable() {
                // spin
            }
            uart.write_raw(&ch).unwrap();
            if ch[0] == b'\r' {
                let Ok(mut iter) = core::str::from_utf8(&buffer).map(|s| s.trim().split(' '))
                else {
                    writeln!(uart, "not UTF8?!").unwrap();
                    buffer.clear();
                    continue 'outer;
                };
                let command = iter.next().unwrap();
                let arg1 = iter.next();
                let arg2 = iter.next();
                writeln!(uart, "Got {:?} {:?} {:?}", command, arg1, arg2).unwrap();
                match (command, arg1, arg2) {
                    ("read", None, None) => {
                        writeln!(
                            uart,
                            "read powermanagement1 = {:?}",
                            codec.read_powermanagement1()
                        )
                        .unwrap();
                        writeln!(
                            uart,
                            "read powermanagement2 = {:?}",
                            codec.read_powermanagement2()
                        )
                        .unwrap();
                        writeln!(
                            uart,
                            "read powermanagement3 = {:?}",
                            codec.read_powermanagement3()
                        )
                        .unwrap();
                        writeln!(
                            uart,
                            "read audiointerface = {:?}",
                            codec.read_audiointerface()
                        )
                        .unwrap();
                        writeln!(uart, "read companding = {:?}", codec.read_companding()).unwrap();
                        writeln!(
                            uart,
                            "read clockcontrol1 = {:?}",
                            codec.read_clockcontrol1()
                        )
                        .unwrap();
                        writeln!(
                            uart,
                            "read clockcontrol2 = {:?}",
                            codec.read_clockcontrol2()
                        )
                        .unwrap();
                        writeln!(uart, "read gpio = {:?}", codec.read_gpio()).unwrap();
                        writeln!(uart, "read jackdetect1 = {:?}", codec.read_jackdetect1())
                            .unwrap();
                        writeln!(uart, "read daccontrol = {:?}", codec.read_daccontrol()).unwrap();
                        writeln!(
                            uart,
                            "read leftdacvolume = {:?}",
                            codec.read_leftdacvolume()
                        )
                        .unwrap();
                        writeln!(
                            uart,
                            "read rightdacvolume = {:?}",
                            codec.read_rightdacvolume()
                        )
                        .unwrap();
                        writeln!(uart, "read jackdetect2 = {:?}", codec.read_jackdetect2())
                            .unwrap();
                        writeln!(uart, "read adccontrol = {:?}", codec.read_adccontrol()).unwrap();
                        writeln!(
                            uart,
                            "read leftadcvolume = {:?}",
                            codec.read_leftadcvolume()
                        )
                        .unwrap();
                        writeln!(
                            uart,
                            "read rightadcvolume = {:?}",
                            codec.read_rightadcvolume()
                        )
                        .unwrap();
                        writeln!(
                            uart,
                            "read eq1highcutoff = {:?}",
                            codec.read_eq1highcutoff()
                        )
                        .unwrap();
                        writeln!(uart, "read eq2peak1 = {:?}", codec.read_eq2peak1()).unwrap();
                        writeln!(uart, "read eq3peak2 = {:?}", codec.read_eq3peak2()).unwrap();
                        writeln!(uart, "read eq4peak3 = {:?}", codec.read_eq4peak3()).unwrap();
                        writeln!(uart, "read eq5lowcutoff = {:?}", codec.read_eq5lowcutoff())
                            .unwrap();
                        writeln!(uart, "read daclimiter1 = {:?}", codec.read_daclimiter1())
                            .unwrap();
                        writeln!(uart, "read daclimiter2 = {:?}", codec.read_daclimiter2())
                            .unwrap();
                        writeln!(uart, "read notchfilter1 = {:?}", codec.read_notchfilter1())
                            .unwrap();
                        writeln!(uart, "read notchfilter2 = {:?}", codec.read_notchfilter2())
                            .unwrap();
                        writeln!(uart, "read notchfilter3 = {:?}", codec.read_notchfilter3())
                            .unwrap();
                        writeln!(uart, "read notchfilter4 = {:?}", codec.read_notchfilter4())
                            .unwrap();
                        writeln!(uart, "read alccontrol1 = {:?}", codec.read_alccontrol1())
                            .unwrap();
                        writeln!(uart, "read alccontrol2 = {:?}", codec.read_alccontrol2())
                            .unwrap();
                        writeln!(uart, "read alccontrol3 = {:?}", codec.read_alccontrol3())
                            .unwrap();
                        writeln!(uart, "read noisegate = {:?}", codec.read_noisegate()).unwrap();
                        writeln!(uart, "read plln = {:?}", codec.read_plln()).unwrap();
                        writeln!(uart, "read pllk1 = {:?}", codec.read_pllk1()).unwrap();
                        writeln!(uart, "read pllk2 = {:?}", codec.read_pllk2()).unwrap();
                        writeln!(uart, "read pllk3 = {:?}", codec.read_pllk3()).unwrap();
                        writeln!(
                            uart,
                            "read threedcontrol = {:?}",
                            codec.read_threedcontrol()
                        )
                        .unwrap();
                        writeln!(
                            uart,
                            "read rightspeakersubmix = {:?}",
                            codec.read_rightspeakersubmix()
                        )
                        .unwrap();
                        writeln!(uart, "read inputcontrol = {:?}", codec.read_inputcontrol())
                            .unwrap();
                        writeln!(
                            uart,
                            "read leftinputpgagain = {:?}",
                            codec.read_leftinputpgagain()
                        )
                        .unwrap();
                        writeln!(
                            uart,
                            "read rightinputpgagain = {:?}",
                            codec.read_rightinputpgagain()
                        )
                        .unwrap();
                        writeln!(uart, "read leftadcboost = {:?}", codec.read_leftadcboost())
                            .unwrap();
                        writeln!(
                            uart,
                            "read rightadcboost = {:?}",
                            codec.read_rightadcboost()
                        )
                        .unwrap();
                        writeln!(
                            uart,
                            "read outputcontrol = {:?}",
                            codec.read_outputcontrol()
                        )
                        .unwrap();
                        writeln!(uart, "read leftmixer = {:?}", codec.read_leftmixer()).unwrap();
                        writeln!(uart, "read rightmixer = {:?}", codec.read_rightmixer()).unwrap();
                        writeln!(uart, "read lhpvolume = {:?}", codec.read_lhpvolume()).unwrap();
                        writeln!(uart, "read rhpvolume = {:?}", codec.read_rhpvolume()).unwrap();
                        writeln!(
                            uart,
                            "read lspkoutvolume = {:?}",
                            codec.read_lspkoutvolume()
                        )
                        .unwrap();
                        writeln!(
                            uart,
                            "read rspkoutvolume = {:?}",
                            codec.read_rspkoutvolume()
                        )
                        .unwrap();
                        writeln!(uart, "read aux2mixer = {:?}", codec.read_aux2mixer()).unwrap();
                        writeln!(uart, "read aux1mixer = {:?}", codec.read_aux1mixer()).unwrap();
                        writeln!(
                            uart,
                            "read powermanagement = {:?}",
                            codec.read_powermanagement()
                        )
                        .unwrap();
                        writeln!(uart, "read lefttimeslot = {:?}", codec.read_lefttimeslot())
                            .unwrap();
                        writeln!(uart, "read misc = {:?}", codec.read_misc()).unwrap();
                        writeln!(
                            uart,
                            "read righttimeslot = {:?}",
                            codec.read_righttimeslot()
                        )
                        .unwrap();
                        writeln!(
                            uart,
                            "read devicerevisionno = {:?}",
                            codec.read_devicerevisionno()
                        )
                        .unwrap();
                        writeln!(uart, "read deviceid = {:?}", codec.read_deviceid()).unwrap();
                        writeln!(uart, "read dacdither = {:?}", codec.read_dacdither()).unwrap();
                        writeln!(
                            uart,
                            "read alcenhancements1 = {:?}",
                            codec.read_alcenhancements1()
                        )
                        .unwrap();
                        writeln!(
                            uart,
                            "read alcenhancements2 = {:?}",
                            codec.read_alcenhancements2()
                        )
                        .unwrap();
                        writeln!(uart, "read misccontrols = {:?}", codec.read_misccontrols())
                            .unwrap();
                        writeln!(
                            uart,
                            "read tieoffoverrides = {:?}",
                            codec.read_tieoffoverrides()
                        )
                        .unwrap();
                        writeln!(
                            uart,
                            "read powertieoffctrl = {:?}",
                            codec.read_powertieoffctrl()
                        )
                        .unwrap();
                        writeln!(
                            uart,
                            "read p2pdetectorread = {:?}",
                            codec.read_p2pdetectorread()
                        )
                        .unwrap();
                        writeln!(
                            uart,
                            "read peakdetectorread = {:?}",
                            codec.read_peakdetectorread()
                        )
                        .unwrap();
                        writeln!(
                            uart,
                            "read controlandstatus = {:?}",
                            codec.read_controlandstatus()
                        )
                        .unwrap();
                        writeln!(
                            uart,
                            "read outputtieoffcontrol = {:?}",
                            codec.read_outputtieoffcontrol()
                        )
                        .unwrap();
                    }
                    ("read", Some("powermanagement1"), None) => {
                        writeln!(
                            uart,
                            "read powermanagement1 = {:?}",
                            codec.read_powermanagement1()
                        )
                        .unwrap();
                    }
                    ("read", Some("powermanagement2"), None) => {
                        writeln!(
                            uart,
                            "read powermanagement2 = {:?}",
                            codec.read_powermanagement2()
                        )
                        .unwrap();
                    }
                    ("read", Some("powermanagement3"), None) => {
                        writeln!(
                            uart,
                            "read powermanagement3 = {:?}",
                            codec.read_powermanagement3()
                        )
                        .unwrap();
                    }
                    ("read", Some("audiointerface"), None) => {
                        writeln!(
                            uart,
                            "read audiointerface = {:?}",
                            codec.read_audiointerface()
                        )
                        .unwrap();
                    }
                    ("read", Some("companding"), None) => {
                        writeln!(uart, "read companding = {:?}", codec.read_companding()).unwrap();
                    }
                    ("read", Some("clockcontrol1"), None) => {
                        writeln!(
                            uart,
                            "read clockcontrol1 = {:?}",
                            codec.read_clockcontrol1()
                        )
                        .unwrap();
                    }
                    ("read", Some("clockcontrol2"), None) => {
                        writeln!(
                            uart,
                            "read clockcontrol2 = {:?}",
                            codec.read_clockcontrol2()
                        )
                        .unwrap();
                    }
                    ("read", Some("gpio"), None) => {
                        writeln!(uart, "read gpio = {:?}", codec.read_gpio()).unwrap();
                    }
                    ("read", Some("jackdetect1"), None) => {
                        writeln!(uart, "read jackdetect1 = {:?}", codec.read_jackdetect1())
                            .unwrap();
                    }
                    ("read", Some("daccontrol"), None) => {
                        writeln!(uart, "read daccontrol = {:?}", codec.read_daccontrol()).unwrap();
                    }
                    ("read", Some("leftdacvolume"), None) => {
                        writeln!(
                            uart,
                            "read leftdacvolume = {:?}",
                            codec.read_leftdacvolume()
                        )
                        .unwrap();
                    }
                    ("read", Some("rightdacvolume"), None) => {
                        writeln!(
                            uart,
                            "read rightdacvolume = {:?}",
                            codec.read_rightdacvolume()
                        )
                        .unwrap();
                    }
                    ("read", Some("jackdetect2"), None) => {
                        writeln!(uart, "read jackdetect2 = {:?}", codec.read_jackdetect2())
                            .unwrap();
                    }
                    ("read", Some("adccontrol"), None) => {
                        writeln!(uart, "read adccontrol = {:?}", codec.read_adccontrol()).unwrap();
                    }
                    ("read", Some("leftadcvolume"), None) => {
                        writeln!(
                            uart,
                            "read leftadcvolume = {:?}",
                            codec.read_leftadcvolume()
                        )
                        .unwrap();
                    }
                    ("read", Some("rightadcvolume"), None) => {
                        writeln!(
                            uart,
                            "read rightadcvolume = {:?}",
                            codec.read_rightadcvolume()
                        )
                        .unwrap();
                    }
                    ("read", Some("eq1highcutoff"), None) => {
                        writeln!(
                            uart,
                            "read eq1highcutoff = {:?}",
                            codec.read_eq1highcutoff()
                        )
                        .unwrap();
                    }
                    ("read", Some("eq2peak1"), None) => {
                        writeln!(uart, "read eq2peak1 = {:?}", codec.read_eq2peak1()).unwrap();
                    }
                    ("read", Some("eq3peak2"), None) => {
                        writeln!(uart, "read eq3peak2 = {:?}", codec.read_eq3peak2()).unwrap();
                    }
                    ("read", Some("eq4peak3"), None) => {
                        writeln!(uart, "read eq4peak3 = {:?}", codec.read_eq4peak3()).unwrap();
                    }
                    ("read", Some("eq5lowcutoff"), None) => {
                        writeln!(uart, "read eq5lowcutoff = {:?}", codec.read_eq5lowcutoff())
                            .unwrap();
                    }
                    ("read", Some("daclimiter1"), None) => {
                        writeln!(uart, "read daclimiter1 = {:?}", codec.read_daclimiter1())
                            .unwrap();
                    }
                    ("read", Some("daclimiter2"), None) => {
                        writeln!(uart, "read daclimiter2 = {:?}", codec.read_daclimiter2())
                            .unwrap();
                    }
                    ("read", Some("notchfilter1"), None) => {
                        writeln!(uart, "read notchfilter1 = {:?}", codec.read_notchfilter1())
                            .unwrap();
                    }
                    ("read", Some("notchfilter2"), None) => {
                        writeln!(uart, "read notchfilter2 = {:?}", codec.read_notchfilter2())
                            .unwrap();
                    }
                    ("read", Some("notchfilter3"), None) => {
                        writeln!(uart, "read notchfilter3 = {:?}", codec.read_notchfilter3())
                            .unwrap();
                    }
                    ("read", Some("notchfilter4"), None) => {
                        writeln!(uart, "read notchfilter4 = {:?}", codec.read_notchfilter4())
                            .unwrap();
                    }
                    ("read", Some("alccontrol1"), None) => {
                        writeln!(uart, "read alccontrol1 = {:?}", codec.read_alccontrol1())
                            .unwrap();
                    }
                    ("read", Some("alccontrol2"), None) => {
                        writeln!(uart, "read alccontrol2 = {:?}", codec.read_alccontrol2())
                            .unwrap();
                    }
                    ("read", Some("alccontrol3"), None) => {
                        writeln!(uart, "read alccontrol3 = {:?}", codec.read_alccontrol3())
                            .unwrap();
                    }
                    ("read", Some("noisegate"), None) => {
                        writeln!(uart, "read noisegate = {:?}", codec.read_noisegate()).unwrap();
                    }
                    ("read", Some("plln"), None) => {
                        writeln!(uart, "read plln = {:?}", codec.read_plln()).unwrap();
                    }
                    ("read", Some("pllk1"), None) => {
                        writeln!(uart, "read pllk1 = {:?}", codec.read_pllk1()).unwrap();
                    }
                    ("read", Some("pllk2"), None) => {
                        writeln!(uart, "read pllk2 = {:?}", codec.read_pllk2()).unwrap();
                    }
                    ("read", Some("pllk3"), None) => {
                        writeln!(uart, "read pllk3 = {:?}", codec.read_pllk3()).unwrap();
                    }
                    ("read", Some("threedcontrol"), None) => {
                        writeln!(
                            uart,
                            "read threedcontrol = {:?}",
                            codec.read_threedcontrol()
                        )
                        .unwrap();
                    }
                    ("read", Some("rightspeakersubmix"), None) => {
                        writeln!(
                            uart,
                            "read rightspeakersubmix = {:?}",
                            codec.read_rightspeakersubmix()
                        )
                        .unwrap();
                    }
                    ("read", Some("inputcontrol"), None) => {
                        writeln!(uart, "read inputcontrol = {:?}", codec.read_inputcontrol())
                            .unwrap();
                    }
                    ("read", Some("leftinputpgagain"), None) => {
                        writeln!(
                            uart,
                            "read leftinputpgagain = {:?}",
                            codec.read_leftinputpgagain()
                        )
                        .unwrap();
                    }
                    ("read", Some("rightinputpgagain"), None) => {
                        writeln!(
                            uart,
                            "read rightinputpgagain = {:?}",
                            codec.read_rightinputpgagain()
                        )
                        .unwrap();
                    }
                    ("read", Some("leftadcboost"), None) => {
                        writeln!(uart, "read leftadcboost = {:?}", codec.read_leftadcboost())
                            .unwrap();
                    }
                    ("read", Some("rightadcboost"), None) => {
                        writeln!(
                            uart,
                            "read rightadcboost = {:?}",
                            codec.read_rightadcboost()
                        )
                        .unwrap();
                    }
                    ("read", Some("outputcontrol"), None) => {
                        writeln!(
                            uart,
                            "read outputcontrol = {:?}",
                            codec.read_outputcontrol()
                        )
                        .unwrap();
                    }
                    ("read", Some("leftmixer"), None) => {
                        writeln!(uart, "read leftmixer = {:?}", codec.read_leftmixer()).unwrap();
                    }
                    ("read", Some("rightmixer"), None) => {
                        writeln!(uart, "read rightmixer = {:?}", codec.read_rightmixer()).unwrap();
                    }
                    ("read", Some("lhpvolume"), None) => {
                        writeln!(uart, "read lhpvolume = {:?}", codec.read_lhpvolume()).unwrap();
                    }
                    ("read", Some("rhpvolume"), None) => {
                        writeln!(uart, "read rhpvolume = {:?}", codec.read_rhpvolume()).unwrap();
                    }
                    ("read", Some("lspkoutvolume"), None) => {
                        writeln!(
                            uart,
                            "read lspkoutvolume = {:?}",
                            codec.read_lspkoutvolume()
                        )
                        .unwrap();
                    }
                    ("read", Some("rspkoutvolume"), None) => {
                        writeln!(
                            uart,
                            "read rspkoutvolume = {:?}",
                            codec.read_rspkoutvolume()
                        )
                        .unwrap();
                    }
                    ("read", Some("aux2mixer"), None) => {
                        writeln!(uart, "read aux2mixer = {:?}", codec.read_aux2mixer()).unwrap();
                    }
                    ("read", Some("aux1mixer"), None) => {
                        writeln!(uart, "read aux1mixer = {:?}", codec.read_aux1mixer()).unwrap();
                    }
                    ("read", Some("powermanagement"), None) => {
                        writeln!(
                            uart,
                            "read powermanagement = {:?}",
                            codec.read_powermanagement()
                        )
                        .unwrap();
                    }
                    ("read", Some("lefttimeslot"), None) => {
                        writeln!(uart, "read lefttimeslot = {:?}", codec.read_lefttimeslot())
                            .unwrap();
                    }
                    ("read", Some("misc"), None) => {
                        writeln!(uart, "read misc = {:?}", codec.read_misc()).unwrap();
                    }
                    ("read", Some("righttimeslot"), None) => {
                        writeln!(
                            uart,
                            "read righttimeslot = {:?}",
                            codec.read_righttimeslot()
                        )
                        .unwrap();
                    }
                    ("read", Some("devicerevisionno"), None) => {
                        writeln!(
                            uart,
                            "read devicerevisionno = {:?}",
                            codec.read_devicerevisionno()
                        )
                        .unwrap();
                    }
                    ("read", Some("deviceid"), None) => {
                        writeln!(uart, "read deviceid = {:?}", codec.read_deviceid()).unwrap();
                    }
                    ("read", Some("dacdither"), None) => {
                        writeln!(uart, "read dacdither = {:?}", codec.read_dacdither()).unwrap();
                    }
                    ("read", Some("alcenhancements1"), None) => {
                        writeln!(
                            uart,
                            "read alcenhancements1 = {:?}",
                            codec.read_alcenhancements1()
                        )
                        .unwrap();
                    }
                    ("read", Some("alcenhancements2"), None) => {
                        writeln!(
                            uart,
                            "read alcenhancements2 = {:?}",
                            codec.read_alcenhancements2()
                        )
                        .unwrap();
                    }
                    ("read", Some("misccontrols"), None) => {
                        writeln!(uart, "read misccontrols = {:?}", codec.read_misccontrols())
                            .unwrap();
                    }
                    ("read", Some("tieoffoverrides"), None) => {
                        writeln!(
                            uart,
                            "read tieoffoverrides = {:?}",
                            codec.read_tieoffoverrides()
                        )
                        .unwrap();
                    }
                    ("read", Some("powertieoffctrl"), None) => {
                        writeln!(
                            uart,
                            "read powertieoffctrl = {:?}",
                            codec.read_powertieoffctrl()
                        )
                        .unwrap();
                    }
                    ("read", Some("p2pdetectorread"), None) => {
                        writeln!(
                            uart,
                            "read p2pdetectorread = {:?}",
                            codec.read_p2pdetectorread()
                        )
                        .unwrap();
                    }
                    ("read", Some("peakdetectorread"), None) => {
                        writeln!(
                            uart,
                            "read peakdetectorread = {:?}",
                            codec.read_peakdetectorread()
                        )
                        .unwrap();
                    }
                    ("read", Some("controlandstatus"), None) => {
                        writeln!(
                            uart,
                            "read controlandstatus = {:?}",
                            codec.read_controlandstatus()
                        )
                        .unwrap();
                    }
                    ("read", Some("outputtieoffcontrol"), None) => {
                        writeln!(
                            uart,
                            "read outputtieoffcontrol = {:?}",
                            codec.read_outputtieoffcontrol()
                        )
                        .unwrap();
                    }

                    ("write", Some("powermanagement1"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write powermanagement1 = 0x{:03x}", value).unwrap();
                            codec
                                .modify_powermanagement1(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(
                                uart,
                                "write powermanagement1 = {:?}",
                                codec.read_powermanagement1()
                            )
                            .unwrap();
                        }
                    }
                    ("write", Some("powermanagement2"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write powermanagement2 = 0x{:03x}", value).unwrap();
                            codec
                                .modify_powermanagement2(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(
                                uart,
                                "write powermanagement2 = {:?}",
                                codec.read_powermanagement2()
                            )
                            .unwrap();
                        }
                    }
                    ("write", Some("powermanagement3"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write powermanagement3 = 0x{:03x}", value).unwrap();
                            codec
                                .modify_powermanagement3(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(
                                uart,
                                "write powermanagement3 = {:?}",
                                codec.read_powermanagement3()
                            )
                            .unwrap();
                        }
                    }
                    ("write", Some("audiointerface"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write audiointerface = 0x{:03x}", value).unwrap();
                            codec
                                .modify_audiointerface(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(
                                uart,
                                "write audiointerface = {:?}",
                                codec.read_audiointerface()
                            )
                            .unwrap();
                        }
                    }
                    ("write", Some("companding"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write companding = 0x{:03x}", value).unwrap();
                            codec
                                .modify_companding(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(uart, "write companding = {:?}", codec.read_companding())
                                .unwrap();
                        }
                    }
                    ("write", Some("clockcontrol1"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write clockcontrol1 = 0x{:03x}", value).unwrap();
                            codec
                                .modify_clockcontrol1(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(
                                uart,
                                "write clockcontrol1 = {:?}",
                                codec.read_clockcontrol1()
                            )
                            .unwrap();
                        }
                    }
                    ("write", Some("clockcontrol2"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write clockcontrol2 = 0x{:03x}", value).unwrap();
                            codec
                                .modify_clockcontrol2(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(
                                uart,
                                "write clockcontrol2 = {:?}",
                                codec.read_clockcontrol2()
                            )
                            .unwrap();
                        }
                    }
                    ("write", Some("gpio"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write gpio = 0x{:03x}", value).unwrap();
                            codec
                                .modify_gpio(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(uart, "write gpio = {:?}", codec.read_gpio()).unwrap();
                        }
                    }
                    ("write", Some("jackdetect1"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write jackdetect1 = 0x{:03x}", value).unwrap();
                            codec
                                .modify_jackdetect1(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(uart, "write jackdetect1 = {:?}", codec.read_jackdetect1())
                                .unwrap();
                        }
                    }
                    ("write", Some("daccontrol"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write daccontrol = 0x{:03x}", value).unwrap();
                            codec
                                .modify_daccontrol(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(uart, "write daccontrol = {:?}", codec.read_daccontrol())
                                .unwrap();
                        }
                    }
                    ("write", Some("leftdacvolume"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write leftdacvolume = 0x{:03x}", value).unwrap();
                            codec
                                .modify_leftdacvolume(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(
                                uart,
                                "write leftdacvolume = {:?}",
                                codec.read_leftdacvolume()
                            )
                            .unwrap();
                        }
                    }
                    ("write", Some("rightdacvolume"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write rightdacvolume = 0x{:03x}", value).unwrap();
                            codec
                                .modify_rightdacvolume(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(
                                uart,
                                "write rightdacvolume = {:?}",
                                codec.read_rightdacvolume()
                            )
                            .unwrap();
                        }
                    }
                    ("write", Some("jackdetect2"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write jackdetect2 = 0x{:03x}", value).unwrap();
                            codec
                                .modify_jackdetect2(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(uart, "write jackdetect2 = {:?}", codec.read_jackdetect2())
                                .unwrap();
                        }
                    }
                    ("write", Some("adccontrol"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write adccontrol = 0x{:03x}", value).unwrap();
                            codec
                                .modify_adccontrol(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(uart, "write adccontrol = {:?}", codec.read_adccontrol())
                                .unwrap();
                        }
                    }
                    ("write", Some("leftadcvolume"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write leftadcvolume = 0x{:03x}", value).unwrap();
                            codec
                                .modify_leftadcvolume(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(
                                uart,
                                "write leftadcvolume = {:?}",
                                codec.read_leftadcvolume()
                            )
                            .unwrap();
                        }
                    }
                    ("write", Some("rightadcvolume"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write rightadcvolume = 0x{:03x}", value).unwrap();
                            codec
                                .modify_rightadcvolume(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(
                                uart,
                                "write rightadcvolume = {:?}",
                                codec.read_rightadcvolume()
                            )
                            .unwrap();
                        }
                    }
                    ("write", Some("eq1highcutoff"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write eq1highcutoff = 0x{:03x}", value).unwrap();
                            codec
                                .modify_eq1highcutoff(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(
                                uart,
                                "write eq1highcutoff = {:?}",
                                codec.read_eq1highcutoff()
                            )
                            .unwrap();
                        }
                    }
                    ("write", Some("eq2peak1"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write eq2peak1 = 0x{:03x}", value).unwrap();
                            codec
                                .modify_eq2peak1(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(uart, "write eq2peak1 = {:?}", codec.read_eq2peak1()).unwrap();
                        }
                    }
                    ("write", Some("eq3peak2"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write eq3peak2 = 0x{:03x}", value).unwrap();
                            codec
                                .modify_eq3peak2(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(uart, "write eq3peak2 = {:?}", codec.read_eq3peak2()).unwrap();
                        }
                    }
                    ("write", Some("eq4peak3"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write eq4peak3 = 0x{:03x}", value).unwrap();
                            codec
                                .modify_eq4peak3(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(uart, "write eq4peak3 = {:?}", codec.read_eq4peak3()).unwrap();
                        }
                    }
                    ("write", Some("eq5lowcutoff"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write eq5lowcutoff = 0x{:03x}", value).unwrap();
                            codec
                                .modify_eq5lowcutoff(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(uart, "write eq5lowcutoff = {:?}", codec.read_eq5lowcutoff())
                                .unwrap();
                        }
                    }
                    ("write", Some("daclimiter1"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write daclimiter1 = 0x{:03x}", value).unwrap();
                            codec
                                .modify_daclimiter1(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(uart, "write daclimiter1 = {:?}", codec.read_daclimiter1())
                                .unwrap();
                        }
                    }
                    ("write", Some("daclimiter2"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write daclimiter2 = 0x{:03x}", value).unwrap();
                            codec
                                .modify_daclimiter2(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(uart, "write daclimiter2 = {:?}", codec.read_daclimiter2())
                                .unwrap();
                        }
                    }
                    ("write", Some("notchfilter1"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write notchfilter1 = 0x{:03x}", value).unwrap();
                            codec
                                .modify_notchfilter1(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(uart, "write notchfilter1 = {:?}", codec.read_notchfilter1())
                                .unwrap();
                        }
                    }
                    ("write", Some("notchfilter2"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write notchfilter2 = 0x{:03x}", value).unwrap();
                            codec
                                .modify_notchfilter2(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(uart, "write notchfilter2 = {:?}", codec.read_notchfilter2())
                                .unwrap();
                        }
                    }
                    ("write", Some("notchfilter3"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write notchfilter3 = 0x{:03x}", value).unwrap();
                            codec
                                .modify_notchfilter3(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(uart, "write notchfilter3 = {:?}", codec.read_notchfilter3())
                                .unwrap();
                        }
                    }
                    ("write", Some("notchfilter4"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write notchfilter4 = 0x{:03x}", value).unwrap();
                            codec
                                .modify_notchfilter4(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(uart, "write notchfilter4 = {:?}", codec.read_notchfilter4())
                                .unwrap();
                        }
                    }
                    ("write", Some("alccontrol1"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write alccontrol1 = 0x{:03x}", value).unwrap();
                            codec
                                .modify_alccontrol1(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(uart, "write alccontrol1 = {:?}", codec.read_alccontrol1())
                                .unwrap();
                        }
                    }
                    ("write", Some("alccontrol2"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write alccontrol2 = 0x{:03x}", value).unwrap();
                            codec
                                .modify_alccontrol2(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(uart, "write alccontrol2 = {:?}", codec.read_alccontrol2())
                                .unwrap();
                        }
                    }
                    ("write", Some("alccontrol3"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write alccontrol3 = 0x{:03x}", value).unwrap();
                            codec
                                .modify_alccontrol3(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(uart, "write alccontrol3 = {:?}", codec.read_alccontrol3())
                                .unwrap();
                        }
                    }
                    ("write", Some("noisegate"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write noisegate = 0x{:03x}", value).unwrap();
                            codec
                                .modify_noisegate(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(uart, "write noisegate = {:?}", codec.read_noisegate())
                                .unwrap();
                        }
                    }
                    ("write", Some("plln"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write plln = 0x{:03x}", value).unwrap();
                            codec
                                .modify_plln(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(uart, "write plln = {:?}", codec.read_plln()).unwrap();
                        }
                    }
                    ("write", Some("pllk1"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write pllk1 = 0x{:03x}", value).unwrap();
                            codec
                                .modify_pllk1(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(uart, "write pllk1 = {:?}", codec.read_pllk1()).unwrap();
                        }
                    }
                    ("write", Some("pllk2"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write pllk2 = 0x{:03x}", value).unwrap();
                            codec
                                .modify_pllk2(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(uart, "write pllk2 = {:?}", codec.read_pllk2()).unwrap();
                        }
                    }
                    ("write", Some("pllk3"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write pllk3 = 0x{:03x}", value).unwrap();
                            codec
                                .modify_pllk3(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(uart, "write pllk3 = {:?}", codec.read_pllk3()).unwrap();
                        }
                    }
                    ("write", Some("threedcontrol"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write threedcontrol = 0x{:03x}", value).unwrap();
                            codec
                                .modify_threedcontrol(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(
                                uart,
                                "write threedcontrol = {:?}",
                                codec.read_threedcontrol()
                            )
                            .unwrap();
                        }
                    }
                    ("write", Some("rightspeakersubmix"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write rightspeakersubmix = 0x{:03x}", value).unwrap();
                            codec
                                .modify_rightspeakersubmix(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(
                                uart,
                                "write rightspeakersubmix = {:?}",
                                codec.read_rightspeakersubmix()
                            )
                            .unwrap();
                        }
                    }
                    ("write", Some("inputcontrol"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write inputcontrol = 0x{:03x}", value).unwrap();
                            codec
                                .modify_inputcontrol(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(uart, "write inputcontrol = {:?}", codec.read_inputcontrol())
                                .unwrap();
                        }
                    }
                    ("write", Some("leftinputpgagain"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write leftinputpgagain = 0x{:03x}", value).unwrap();
                            codec
                                .modify_leftinputpgagain(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(
                                uart,
                                "write leftinputpgagain = {:?}",
                                codec.read_leftinputpgagain()
                            )
                            .unwrap();
                        }
                    }
                    ("write", Some("rightinputpgagain"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write rightinputpgagain = 0x{:03x}", value).unwrap();
                            codec
                                .modify_rightinputpgagain(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(
                                uart,
                                "write rightinputpgagain = {:?}",
                                codec.read_rightinputpgagain()
                            )
                            .unwrap();
                        }
                    }
                    ("write", Some("leftadcboost"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write leftadcboost = 0x{:03x}", value).unwrap();
                            codec
                                .modify_leftadcboost(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(uart, "write leftadcboost = {:?}", codec.read_leftadcboost())
                                .unwrap();
                        }
                    }
                    ("write", Some("rightadcboost"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write rightadcboost = 0x{:03x}", value).unwrap();
                            codec
                                .modify_rightadcboost(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(
                                uart,
                                "write rightadcboost = {:?}",
                                codec.read_rightadcboost()
                            )
                            .unwrap();
                        }
                    }
                    ("write", Some("outputcontrol"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write outputcontrol = 0x{:03x}", value).unwrap();
                            codec
                                .modify_outputcontrol(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(
                                uart,
                                "write outputcontrol = {:?}",
                                codec.read_outputcontrol()
                            )
                            .unwrap();
                        }
                    }
                    ("write", Some("leftmixer"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write leftmixer = 0x{:03x}", value).unwrap();
                            codec
                                .modify_leftmixer(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(uart, "write leftmixer = {:?}", codec.read_leftmixer())
                                .unwrap();
                        }
                    }
                    ("write", Some("rightmixer"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write rightmixer = 0x{:03x}", value).unwrap();
                            codec
                                .modify_rightmixer(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(uart, "write rightmixer = {:?}", codec.read_rightmixer())
                                .unwrap();
                        }
                    }
                    ("write", Some("lhpvolume"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write lhpvolume = 0x{:03x}", value).unwrap();
                            codec
                                .modify_lhpvolume(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(uart, "write lhpvolume = {:?}", codec.read_lhpvolume())
                                .unwrap();
                        }
                    }
                    ("write", Some("rhpvolume"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write rhpvolume = 0x{:03x}", value).unwrap();
                            codec
                                .modify_rhpvolume(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(uart, "write rhpvolume = {:?}", codec.read_rhpvolume())
                                .unwrap();
                        }
                    }
                    ("write", Some("lspkoutvolume"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write lspkoutvolume = 0x{:03x}", value).unwrap();
                            codec
                                .modify_lspkoutvolume(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(
                                uart,
                                "write lspkoutvolume = {:?}",
                                codec.read_lspkoutvolume()
                            )
                            .unwrap();
                        }
                    }
                    ("write", Some("rspkoutvolume"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write rspkoutvolume = 0x{:03x}", value).unwrap();
                            codec
                                .modify_rspkoutvolume(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(
                                uart,
                                "write rspkoutvolume = {:?}",
                                codec.read_rspkoutvolume()
                            )
                            .unwrap();
                        }
                    }
                    ("write", Some("aux2mixer"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write aux2mixer = 0x{:03x}", value).unwrap();
                            codec
                                .modify_aux2mixer(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(uart, "write aux2mixer = {:?}", codec.read_aux2mixer())
                                .unwrap();
                        }
                    }
                    ("write", Some("aux1mixer"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write aux1mixer = 0x{:03x}", value).unwrap();
                            codec
                                .modify_aux1mixer(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(uart, "write aux1mixer = {:?}", codec.read_aux1mixer())
                                .unwrap();
                        }
                    }
                    ("write", Some("powermanagement"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write powermanagement = 0x{:03x}", value).unwrap();
                            codec
                                .modify_powermanagement(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(
                                uart,
                                "write powermanagement = {:?}",
                                codec.read_powermanagement()
                            )
                            .unwrap();
                        }
                    }
                    ("write", Some("lefttimeslot"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write lefttimeslot = 0x{:03x}", value).unwrap();
                            codec
                                .modify_lefttimeslot(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(uart, "write lefttimeslot = {:?}", codec.read_lefttimeslot())
                                .unwrap();
                        }
                    }
                    ("write", Some("misc"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write misc = 0x{:03x}", value).unwrap();
                            codec
                                .modify_misc(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(uart, "write misc = {:?}", codec.read_misc()).unwrap();
                        }
                    }
                    ("write", Some("righttimeslot"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write righttimeslot = 0x{:03x}", value).unwrap();
                            codec
                                .modify_righttimeslot(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(
                                uart,
                                "write righttimeslot = {:?}",
                                codec.read_righttimeslot()
                            )
                            .unwrap();
                        }
                    }
                    ("write", Some("devicerevisionno"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write devicerevisionno = 0x{:03x}", value).unwrap();
                            codec
                                .modify_devicerevisionno(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(
                                uart,
                                "write devicerevisionno = {:?}",
                                codec.read_devicerevisionno()
                            )
                            .unwrap();
                        }
                    }
                    ("write", Some("deviceid"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write deviceid = 0x{:03x}", value).unwrap();
                            codec
                                .modify_deviceid(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(uart, "write deviceid = {:?}", codec.read_deviceid()).unwrap();
                        }
                    }
                    ("write", Some("dacdither"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write dacdither = 0x{:03x}", value).unwrap();
                            codec
                                .modify_dacdither(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(uart, "write dacdither = {:?}", codec.read_dacdither())
                                .unwrap();
                        }
                    }
                    ("write", Some("alcenhancements1"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write alcenhancements1 = 0x{:03x}", value).unwrap();
                            codec
                                .modify_alcenhancements1(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(
                                uart,
                                "write alcenhancements1 = {:?}",
                                codec.read_alcenhancements1()
                            )
                            .unwrap();
                        }
                    }
                    ("write", Some("alcenhancements2"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write alcenhancements2 = 0x{:03x}", value).unwrap();
                            codec
                                .modify_alcenhancements2(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(
                                uart,
                                "write alcenhancements2 = {:?}",
                                codec.read_alcenhancements2()
                            )
                            .unwrap();
                        }
                    }
                    ("write", Some("misccontrols"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write misccontrols = 0x{:03x}", value).unwrap();
                            codec
                                .modify_misccontrols(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(uart, "write misccontrols = {:?}", codec.read_misccontrols())
                                .unwrap();
                        }
                    }
                    ("write", Some("tieoffoverrides"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write tieoffoverrides = 0x{:03x}", value).unwrap();
                            codec
                                .modify_tieoffoverrides(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(
                                uart,
                                "write tieoffoverrides = {:?}",
                                codec.read_tieoffoverrides()
                            )
                            .unwrap();
                        }
                    }
                    ("write", Some("powertieoffctrl"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write powertieoffctrl = 0x{:03x}", value).unwrap();
                            codec
                                .modify_powertieoffctrl(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(
                                uart,
                                "write powertieoffctrl = {:?}",
                                codec.read_powertieoffctrl()
                            )
                            .unwrap();
                        }
                    }
                    ("write", Some("p2pdetectorread"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write p2pdetectorread = 0x{:03x}", value).unwrap();
                            codec
                                .modify_p2pdetectorread(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(
                                uart,
                                "write p2pdetectorread = {:?}",
                                codec.read_p2pdetectorread()
                            )
                            .unwrap();
                        }
                    }
                    ("write", Some("peakdetectorread"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write peakdetectorread = 0x{:03x}", value).unwrap();
                            codec
                                .modify_peakdetectorread(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(
                                uart,
                                "write peakdetectorread = {:?}",
                                codec.read_peakdetectorread()
                            )
                            .unwrap();
                        }
                    }
                    ("write", Some("controlandstatus"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write controlandstatus = 0x{:03x}", value).unwrap();
                            codec
                                .modify_controlandstatus(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(
                                uart,
                                "write controlandstatus = {:?}",
                                codec.read_controlandstatus()
                            )
                            .unwrap();
                        }
                    }
                    ("write", Some("outputtieoffcontrol"), Some(value)) => {
                        if let Ok(value) = parse_integer(value) {
                            writeln!(uart, "write outputtieoffcontrol = 0x{:03x}", value).unwrap();
                            codec
                                .modify_outputtieoffcontrol(|mut w| {
                                    w.0 = value;
                                    w
                                })
                                .unwrap();
                            writeln!(
                                uart,
                                "write outputtieoffcontrol = {:?}",
                                codec.read_outputtieoffcontrol()
                            )
                            .unwrap();
                        }
                    }

                    ("play", None, None) => {
                        writeln!(uart, "Looping audio sample...").unwrap();
                        let mut sample_buffer = &CLIP[..];
                        loop {
                            let read = player.play_samples_16bit_stereo(sample_buffer);
                            sample_buffer = &sample_buffer[read..];
                            if sample_buffer.is_empty() {
                                sample_buffer = &CLIP[..];
                            }
                            if let nb::Result::Ok(0x1B) = uart.read() {
                                writeln!(uart, "Break!").unwrap();
                                break;
                            }
                        }
                        writeln!(uart, "Done!").unwrap();
                    }
                    _ => {
                        writeln!(uart, "Unknown command").unwrap();
                    }
                }
                buffer.clear();
            } else if ch[0] == b'\n' {
                // Drop it
            } else if ch[0] == 0x08 || ch[0] == 0x7F {
                buffer.pop();
            } else {
                buffer.push(ch[0]).expect("buffer space");
            }
        }
    }
}

fn parse_integer(s: &str) -> Result<u16, ParseIntError> {
    if let Some(hex) = s.strip_prefix("0x") {
        u16::from_str_radix(hex, 16)
    } else {
        s.parse()
    }
}

fn clean_error<T>(e: nau88c22::Error<T>) -> nau88c22::Error<()> {
    match e {
        nau88c22::Error::I2c(_) => nau88c22::Error::I2c(()),
        nau88c22::Error::WrongDeviceId => nau88c22::Error::WrongDeviceId,
    }
}

// End of file
