#![no_std]
#![no_main]
#![allow(non_snake_case)]
pub mod adafruit_MCP4728;
use adafruit_MCP4728::AdafruitMCP4728;

// use core::fmt::Write;
use embedded_hal::delay::DelayNs;
use fugit::RateExtU32;
use panic_halt as _;
use rp_pico::entry;
use rp_pico::hal;
use rp_pico::hal::pac;

#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure two pins as being I²C, not GPIO
    let sda_pin: hal::gpio::Pin<_, hal::gpio::FunctionI2C, _> = pins.gpio16.reconfigure();
    let scl_pin: hal::gpio::Pin<_, hal::gpio::FunctionI2C, _> = pins.gpio17.reconfigure();

    // Create the I²C driver, using the two pre-configured pins. This will fail
    // at compile time if the pins are in the wrong mode, or if this I²C
    // peripheral isn't available on these pins!
    let i2c = hal::I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        400.kHz(),
        // linux_embedded_hal::i2c::Speed::Standard(400_000),
        &mut pac.RESETS,
        &clocks.peripheral_clock,
    );

    let address: u8 = 0x60;

    let mut dac = AdafruitMCP4728::new(i2c, address);
    // dac.begin()?;
    // Set the channel value
    // let channel = 0; // Channel number (0-3)
    // let value = 2048; // Value to set (0-4095 for 12-bit DAC)
    // dac.set_channel_value(channel, value)?;
    dac.set_channel_value(0, 0);
    dac.set_channel_value(1, 1000);
    dac.set_channel_value(2, 2048);
    dac.set_channel_value(3, 4095);

    loop {}
}
