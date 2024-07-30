#![no_std]
#![no_main]
#![allow(non_snake_case)]
pub mod adafruit_MCP4728;
use adafruit_MCP4728::AdafruitMCP4728;

use cortex_m::delay::Delay;
use embedded_hal::digital::OutputPin;
use fugit::RateExtU32;
use hal::gpio::{bank0, FunctionI2C, FunctionSioOutput, Pin, PullNone};
use panic_halt as _;
use rp_pico::entry;
use rp_pico::hal;

use rp_pico::hal::pac;

fn shift_out(
    data_pin: &mut Pin<bank0::Gpio4, FunctionSioOutput, PullNone>,
    clock_pin: &mut Pin<bank0::Gpio3, FunctionSioOutput, PullNone>,
    value: u8,
    delay: &mut Delay,
) {
    for i in 0..8 {
        if (value & (1 << i)) != 0 {
            data_pin.set_high().unwrap();
        } else {
            data_pin.set_low().unwrap();
        }
        clock_pin.set_high().unwrap();
        delay.delay_us(1);
        clock_pin.set_low().unwrap();
        delay.delay_us(1);
    }
}

fn bit_write(byte: &mut u8, bit: u8, value: bool) {
    if value {
        *byte |= 1 << bit;
    } else {
        *byte &= !(1 << bit);
    }
}

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    // watchdog driver needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // default is to generate a 125 MHz system clock
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

    // single-cycle I/O block controls GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // set the pins up according to their function on this particular board (pico pi)
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // configure two I2C pins, not GPIO
    let sda_pin: Pin<_, FunctionI2C, _> = pins.gpio16.reconfigure();
    let scl_pin: Pin<_, FunctionI2C, _> = pins.gpio17.reconfigure();

    // create the I2C driver, using the two pre-configured pins; will fail
    // at compile time if the pins are in the wrong mode, or if this I2C
    // peripheral isn't available on these pins!
    let i2c = hal::I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        400.kHz(), // from fugit::RateExtU32
        &mut pac.RESETS,
        &clocks.peripheral_clock,
    );

    let address: u8 = 0x60;

    let mut dac = AdafruitMCP4728::new(i2c, address);

    // set the channel value, for example:
    // let channel = 0; // Channel number (0-3)
    // let value = 2048; // Value to set (0-4095 for 12-bit DAC)
    // dac.set_channel_value(channel, value);
    // #TODO: should handle errors, here :/
    let _ = dac.set_channel_value(0, 0);
    let _ = dac.set_channel_value(1, 1000);
    let _ = dac.set_channel_value(2, 2048);
    let _ = dac.set_channel_value(3, 4095);

    let mut dac0value: u16 = 0;

    // 74HC595 shift register stuff
    let core = pac::CorePeripherals::take().unwrap();

    let mut latchPin: Pin<_, FunctionSioOutput, PullNone> = pins.gpio2.reconfigure(); // ST_CP latch - green wire
    let mut clockPin: Pin<_, FunctionSioOutput, PullNone> = pins.gpio3.reconfigure(); // SH_CP clock - yellow wire
    let mut dataPin: Pin<_, FunctionSioOutput, PullNone> = pins.gpio4.reconfigure(); // DS data - blue wire
    let mut delay = Delay::new(core.SYST, 125_000_000); // ...assuming 125 MHz clock

    let mut bitToSet: u8 = 0;
    loop {
        let _ = dac.set_channel_value(0, dac0value);
        dac0value += 100;
        if dac0value >= 4095 {
            dac0value = 0;
        }

        // 74HC595 shift register blinky stuff
        latchPin.set_low().unwrap();
        let mut bitsToSend: u8 = 0;
        bit_write(&mut bitsToSend, bitToSet, true);
        shift_out(&mut dataPin, &mut clockPin, bitsToSend, &mut delay);
        latchPin.set_high().unwrap();
        bitToSet += 1;
        if bitToSet > 7 {
            bitToSet = 0;
        }

        delay.delay_ms(100);
    }
}
