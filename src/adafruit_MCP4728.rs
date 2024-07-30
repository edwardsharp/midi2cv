// so this is a crude, minimal port of this: https://github.com/adafruit/Adafruit_MCP4728/
use embedded_hal::i2c::{Error, I2c};

pub struct AdafruitMCP4728<I2C> {
    i2c: I2C,
    address: u8,
}

// commands:
const MCP4728_MULTI_IR_CMD: u8 = 0x40;
const RESET: u8 = 0x06;

// opinionated defaults from what was otherwise enums
// see also: https://github.com/adafruit/Adafruit_MCP4728/blob/master/Adafruit_MCP4728.h
const MCP4728_PD_MODE_NORMAL: u8 = 0b00;
const MCP4728_GAIN_1X: u8 = 0b00;
const MCP4728_VREF_VDD: u8 = 0b00;
const MCP4728_UDAC: u8 = 0;

impl<I2C: I2c> AdafruitMCP4728<I2C> {
    pub fn new(i2c: I2C, address: u8) -> Self {
        Self { i2c, address }
    }

    pub fn reset(&mut self) -> Result<(), I2C::Error> {
        self.i2c.write(self.address, &[RESET])
    }

    pub fn set_channel_value(&mut self, channel: u8, value: u16) -> Result<(), I2C::Error> {
        let mut new_value: u16 = value;

        let mut output_buffer: [u8; 3] = [0; 3];

        // build the setter header/ "address"
        // 0 1 0 0 0 DAC1 DAC0 UDAC[A]
        let mut sequential_write_cmd = MCP4728_MULTI_IR_CMD;
        sequential_write_cmd |= (channel << 1);
        sequential_write_cmd |= MCP4728_UDAC;

        output_buffer[0] = sequential_write_cmd;

        // VREF PD1 PD0 Gx D11 D10 D9 D8 [A] D7 D6 D5 D4 D3 D2 D1 D0 [A]
        new_value |= ((MCP4728_VREF_VDD as u16) << 15);
        new_value |= ((MCP4728_PD_MODE_NORMAL as u16) << 13);
        new_value |= ((MCP4728_GAIN_1X as u16) << 12);

        output_buffer[1] = (new_value >> 8) as u8;
        output_buffer[2] = (new_value & 0xFF) as u8;

        self.i2c.write(self.address, &output_buffer)
    }
}
