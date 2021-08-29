#![deny(unsafe_code)]
#![no_std]
extern crate cortex_m_semihosting as sh;
use embedded_hal::blocking::{
    delay::DelayMs,
    i2c::{Write, WriteRead, Read},
};
use sh::hprintln;

const I2C_ADDRESS: u8 = 0x40;

/// Error. // применимость?
#[derive(Debug, Copy, Clone)]
pub enum Error<E> {
    /// Device is not calibrated.
    Uncalibrated,
    /// Underlying bus error.
    Bus(E),
    /// Checksum mismatch.
    Checksum,
}

/// Resolution of temperature
#[derive(Debug, Clone, Copy)]
pub enum TResolution {
    /// 11 bits
    _11,
    /// 14 bits
    _14,
}

/// Resolution of humidity
#[derive(Debug, Clone, Copy)]
pub enum HResolution {
    /// 8 bits
    _8,
    /// 11 bits
    _11,
    /// 14 bits
    _14,
}

struct Register;

impl Register {
    const TEMPERATURE: u8 = 0x00;
    // const HUMIDITY: u8 = 0x01;
    // const CONFIGURATION: u8 = 0x02;
    // const SERIAL_ID1: u8 = 0xFB;
    // const SERIAL_ID2: u8 = 0xFC;
    // const SERIAL_ID3: u8 = 0xFD;
    const MANUFACTURER: u8 = 0xFE;
    const DEVICE_ID: u8 = 0xFF;
}

struct ConfigBitFlags;

impl ConfigBitFlags {
    const RST: u8 = 0b1000_0000;
    // const HEAT: u16 = 0b0010_0000;
     const MODE: u8 = 0b0001_0000;
    // const BTST: u16 = 0b0000_1000_0000_0000;
    // const T_MODE: u16 = 0b0000_0100;
    // const H_MODE9: u16 = 0b000_0010;
    // const H_MODE8: u16 = 0b000_0001;
}

pub struct Hdc1080<I2C, D> {
    i2c: I2C,
    delay: D,
}

impl<I2C, D, E> Hdc1080<I2C, D> where
    I2C: WriteRead<Error = E> + Write<Error = E>+ Read<Error = E>,
    D: DelayMs<u16>,
{
    /// New HDC1080 device from an I2C peripheral.
    pub fn new(i2c: I2C, delay: D ) -> Result<Self, Error<E>> {
        let dev = Self {
            i2c: i2c,
            delay: delay,
        };

        Ok(dev)
    }

    /// Soft resets the sensor.
    pub fn reset(&mut self) -> Result<(), E> {
        // Send soft reset command
        self.i2c.write(I2C_ADDRESS, &[ConfigBitFlags::RST]).unwrap_or_default();

        // Wait 20ms as stated in specification
        self.delay.delay_ms(10);

        Ok(())
    }

    pub fn temperature(& mut self) -> Result<u16,Error<E>> {
        let mut buf:[u8;2] = [0,0];
        let result:u16 ;
        self.i2c.write_read(I2C_ADDRESS, &[Register::TEMPERATURE],&mut buf).unwrap_or_default();
        result = ((buf[0] as u16)<<8)|(buf[1] as u16);
        Ok(result)
    }

    /// expect 0x1050
    pub fn get_device_id(&mut self) -> Result<u16, Error<E>> {
        let mut buf:[u8;2] = [0,0];
        let result:u16 ;
        self.i2c.write_read(I2C_ADDRESS, &[Register::DEVICE_ID],&mut buf).unwrap_or_default();
        result = ((buf[0] as u16)<<8)|(buf[1] as u16);
        Ok(result)
    }
    /// expect 0x5449
    pub fn get_man_id(&mut self) -> Result<u16, Error<E>> {
        let mut buf = [0u8; 2];
        let result:u16 ;
        self.i2c.write(I2C_ADDRESS,&[0xfe]).unwrap_or_default();
        self.delay.delay_ms(20);
        self.i2c.read(I2C_ADDRESS, &mut buf).unwrap_or_default();//Register::DEVICE_ID
        result = ((buf[0] as u16)<<8)|(buf[1] as u16);
        Ok(result)
    }
}