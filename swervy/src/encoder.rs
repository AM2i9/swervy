use core::f32::consts::PI;

use embassy_embedded_hal::shared_bus::{asynch::i2c::I2cDevice, I2cDeviceError};

use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embedded_hal_async::i2c::I2c;
use esp_hal::{peripherals::I2C0, Async};
use esp_println::println;

#[allow(unused)]
#[derive(Clone, Copy)]
pub enum EncoderChannel {
    Zero,
    One,
    Two,
    Three,
    Four,
    Five,
    Six,
    Seven,
}

/// Struct implementation of a MTT6701 Magnetic Absolute Encoder over I2C
pub struct MuxedEncoder<'a> {
    channel: EncoderChannel,
    bus: I2cDevice<'a, NoopRawMutex, esp_hal::i2c::I2c<'static, I2C0, Async>>,
    address: u8,
}

impl<'a> MuxedEncoder<'a> {
    pub fn new(
        bus: I2cDevice<'a, NoopRawMutex, esp_hal::i2c::I2c<'static, I2C0, Async>>,
        channel: EncoderChannel,
        address: u8,
    ) -> Self {
        Self {
            channel,
            bus,
            address,
        }
    }

    /// Raw angle count (min: 0, max: 16384)
    pub async fn get_raw_angle(&mut self) -> Result<u16, I2cDeviceError<esp_hal::i2c::Error>> {
        self.bus.write(0x70, &[1 << (self.channel as u8)]).await?;

        let mut buffer = [0; 2];
        println!("bleh1");
        self
            .bus
            .write_read(self.address, &[0x03], &mut buffer)
            .await?;
        println!("bleh2");

        Ok(((buffer[0] as u16) << 6) | (buffer[1] as u16))
    }

    /// Read angle as a u8 % value (out of 100)
    pub async fn get_angle(&mut self) -> Result<u8, I2cDeviceError<esp_hal::i2c::Error>> {
        // match self.get_raw_angle().await {
        //     Ok(c) => Ok(),
        //     Err(e) => Err(e)
        // }
        // TODO: make this 0-255 instead of 0-100
        Ok(((self.get_raw_angle().await? as f32) / 16384.0) as u8)
    }

    /// Read angle as degrees (0-360)
    pub async fn get_angle_degrees(&mut self) -> Result<u16, I2cDeviceError<esp_hal::i2c::Error>> {
        // match self.get_raw_angle().await {
        //     Ok(angle) => Ok((((angle * 360) as f32) / 16384.0) as u16),
        //     Err(e) => Err(e)
        // }        
        Ok((((self.get_raw_angle().await? * 360) as f32) / 16384.0) as u16)
    }

    /// Read angle as radians (0-2*PI)
    pub async fn get_angle_radians(&mut self) -> Result<u16, I2cDeviceError<esp_hal::i2c::Error>> {
        // match self.get_raw_angle().await {
        //     Ok(angle) => Ok((((angle * 2) as f32 * PI) / 16384.0) as u16),
        //     Err(e) => Err(e)
        // }
        Ok((((self.get_raw_angle().await? * 2) as f32 * PI) / 16384.0) as u16)
    }
}
