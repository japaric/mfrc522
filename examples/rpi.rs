//! Raspberry Pi demo
//!
//! # Connections
//!
//! IMPORTANT: Do *not* use PIN24 / BCM8 / CE0 as the NCS pin
//!
//! - PIN1 = 3V3 = VCC
//! - PIN19 = BCM10 = MOSI
//! - PIN21 = BCM9 = MISO (SCL)
//! - PIN23 = BCM11 = SCLK
//! - PIN22 = BCM25 = NCS
//! - PIN6 = GND = GND

extern crate embedded_hal as hal;
extern crate mfrc522;
extern crate spidev;
extern crate sysfs_gpio;

use std::fs::File;
use std::io::{self, Write};

use mfrc522::Mfrc522;
use spidev::{Spidev, SpidevOptions, SpidevTransfer};
use sysfs_gpio::{Direction, Pin};

pub struct MySpidev(Spidev);

impl hal::blocking::spi::FullDuplex<u8> for MySpidev {
    type Error = io::Error;

    fn transfer<'b>(&mut self, buffer: &'b mut [u8]) -> Result<&'b [u8], io::Error> {
        let tx = buffer.to_owned();
        self.0
            .transfer(&mut SpidevTransfer::read_write(&tx, buffer))?;
        Ok(buffer)
    }

    fn write(&mut self, bytes: &[u8]) -> Result<(), io::Error> {
        let mut rx = bytes.to_owned();
        self.0
            .transfer(&mut SpidevTransfer::read_write(bytes, &mut rx))?;
        Ok(())
    }
}

pub struct MyPin(Pin);

// NOTE this requires tweaking permissions and configuring LED0
//
// ```
// $ echo gpio | sudo tee /sys/class/leds/led0/trigger
// $ sudo chown root:gpio /sys/class/leds/led0/brightness
// $ sudo chmod 770 /sys/class/leds/led0/brightness
// ```
//
// Alternatively you can omit the LED and comment out the contents of the `on` and `off` methods
// below
pub struct Led;

impl Led {
    fn on(&mut self) {
        File::create("/sys/class/leds/led0/brightness")
            .unwrap()
            .write_all(b"1\n")
            .unwrap();
    }

    fn off(&mut self) {
        File::create("/sys/class/leds/led0/brightness")
            .unwrap()
            .write_all(b"0\n")
            .unwrap();
    }
}

impl hal::digital::OutputPin for MyPin {
    fn set_high(&mut self) {
        self.0.set_value(1).unwrap()
    }

    fn set_low(&mut self) {
        self.0.set_value(0).unwrap()
    }

    fn is_low(&self) -> bool {
        self.0.get_value().unwrap() == 0
    }

    fn is_high(&self) -> bool {
        !self.is_low()
    }
}

fn main() {
    let mut spi = Spidev::open("/dev/spidev0.0").unwrap();
    let options = SpidevOptions::new()
        .max_speed_hz(1_000_000)
        .mode(spidev::SPI_MODE_0)
        .build();
    spi.configure(&options).unwrap();

    let pin = Pin::new(25);
    pin.export().unwrap();
    while !pin.is_exported() {}
    pin.set_direction(Direction::Out).unwrap();
    pin.set_value(1).unwrap();

    let mut led = Led;
    let mut mfrc522 = Mfrc522::new(MySpidev(spi), MyPin(pin)).unwrap();

    let vers = mfrc522.version().unwrap();

    println!("VERSION: 0x{:x}", vers);

    assert!(vers == 0x91 || vers == 0x92);

    loop {
        const CARD_UID: [u8; 4] = [34, 246, 178, 171];
        const TAG_UID: [u8; 4] = [128, 170, 179, 76];

        if let Ok(atqa) = mfrc522.reqa() {
            if let Ok(uid) = mfrc522.select(&atqa) {
                println!("UID: {:?}", uid);

                if uid.bytes() == &CARD_UID {
                    led.off();
                    println!("CARD");
                } else if uid.bytes() == &TAG_UID {
                    led.on();
                    println!("TAG");
                }
            }
        }
    }
}
