//! Raspberry Pi demo
//!
//! # Connections
//!
//! IMPORTANT: Do *not* use PIN24 / BCM8 / CE0 as the NSS pin
//!
//! - PIN1 = 3V3 = VCC
//! - PIN19 = BCM10 = MOSI
//! - PIN21 = BCM9 = MISO (SCL)
//! - PIN23 = BCM11 = SCLK
//! - PIN22 = BCM25 = NSS (SDA)
//! - PIN6 = GND = GND

extern crate linux_embedded_hal as hal;
extern crate mfrc522;

use std::fs::File;
use std::io::Write;

use hal::spidev::SpidevOptions;
use hal::sysfs_gpio::Direction;
use hal::{Pin, Spidev};
use mfrc522::Mfrc522;

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

fn main() {
    let mut spi = Spidev::open("/dev/spidev0.0").unwrap();
    let options = SpidevOptions::new()
        .max_speed_hz(1_000_000)
        .mode(hal::spidev::SPI_MODE_0)
        .build();
    spi.configure(&options).unwrap();

    let pin = Pin::new(25);
    pin.export().unwrap();
    while !pin.is_exported() {}
    pin.set_direction(Direction::Out).unwrap();
    pin.set_value(1).unwrap();

    let mut led = Led;
    let mut mfrc522 = Mfrc522::new(spi, pin).unwrap();

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
