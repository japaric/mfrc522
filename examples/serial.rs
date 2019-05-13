//! Serial demo
//!
//! # Connections
//!
//! - 3V3 = VCC
//! - TX = RX (SDA/NSS)
//! - RX = TX (MISO/SCL)
//! - RST = VCC
//! - GND = GND

extern crate serial_embedded_hal as hal;
extern crate mfrc522;

use hal::*;

use mfrc522::Mfrc522;

fn main() {
    let port = std::env::args()
        .skip(1)
        .next()
        .expect("please provide a serial port name as an argument");

    let serial = Serial::new(&port, &PortSettings {
        baud_rate: BaudRate::Baud9600,
        char_size: CharSize::Bits8,
        parity: Parity::ParityNone,
        stop_bits: StopBits::Stop1,
        flow_control: FlowControl::FlowNone,
    }).expect("failed to open serial port");

    let (tx, rx) = serial.split();

    let mut mfrc522 = Mfrc522::new_serial(tx, rx).unwrap();

    let vers = mfrc522.version().unwrap();

    println!("VERSION: 0x{:x}", vers);

    assert!(vers == 0x91 || vers == 0x92);

    loop {
        const CARD_UID: [u8; 4] = [34, 246, 178, 171];
        const TAG_UID: [u8; 4] = [128, 170, 179, 76];

        if let Ok(atqa) = mfrc522.reqa() {
            if let Ok(uid) = mfrc522.select(&atqa) {
                println!("UID: {:x?}", uid);

                if uid.bytes() == &CARD_UID {
                    println!("CARD");
                } else if uid.bytes() == &TAG_UID {
                    println!("TAG");
                }
            }
        }
    }
}
