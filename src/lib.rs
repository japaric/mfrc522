//! A platform agnostic driver to interface the MFRC522 (RFID reader/writer)
//!
//! This driver was built using [`embedded-hal`] traits.
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal/~0.1
//!
//! # Examples
//!
//! You'll find an example for the Raspeberry Pi in the `examples` directory. You should find an
//! example for ARM Cortex-M microcontrollers on the [`blue-pill`] repository. If that branch is
//! gone, check the master branch.
//!
//! [`blue-pill`]: https://github.com/japaric/blue-pill/tree/singletons/examples
//!
//! # References
//!
//! - [Identification cards - Contactless integrated circuit(s) cards - Proximity cards - Part 3:
//! Initialization and anticollision][1]
//! - [MFRC522 data sheet][2]
//!
//! [1]: http://wg8.de/wg8n1496_17n3613_Ballot_FCD14443-3.pdf
//! [2]: https://www.nxp.com/docs/en/data-sheet/MFRC522.pdf

#![allow(dead_code)]
#![deny(missing_docs)]
#![deny(warnings)]
#![no_std]

extern crate embedded_hal as hal;
extern crate generic_array;

use core::mem;

use generic_array::typenum::consts::*;
use generic_array::{ArrayLength, GenericArray};
use hal::blocking::spi;
use hal::digital::v2::OutputPin;
use hal::spi::{Mode, Phase, Polarity};

mod picc;

/// Errors
#[derive(Debug)]
pub enum Error<E, OPE> {
    /// Wrong Block Character Check (BCC)
    Bcc,
    /// FIFO buffer overflow
    BufferOverflow,
    /// Collision
    Collision,
    /// Wrong CRC
    Crc,
    /// Incomplete RX frame
    IncompleteFrame,
    /// Internal temperature sensor detects overheating
    Overheating,
    /// Parity check failed
    Parity,
    /// Error during MFAuthent operation
    Protocol,
    /// SPI bus error
    Spi(E),
    /// Timeout
    Timeout,
    /// ???
    Wr,
    /// Output pin operation failed
    OutputPin(OPE),
}

// XXX coherence :-(
// impl<SPI> From<SPI::Error> for Error<SPI>
// where
//     SPI: spi::FullDuplex<u8>,
// {
//     fn from(e: SPI::Error) -> Error<SPI> {
//         Error::Spi(e)
//     }
// }

/// MFRC522 driver
pub struct Mfrc522<SPI, NSS> {
    spi: SPI,
    nss: NSS,
}

const ERR_IRQ: u8 = 1 << 1;
const IDLE_IRQ: u8 = 1 << 4;
const RX_IRQ: u8 = 1 << 5;
const TIMER_IRQ: u8 = 1 << 0;

const CRC_IRQ: u8 = 1 << 2;

impl<E, OPE, NSS, SPI> Mfrc522<SPI, NSS>
where
    SPI: spi::Transfer<u8, Error = E> + spi::Write<u8, Error = E>,
    NSS: OutputPin<Error = OPE>,
{
    /// Creates a new driver from a SPI driver and a NSS pin
    pub fn new(spi: SPI, nss: NSS) -> Result<Self, Error<E, OPE>> {
        let mut mfrc522 = Mfrc522 { spi, nss };

        // soft reset
        mfrc522.command(Command::SoftReset)?;

        while mfrc522.read(Register::Command)? & (1 << 4) != 0 {}

        // configure timer to operate at 10 KHz.
        // f_timer = 13.56 MHz / (2 + TPrescaler + 2)
        mfrc522.write(Register::Demod, 0x4d | (1 << 4))?;
        mfrc522.write(Register::TMode, 0x0 | (1 << 7) | 0b10)?;
        mfrc522.write(Register::TPrescaler, 165)?;

        // configure timer for a 5 ms timeout
        mfrc522.write(Register::ReloadL, 50)?;

        // forces 100% ASK modulation
        // NOTE my tags don't work without this ...
        mfrc522.write(Register::TxAsk, 1 << 6)?;

        // set preset value for the CRC co-processor to 0x6363
        // in accordance to section 6.2.4 of ISO/IEC FCD 14443-3
        mfrc522.write(Register::Mode, (0x3f & (!0b11)) | 0b01)?;

        // enable the antenna
        mfrc522.write(Register::TxControl, 0x80 | 0b11)?;

        Ok(mfrc522)
    }

    /// Sends a REQuest type A to nearby PICCs
    pub fn reqa<'b>(&mut self) -> Result<AtqA, Error<E, OPE>> {
        // NOTE REQA is a short frame (7 bits)
        self.transceive(&[picc::REQA], 7)
            .map(|bytes| AtqA { bytes })
    }

    /// Selects an idle PICC
    ///
    /// NOTE currently this only supports single size UIDs
    // TODO anticollision loop
    // TODO add optional UID to select an specific PICC
    pub fn select(&mut self, _atqa: &AtqA) -> Result<Uid, Error<E, OPE>> {
        let rx = self.transceive::<U5>(&[picc::SEL_CL1, 0x20], 0)?;

        assert_ne!(
            rx[0],
            picc::CT,
            "double and triple size UIDs are currently not supported"
        );

        let expected_bcc = rx[4];
        let computed_bcc = rx[0] ^ rx[1] ^ rx[2] ^ rx[3];

        // XXX can this ever fail? (buggy PICC?)
        if computed_bcc != expected_bcc {
            return Err(Error::Bcc);
        }

        let mut tx: [u8; 9] = unsafe { mem::uninitialized() };
        tx[0] = picc::SEL_CL1;
        tx[1] = 0x70;
        tx[2..7].copy_from_slice(&rx);

        let crc = self.calculate_crc(&tx[..7])?;
        tx[7..].copy_from_slice(&crc);

        // enable automatic CRC validation during reception
        let rx2 = self.transceive::<U3>(&tx, 0)?;

        let crc2 = self.calculate_crc(&rx2[..1])?;

        if &rx2[1..] != &crc2 {
            return Err(Error::Crc);
        }

        let sak = rx2[0];

        let compliant = match (sak & (1 << 2) != 0, sak & (1 << 5) != 0) {
            // indicates that the UID is incomplete -- this is unreachable because we only support
            // single size UIDs
            (_, true) => unreachable!(),
            (true, false) => true,
            (false, false) => false,
        };

        Ok(Uid {
            bytes: [rx[0], rx[1], rx[2], rx[3]],
            compliant,
        })
    }

    /// Returns the version of the MFRC522
    pub fn version(&mut self) -> Result<u8, Error<E, OPE>> {
        self.read(Register::Version)
    }

    fn calculate_crc(&mut self, data: &[u8]) -> Result<[u8; 2], Error<E, OPE>> {
        // stop any ongoing command
        self.command(Command::Idle)?;

        // clear the CRC_IRQ interrupt flag
        self.write(Register::DivIrq, 1 << 2)?;

        // flush FIFO buffer
        self.flush_fifo_buffer()?;

        // write data to transmit to the FIFO buffer
        self.write_many(Register::FifoData, data)?;

        self.command(Command::CalcCRC)?;

        // TODO timeout when connection to the MFRC522 is lost
        // wait for CRC to complete
        let mut irq;
        loop {
            irq = self.read(Register::DivIrq)?;

            if irq & CRC_IRQ != 0 {
                self.command(Command::Idle)?;
                let crc = [
                    self.read(Register::CrcResultL)?,
                    self.read(Register::CrcResultH)?,
                ];

                break Ok(crc);
            }
        }
    }

    fn check_error_register(&mut self) -> Result<(), Error<E, OPE>> {
        const PROTOCOL_ERR: u8 = 1 << 0;
        const PARITY_ERR: u8 = 1 << 1;
        const CRC_ERR: u8 = 1 << 2;
        const COLL_ERR: u8 = 1 << 3;
        const BUFFER_OVFL: u8 = 1 << 4;
        const TEMP_ERR: u8 = 1 << 6;
        const WR_ERR: u8 = 1 << 7;

        let err = self.read(Register::Error)?;

        if err & PROTOCOL_ERR != 0 {
            Err(Error::Protocol)
        } else if err & PARITY_ERR != 0 {
            Err(Error::Parity)
        } else if err & CRC_ERR != 0 {
            Err(Error::Crc)
        } else if err & COLL_ERR != 0 {
            Err(Error::Collision)
        } else if err & BUFFER_OVFL != 0 {
            Err(Error::BufferOverflow)
        } else if err & TEMP_ERR != 0 {
            Err(Error::Overheating)
        } else if err & WR_ERR != 0 {
            Err(Error::Wr)
        } else {
            Ok(())
        }
    }

    fn command(&mut self, command: Command) -> Result<(), Error<E, OPE>> {
        self.write(Register::Command, command.value())
    }

    fn flush_fifo_buffer(&mut self) -> Result<(), Error<E, OPE>> {
        self.write(Register::FifoLevel, 1 << 7)
    }

    fn transceive<RX>(
        &mut self,
        tx_buffer: &[u8],
        tx_last_bits: u8,
    ) -> Result<GenericArray<u8, RX>, Error<E, OPE>>
    where
        RX: ArrayLength<u8>,
    {
        // stop any ongoing command
        self.command(Command::Idle)?;

        // clear all interrupt flags
        self.write(Register::ComIrq, 0x7f)?;

        // flush FIFO buffer
        self.flush_fifo_buffer()?;

        // write data to transmit to the FIFO buffer
        self.write_many(Register::FifoData, tx_buffer)?;

        // signal command
        self.command(Command::Transceive)?;

        // configure short frame and start transmission
        self.write(Register::BitFraming, (1 << 7) | tx_last_bits)?;

        // TODO timeout when connection to the MFRC522 is lost (?)
        // wait for transmission + reception to complete
        let mut irq;
        loop {
            irq = self.read(Register::ComIrq)?;

            if irq & (RX_IRQ | ERR_IRQ | IDLE_IRQ) != 0 {
                break;
            } else if irq & TIMER_IRQ != 0 {
                return Err(Error::Timeout);
            }
        }

        // XXX do we need a guard here?
        // check for any outstanding error
        // if irq & ERR_IRQ != 0 {
        self.check_error_register()?;
        // }

        // grab RX data
        let mut rx_buffer: GenericArray<u8, RX> = unsafe { mem::uninitialized() };

        {
            let rx_buffer: &mut [u8] = &mut rx_buffer;

            let received_bytes = self.read(Register::FifoLevel)?;

            if received_bytes as usize != rx_buffer.len() {
                return Err(Error::IncompleteFrame);
            }

            self.read_many(Register::FifoData, rx_buffer)?;
        }

        Ok(rx_buffer)
    }

    // lowest level  API
    fn read(&mut self, reg: Register) -> Result<u8, Error<E, OPE>> {
        let mut buffer = [reg.read_address(), 0];

        self.with_nss_low(|mfr| {
            let buffer = mfr.spi.transfer(&mut buffer)?;

            Ok(buffer[1])
        })
    }

    fn read_many<'b>(
        &mut self,
        reg: Register,
        buffer: &'b mut [u8],
    ) -> Result<&'b [u8], Error<E, OPE>> {
        let byte = reg.read_address();

        self.with_nss_low(move |mfr| {
            mfr.spi.transfer(&mut [byte])?;

            let n = buffer.len();
            for slot in &mut buffer[..n - 1] {
                *slot = mfr.spi.transfer(&mut [byte])?[0];
            }

            buffer[n - 1] = mfr.spi.transfer(&mut [0])?[0];

            Ok(&*buffer)
        })
    }

    fn rmw<F>(&mut self, reg: Register, f: F) -> Result<(), Error<E, OPE>>
    where
        F: FnOnce(u8) -> u8,
    {
        let byte = self.read(reg)?;
        self.write(reg, f(byte))?;
        Ok(())
    }

    fn write(&mut self, reg: Register, val: u8) -> Result<(), Error<E, OPE>> {
        self.with_nss_low(|mfr| mfr.spi.write(&[reg.write_address(), val]))
    }

    fn write_many(&mut self, reg: Register, bytes: &[u8]) -> Result<(), Error<E, OPE>> {
        self.with_nss_low(|mfr| {
            mfr.spi.write(&[reg.write_address()])?;
            mfr.spi.write(bytes)?;

            Ok(())
        })
    }

    fn with_nss_low<F, T>(&mut self, f: F) -> Result<T, Error<E, OPE>>
    where
        F: FnOnce(&mut Self) -> Result<T, E>,
    {
        self.nss.set_low().map_err(Error::OutputPin)?;
        let f_result = f(self).map_err(Error::Spi);
        let pin_result = self.nss.set_high().map_err(Error::OutputPin);

        if f_result.is_ok() {
            pin_result?;
        }
        f_result
    }
}

/// SPI mode
pub const MODE: Mode = Mode {
    polarity: Polarity::IdleLow,
    phase: Phase::CaptureOnFirstTransition,
};

#[derive(Clone, Copy)]
enum Command {
    Idle,
    Mem,
    GenerateRandomID,
    CalcCRC,
    Transmit,
    NoCmdChange,
    Receive,
    Transceive,
    MFAuthent,
    SoftReset,
}

impl Command {
    fn value(&self) -> u8 {
        match *self {
            Command::Idle => 0b0000,
            Command::Mem => 0b0001,
            Command::GenerateRandomID => 0b0010,
            Command::CalcCRC => 0b0011,
            Command::Transmit => 0b0100,
            Command::NoCmdChange => 0b0111,
            Command::Receive => 0b1000,
            Command::Transceive => 0b1100,
            Command::MFAuthent => 0b1110,
            Command::SoftReset => 0b1111,
        }
    }
}

#[derive(Clone, Copy)]
enum Register {
    BitFraming = 0x0d,
    Coll = 0x0e,
    ComIrq = 0x04,
    Command = 0x01,
    CrcResultH = 0x21,
    CrcResultL = 0x22,
    Demod = 0x19,
    DivIrq = 0x05,
    Error = 0x06,
    FifoData = 0x09,
    FifoLevel = 0x0a,
    ModWidth = 0x24,
    Mode = 0x11,
    ReloadH = 0x2c,
    ReloadL = 0x2d,
    RxMode = 0x13,
    TCountValH = 0x2e,
    TCountValL = 0x2f,
    TMode = 0x2a,
    TPrescaler = 0x2b,
    TxAsk = 0x15,
    TxControl = 0x14,
    TxMode = 0x12,
    Version = 0x37,
}

const R: u8 = 1 << 7;
#[allow(dead_code)]
const W: u8 = 0 << 7;

impl Register {
    fn read_address(&self) -> u8 {
        ((*self as u8) << 1) | R
    }

    fn write_address(&self) -> u8 {
        ((*self as u8) << 1) | W
    }
}

/// Answer To reQuest A
pub struct AtqA {
    bytes: GenericArray<u8, U2>,
}

/// Single size UID
#[derive(Debug)]
pub struct Uid {
    bytes: [u8; 4],
    compliant: bool,
}

impl Uid {
    /// The bytes of the UID
    pub fn bytes(&self) -> &[u8; 4] {
        &self.bytes
    }

    /// Is the PICC compliant with ISO/IEC 14443-4?
    pub fn is_compliant(&self) -> bool {
        self.compliant
    }
}
