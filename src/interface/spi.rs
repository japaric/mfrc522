#![allow(deprecated)] // FIXME: Remove when embedded-hal is updated

///! SPI communication interface for MFRC522

use hal::blocking::spi;
use hal::spi::{Mode, Phase, Polarity};
use hal::digital::OutputPin;
use Register;
use interface::Mfrc522Interface;

/// SPI communication interface for MFRC522
pub struct SpiInterface<SPI, NSS> {
    spi: SPI,
    nss: NSS,
}

impl<E, SPI, NSS> SpiInterface<SPI, NSS>
where
    SPI: spi::Transfer<u8, Error=E> + spi::Write<u8, Error=E>,
    NSS: OutputPin,
{
    /// SPI mode. Pass to the platform SPI implementation to configure the bus correctly for
    /// MFRC522.
    pub const MODE: Mode = Mode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnFirstTransition,
    };

    /// Creates a interface using the provided SPI peripheral and NSS pin.
    pub fn new(spi: SPI, nss: NSS) -> Self {
        Self { spi, nss }
    }

    fn with_nss_low<F, T>(&mut self, f: F) -> T
    where
        F: FnOnce(&mut Self) -> T,
    {
        self.nss.set_low();
        let result = f(self);
        self.nss.set_high();

        result
    }
}

impl<E, SPI, NSS> Mfrc522Interface for SpiInterface<SPI, NSS>
where
    SPI: spi::Transfer<u8, Error=E> + spi::Write<u8, Error=E>,
    NSS: OutputPin,
{
    type Error = E;

    fn read_many<'b>(&mut self, reg: Register, buffer: &'b mut [u8])
        -> Result<&'b [u8], Self::Error>
    {
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

    fn write_many(&mut self, reg: Register, bytes: &[u8]) -> Result<(), Self::Error> {
        self.with_nss_low(|mfr| {
            mfr.spi.write(&[reg.write_address()])?;
            mfr.spi.write(bytes)?;

            Ok(())
        })
    }
}