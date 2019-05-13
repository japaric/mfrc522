//! Communication interfaces for MFRC522

use Register;

/// SPI communication interface
pub mod spi;

pub use self::spi::SpiInterface;

/// Trait for implementing a communication interface for MFRC522
pub trait Mfrc522Interface {
    /// Type of the error the interface may return.
    type Error;

    /// Reads multiple bytes from a register
    fn read_many<'b>(&mut self, reg: Register, buffer: &'b mut [u8])
        -> Result<&'b [u8], Self::Error>;

    /// Writes multiple bytes from a register
    fn write_many(&mut self, reg: Register, bytes: &[u8]) -> Result<(), Self::Error>;

    /// Reads a single register
    fn read(&mut self, reg: Register) -> Result<u8, Self::Error> {
        let mut buffer = [0];

        self.read_many(reg, &mut buffer)?;

        Ok(buffer[0])
    }

    /// Writes a single register
    fn write(&mut self, reg: Register, val: u8) -> Result<(), Self::Error> {
        self.write_many(reg, &[val])
    }

    /// Performs a read-modify-write on a register
    fn rmw<F>(&mut self, reg: Register, f: F) -> Result<(), Self::Error>
    where
        F: FnOnce(u8) -> u8,
    {
        let byte = self.read(reg)?;
        self.write(reg, f(byte))?;
        Ok(())
    }
}
