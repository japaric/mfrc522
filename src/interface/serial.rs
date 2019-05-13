///! Serial communication interface for MFRC522

use hal::serial::{Read, Write};
use Register;
use interface::Mfrc522Interface;

/// SPI communication interface for MFRC522
pub struct SerialInterface<W, R> {
    write: W,
    read: R,
}

impl<E, W, R> SerialInterface<W, R>
where
    W: Write<u8, Error = E>,
    R: Read<u8, Error = E>,
{
    /// Default serial baud rate for MRFC522.
    pub const BAUD_RATE: u32 = 9600;

    /// Creates a interface using the provided serial read/write halves.
    pub fn new(write: W, read: R) -> Self {
        Self { write, read }
    }
}

impl<E, W, R> Mfrc522Interface for SerialInterface<W, R>
where
    W: Write<u8, Error = E>,
    R: Read<u8, Error = E>,
{
    type Error = E;

    fn read_many<'b>(&mut self, reg: Register, buffer: &'b mut [u8])
        -> Result<&'b [u8], Self::Error>
    {
        let addr = (reg as u8) | (1 << 7);

        for slot in &mut buffer[..] {
            block!(self.write.write(addr))?;
            *slot = block!(self.read.read())?;
        }

        Ok(buffer)
    }

    fn write_many(&mut self, reg: Register, bytes: &[u8]) -> Result<(), Self::Error> {
        let addr = reg as u8;

        for &b in bytes {
            block!(self.write.write(addr))?;
            block!(self.write.write(b))?;
            block!(self.read.read())?;
        }

        Ok(())
    }
}