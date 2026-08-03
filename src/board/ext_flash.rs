//! Driver for the Winbond W25Q NOR flash line, implementing `embedded-storage-async::NorFlash`.
//!
//! The part is identified from its JEDEC id by [`W25Q::probe`]. Both the capacity and the address
//! width come from that id.

#![allow(
    clippy::map_err_ignore,
    reason = "the opaque SpiDevice error is intentionally collapsed into FlashError::Spi"
)]

use embassy_time::{Duration, Instant, Timer};
use embedded_hal_async::spi::{Operation, SpiDevice};
use embedded_storage_async::nor_flash::{
    ErrorType, MultiwriteNorFlash, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash,
};

use defmt::*;

pub const PAGE_SIZE: u32 = 256;
pub const SECTOR_SIZE: u32 = 4096;

const JEDEC_MANUFACTURER: u8 = 0xef; // Winbond

const OP_WRITE_ENABLE: u8 = 0x06;
const OP_READ_STATUS_1: u8 = 0x05;
const OP_JEDEC_ID: u8 = 0x9f;

const STATUS_BUSY: u8 = 0x01;

// Datasheet maxima are 3ms (page program) and 400ms (sector erase)
const PAGE_PROGRAM_TIMEOUT: Duration = Duration::from_millis(10);
const SECTOR_ERASE_TIMEOUT: Duration = Duration::from_millis(1000);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub enum FlashError {
    Spi,
    Timeout,
    OutOfBounds,
    NotAligned,
    /// The JEDEC id is not a W25Q part this driver knows the geometry of.
    UnknownDevice {
        manufacturer: u8,
        memory_type: u8,
        capacity_code: u8,
    },
}

/// How many address bytes a part's read/program/erase instructions take, and with it which opcodes
/// to use. The 4-byte variants are only defined on parts larger than 16 MiB.
#[derive(Clone, Copy)]
enum AddressWidth {
    Three,
    Four,
}

/// An opcode plus its address, sized to the part. Kept as an enum rather than a buffer and a length
/// so a frame can only ever be sent at exactly its own width.
enum Command {
    ThreeByte([u8; 4]),
    FourByte([u8; 5]),
}

pub struct W25Q<SPI> {
    spi: SPI,
    size: u32,
    address_width: AddressWidth,
}

/// Size and address width of a part, from the capacity byte of its JEDEC id.
fn geometry(capacity_code: u8) -> Option<(u32, AddressWidth)> {
    match capacity_code {
        0x14 => Some((0x0010_0000, AddressWidth::Three)), // 1 MiB, W25Q80
        0x15 => Some((0x0020_0000, AddressWidth::Three)), // 2 MiB, W25Q16
        0x16 => Some((0x0040_0000, AddressWidth::Three)), // 4 MiB, W25Q32
        0x17 => Some((0x0080_0000, AddressWidth::Three)), // 8 MiB, W25Q64
        0x18 => Some((0x0100_0000, AddressWidth::Three)), // 16 MiB, W25Q128
        0x19 => Some((0x0200_0000, AddressWidth::Four)),  // 32 MiB, W25Q256
        0x20 => Some((0x0400_0000, AddressWidth::Four)),  // 64 MiB, W25Q512
        _ => None,
    }
}

impl NorFlashError for FlashError {
    fn kind(&self) -> NorFlashErrorKind {
        match self {
            FlashError::OutOfBounds => NorFlashErrorKind::OutOfBounds,
            FlashError::NotAligned => NorFlashErrorKind::NotAligned,
            _ => NorFlashErrorKind::Other,
        }
    }
}

impl AddressWidth {
    fn count(self) -> u8 {
        match self {
            AddressWidth::Three => 3,
            AddressWidth::Four => 4,
        }
    }

    fn read_data(self) -> u8 {
        match self {
            AddressWidth::Three => 0x03,
            AddressWidth::Four => 0x13,
        }
    }

    fn page_program(self) -> u8 {
        match self {
            AddressWidth::Three => 0x02,
            AddressWidth::Four => 0x12,
        }
    }

    fn sector_erase(self) -> u8 {
        match self {
            AddressWidth::Three => 0x20,
            AddressWidth::Four => 0x21,
        }
    }
}

impl Command {
    fn bytes(&self) -> &[u8] {
        match self {
            Command::ThreeByte(frame) => frame,
            Command::FourByte(frame) => frame,
        }
    }
}

impl<SPI: SpiDevice<u8>> W25Q<SPI> {
    /// The part is unidentified until [`Self::probe`] succeeds. Size stays zero until then, so any
    /// access without a successful probe is refused rather than issued to an unknown chip.
    pub fn new(spi: SPI) -> Self {
        Self {
            spi,
            size: 0,
            address_width: AddressWidth::Three,
        }
    }

    /// Total size of the identified part in bytes; zero before a successful [`Self::probe`].
    pub fn size(&self) -> u32 {
        self.size
    }

    /// Identifies the part from its JEDEC id and records the geometry the rest of the driver works
    /// from. Retried a few times because the chip may still be leaving power-on reset when we first
    /// get here.
    pub async fn probe(&mut self) -> Result<(), FlashError> {
        let mut id = (0, 0, 0);

        for _i in 0..3 {
            id = self.read_jedec_id().await?;
            let (manufacturer, memory_type, capacity_code) = id;

            if manufacturer == JEDEC_MANUFACTURER
                && let Some((size, address_width)) = geometry(capacity_code)
            {
                self.size = size;
                self.address_width = address_width;

                info!(
                    "W25Q identified (JEDEC {:02x} {:02x} {:02x}): {} bytes, {}-byte addressing",
                    manufacturer,
                    memory_type,
                    capacity_code,
                    size,
                    address_width.count()
                );

                return Ok(());
            }

            Timer::after(Duration::from_micros(100)).await;
        }

        let (manufacturer, memory_type, capacity_code) = id;
        error!(
            "Flash JEDEC id {:02x} {:02x} {:02x} is not a known W25Q part",
            manufacturer, memory_type, capacity_code
        );

        Err(FlashError::UnknownDevice {
            manufacturer,
            memory_type,
            capacity_code,
        })
    }

    /// Returns the manufacturer, memory type and capacity code reported by the 9Fh instruction.
    async fn read_jedec_id(&mut self) -> Result<(u8, u8, u8), FlashError> {
        let mut payload = [OP_JEDEC_ID, 0, 0, 0];
        self.spi.transfer_in_place(&mut payload).await.map_err(|_| FlashError::Spi)?;
        Ok((payload[1], payload[2], payload[3]))
    }

    fn command(&self, opcode: u8, address: u32) -> Command {
        let [a3, a2, a1, a0] = address.to_be_bytes();
        match self.address_width {
            AddressWidth::Three => Command::ThreeByte([opcode, a2, a1, a0]),
            AddressWidth::Four => Command::FourByte([opcode, a3, a2, a1, a0]),
        }
    }

    async fn read_status_1(&mut self) -> Result<u8, FlashError> {
        let mut payload = [OP_READ_STATUS_1, 0];
        self.spi.transfer_in_place(&mut payload).await.map_err(|_| FlashError::Spi)?;
        Ok(payload[1])
    }

    async fn write_enable(&mut self) -> Result<(), FlashError> {
        self.spi.write(&[OP_WRITE_ENABLE]).await.map_err(|_| FlashError::Spi)
    }

    async fn wait_idle(&mut self, poll: Duration, timeout: Duration) -> Result<(), FlashError> {
        let start = Instant::now();
        loop {
            if self.read_status_1().await? & STATUS_BUSY == 0 {
                return Ok(());
            }
            if start.elapsed() > timeout {
                return Err(FlashError::Timeout);
            }
            Timer::after(poll).await;
        }
    }

    fn check_range(&self, offset: u32, len: usize) -> Result<(), FlashError> {
        let end = (offset as usize).checked_add(len).ok_or(FlashError::OutOfBounds)?;

        if end > self.size as usize {
            return Err(FlashError::OutOfBounds);
        }

        Ok(())
    }
}

impl<SPI: SpiDevice<u8>> ErrorType for W25Q<SPI> {
    type Error = FlashError;
}

impl<SPI: SpiDevice<u8>> ReadNorFlash for W25Q<SPI> {
    const READ_SIZE: usize = 1;

    async fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), FlashError> {
        self.check_range(offset, bytes.len())?;

        if bytes.is_empty() {
            return Ok(());
        }

        let command = self.command(self.address_width.read_data(), offset);
        self.spi
            .transaction(&mut [Operation::Write(command.bytes()), Operation::Read(bytes)])
            .await
            .map_err(|_| FlashError::Spi)
    }

    fn capacity(&self) -> usize {
        self.size as usize
    }
}

#[allow(
    clippy::arithmetic_side_effects,
    reason = "page/sector math bounded by check_range and modulo results"
)]
impl<SPI: SpiDevice<u8>> NorFlash for W25Q<SPI> {
    const WRITE_SIZE: usize = 1;
    const ERASE_SIZE: usize = SECTOR_SIZE as usize;

    async fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), FlashError> {
        self.check_range(offset, bytes.len())?;

        // Page programs must not cross 256-byte page boundaries, so split the write into per-page
        // chunks.
        let mut address = offset;
        let mut remaining = bytes;
        while !remaining.is_empty() {
            let page_remaining = (PAGE_SIZE - (address % PAGE_SIZE)) as usize;
            let (chunk, rest) = remaining.split_at(remaining.len().min(page_remaining));

            self.write_enable().await?;

            let command = self.command(self.address_width.page_program(), address);
            self.spi
                .transaction(&mut [Operation::Write(command.bytes()), Operation::Write(chunk)])
                .await
                .map_err(|_| FlashError::Spi)?;

            self.wait_idle(Duration::from_micros(100), PAGE_PROGRAM_TIMEOUT).await?;

            address += chunk.len() as u32;
            remaining = rest;
        }

        Ok(())
    }

    async fn erase(&mut self, from: u32, to: u32) -> Result<(), FlashError> {
        if !from.is_multiple_of(SECTOR_SIZE) || !to.is_multiple_of(SECTOR_SIZE) {
            return Err(FlashError::NotAligned);
        }

        if from > to || to > self.size {
            return Err(FlashError::OutOfBounds);
        }

        let mut address = from;
        while address < to {
            self.write_enable().await?;
            let command = self.command(self.address_width.sector_erase(), address);
            self.spi.write(command.bytes()).await.map_err(|_| FlashError::Spi)?;
            self.wait_idle(Duration::from_millis(1), SECTOR_ERASE_TIMEOUT).await?;

            address += SECTOR_SIZE;
        }

        Ok(())
    }
}

// The W25Q allows programming the same bytes repeatedly (only clearing bits), which is what
// sequential-storage needs to invalidate older map items.
impl<SPI: SpiDevice<u8>> MultiwriteNorFlash for W25Q<SPI> {}
