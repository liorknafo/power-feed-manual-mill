//! Settings in the last 4 KB sector of the first 2 MB of the board's QSPI flash (excluded from the program in memory.x).

use embassy_rp::flash::{Blocking, Flash, ERASE_SIZE};
use embassy_rp::peripherals::FLASH;
use feed_core::settings::{Settings, BLOB_LEN};

pub const FLASH_SIZE: usize = 2 * 1024 * 1024;
const OFFSET: u32 = (FLASH_SIZE - ERASE_SIZE) as u32;

pub struct Storage<'d> {
    flash: Flash<'d, FLASH, Blocking, FLASH_SIZE>,
}

impl<'d> Storage<'d> {
    pub fn new(flash: Flash<'d, FLASH, Blocking, FLASH_SIZE>) -> Self {
        Storage { flash }
    }

    pub fn load(&mut self) -> Option<Settings> {
        let mut buf = [0u8; BLOB_LEN];
        self.flash.blocking_read(OFFSET, &mut buf).ok()?;
        Settings::from_bytes(&buf)
    }

    /// Erase + write. Takes tens of ms with interrupts paused: only call while the motor is idle.
    pub fn save(&mut self, s: &Settings) -> bool {
        let mut page = [0xFFu8; 256];
        page[..BLOB_LEN].copy_from_slice(&s.to_bytes());
        self.flash.blocking_erase(OFFSET, OFFSET + ERASE_SIZE as u32).is_ok() && self.flash.blocking_write(OFFSET, &page).is_ok()
    }
}
