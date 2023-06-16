use defmt::error;
// TODO remove usb stuff from this mod
use usb_device::Result;
use usb_device::UsbError;

// TODO add the ability to freeze the buffer to make read and write exclusive
pub(crate) struct RWBuffer<'a, const LEN: usize> {
    store: &'a mut [u8; LEN],
    read_ptr: usize,
    write_ptr: usize,
}
impl<'a, const LEN: usize> RWBuffer<'a, LEN> {
    pub fn new(store: &'a mut [u8; LEN]) -> Self {
        Self {
            store,
            read_ptr: Default::default(),
            write_ptr: Default::default(),
        }
    }

    pub const fn capacity(&self) -> usize {
        self.store.len()
    }

    pub fn is_empty(&self) -> bool {
        self.write_ptr == 0
    }

    pub fn is_full(&self) -> bool {
        self.write_ptr == self.capacity()
    }

    pub fn has_unread(&self) -> bool {
        self.unread() > 0
    }

    pub fn unread(&self) -> usize {
        assert!(self.read_ptr <= self.write_ptr);
        self.write_ptr - self.read_ptr
    }

    pub fn write<R>(
        &mut self,
        len: usize,
        f: impl FnOnce(&mut [u8]) -> Result<(usize, R)>,
    ) -> Result<(usize, R)> {
        let Some(buf) = self.store.get_mut(self.write_ptr .. self.write_ptr + len)
        else{
            error!("buffer: tried to write more data than capacity");
            return Err(UsbError::BufferOverflow);
        };
        let (written, r) = f(buf)?;
        if written > len {
            error!("buffer: claim to have written more data than allocated");
            return Err(UsbError::BufferOverflow);
        }
        self.write_ptr += written;
        Ok((written, r))
    }

    pub fn clear(&mut self) {
        self.read_ptr = 0;
        self.write_ptr = 0;
    }

    pub fn read<R>(
        &mut self,
        len: usize,
        f: impl FnOnce(&mut [u8]) -> Result<(usize, R)>,
    ) -> Result<(usize, R)> {
        let Some(buf) = self
            .store
            .get_mut(self.read_ptr..self.read_ptr + len)
            else
            {
                error!("buffer: tried to read more data than available");
                return Err(UsbError::InvalidState);
            };

        let (read, r) = f(buf)?;
        if read > len {
            error!("buffer: claim to have read more data than allocated");
            return Err(UsbError::BufferOverflow);
        }
        self.read_ptr += read;
        Ok((read, r))
    }
}
