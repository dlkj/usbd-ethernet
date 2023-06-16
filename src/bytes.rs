// hat tip to tokio-rs/bytes
pub(crate) trait BufMut {
    fn remaining_mut(&self) -> usize;
    fn has_remaining_mut(&self) -> bool {
        self.remaining_mut() > 0
    }

    fn put_slice(&mut self, src: &[u8]);

    fn put_u8(&mut self, n: u8) {
        let src = [n];
        self.put_slice(&src);
    }

    fn put_u16_le(&mut self, n: u16) {
        self.put_slice(&n.to_le_bytes());
    }

    fn put_u32_le(&mut self, n: u32) {
        self.put_slice(&n.to_le_bytes());
    }
}

pub(crate) trait Buf {
    fn remaining(&self) -> usize;
    fn has_remaining(&self) -> bool {
        self.remaining() > 0
    }

    fn chunk(&self) -> &[u8];

    fn advance(&mut self, cnt: usize);

    fn get_slice(&mut self, size: usize) -> &[u8];

    fn get_u8(&mut self) -> Option<u8> {
        const SIZE: usize = core::mem::size_of::<u8>();
        let value = self.chunk().first().copied()?;
        self.advance(SIZE);
        Some(value)
    }

    fn get_u16_le(&mut self) -> Option<u16> {
        const SIZE: usize = core::mem::size_of::<u16>();
        let int_bytes = self.chunk().get(..SIZE)?;
        // panic safety, will never fail
        let value = u16::from_le_bytes(int_bytes.try_into().unwrap());
        self.advance(SIZE);
        Some(value)
    }

    fn get_u32_le(&mut self) -> Option<u32> {
        const SIZE: usize = core::mem::size_of::<u32>();
        let int_bytes = self.chunk().get(..SIZE)?;
        // panic safety, will never fail
        let value = u32::from_le_bytes(int_bytes.try_into().unwrap());
        self.advance(SIZE);
        Some(value)
    }
}

impl BufMut for &mut [u8] {
    fn remaining_mut(&self) -> usize {
        self.len()
    }

    fn put_slice(&mut self, src: &[u8]) {
        self[..src.len()].copy_from_slice(src);
        // use mem::take to make lifetimes happy
        let (_, b) = core::mem::take(self).split_at_mut(src.len());
        *self = b;
    }
}

impl Buf for &[u8] {
    fn remaining(&self) -> usize {
        self.len()
    }

    fn chunk(&self) -> &[u8] {
        self
    }

    fn advance(&mut self, cnt: usize) {
        *self = &self[cnt..];
    }

    fn get_slice(&mut self, size: usize) -> &[u8] {
        let (a, b) = self.split_at(size);
        *self = b;
        a
    }
}
