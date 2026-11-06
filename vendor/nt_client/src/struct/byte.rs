//! Byte buffer reading/writing to pack/unpack NetworkTables `struct` types.

mod private {
    pub trait Sealed {}
}

macro_rules! write {
    ($($(#[$m: meta])* $vis: vis fn $ident: ident => $ty: ty ;)*) => {
        $(
        $(#[$m])*
        #[doc = concat!("Writes an [`", stringify!($ty), "`] to the underlying buffer.")]
        ///
        /// # Panics
        /// Panics if the buffer size exceeds [`isize::MAX`].
        $vis fn $ident(&mut self, val: $ty) {
            self.buf.extend(&val.to_le_bytes());
        }
        )*
    };
}

macro_rules! read {
    ($($(#[$m: meta])* $vis: vis fn $ident: ident => $ty: ty ;)*) => {
        $(
        $(#[$m])*
        #[doc = concat!("Reads an [`", stringify!($ty), "`] at the current cursor position from the underlying view.")]
        #[doc = concat!("After execution, the cursor will be incremented by `mem::size_of::<", stringify!($ty), ">()` bytes.")]
        $vis fn $ident(&mut self) -> Option<$ty> {
            let size = std::mem::size_of::<$ty>();
            let at = self.cursor.checked_add(size)?;
            if at > self.view.len() {
                self.cursor = self.view.len();
                return None;
            }
            let value = <$ty>::from_le_bytes(self.view[self.cursor..at].try_into().expect("byte size correct"));
            self.cursor = at;
            Some(value)
        }
        )*
    };
}

macro_rules! primitives {
    ($($ty: ty as $w: ident, $r: ident;)*) => {
        $(
        impl private::Sealed for $ty {}
        impl BytePrimitive for $ty {
            fn write(self, buf: &mut ByteBuffer) {
                buf.$w(self);
            }
            fn read(read: &mut ByteReader) -> Option<Self> {
                read.$r()
            }
        }
        impl<const N: usize> private::Sealed for [$ty; N] {}
        impl<const N: usize> BytePrimitive for [$ty; N] {
            fn write(self, buf: &mut ByteBuffer) {
                for item in self {
                    buf.$w(item)
                }
            }
            fn read(read: &mut ByteReader) -> Option<Self> {
                let mut arr = [Default::default(); N];
                for elem in &mut arr {
                    *elem = read.$r()?;
                }
                Some(arr)
            }
        }
        )*
    };
}

/// Byte buffer used for packing NetworkTables `struct` data.
///
/// This is meant to model the `ByteBuffer` class in Java, and, as such, does not support unsigned
/// integer types as to keep parity between the two languages.
///
/// Byte order is in little-endian.
///
/// # Examples
///
/// ```
/// use nt_client::r#struct::byte::ByteBuffer;
///
/// // create a new, empty byte buffer
/// let mut buf = ByteBuffer::new();
/// // write a 100i32
/// buf.write_i32(100);
/// // write a 38.2282 as an f64
/// buf.write_f64(38.2282);
///
/// // read from the buffer
/// // reads are done from the beginning of the buffer and move
/// // positively through the buffer
/// let mut reader = buf.read();
/// assert_eq!(reader.read_i32(), Some(100));
/// assert_eq!(reader.read_f64(), Some(38.2282));
/// assert_eq!(reader.read_i8(), None);
/// ```
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ByteBuffer {
    buf: Vec<u8>,
}

impl Default for ByteBuffer {
    fn default() -> Self {
        Self::new()
    }
}

impl From<Vec<u8>> for ByteBuffer {
    fn from(value: Vec<u8>) -> Self {
        Self { buf: value }
    }
}

impl From<ByteBuffer> for Vec<u8> {
    fn from(value: ByteBuffer) -> Self {
        value.buf
    }
}

impl<const S: usize> From<[u8; S]> for ByteBuffer {
    fn from(value: [u8; S]) -> Self {
        Self { buf: value.into() }
    }
}

impl ByteBuffer {
    /// Creates a new, empty `ByteBuffer`.
    pub fn new() -> Self {
        Self {
            buf: Vec::new(),
        }
    }

    /// Returns the size in bytes of the underlying buffer.
    pub fn len(&self) -> usize {
        self.buf.len()
    }

    /// Returns `true` if the underlying buffer has no data.
    pub fn is_empty(&self) -> bool {
        self.buf.is_empty()
    }

    /// Returns a `ByteReader` that will read from this buffer.
    pub fn read<'a>(&'a self) -> ByteReader<'a> {
        ByteReader::new(&self.buf)
    }

    write! {
        pub fn write_i8 => i8;
        pub fn write_i16 => i16;
        pub fn write_i32 => i32;
        pub fn write_i64 => i64;
        pub fn write_isize => isize;

        pub fn write_f32 => f32;
        pub fn write_f64 => f64;
    }
}

/// Byte reader used for unpacking NetworkTables `struct` data.
///
/// This is meant to model the `ByteBuffer` class in Java, and, as such, does not support unsigned
/// integer types as to keep parity between the two languages.
///
/// Byte order is in little-endian.
///
/// # Examples
///
/// ```
/// use nt_client::r#struct::byte::ByteReader;
///
/// // create a backing buffer of binary data
/// let vec = vec![0x05, 0x77, 0xEC, 0xFF, 0xFF, 0x29, 0x5C, 0xE1, 0x41];
/// // reads are immutable and start from the beginning of the view
/// let mut reader = ByteReader::new(&vec);
/// assert_eq!(reader.read_i8(), Some(5));
/// // each read increments the internal cursor, allowing sequential read calls
/// assert_eq!(reader.read_i32(), Some(-5001));
/// assert_eq!(reader.read_f32(), Some(28.17));
/// assert_eq!(reader.read_f64(), None);
/// ```
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ByteReader<'a> {
    view: &'a [u8],
    cursor: usize,
}

impl<'a> ByteReader<'a> {
    /// Creates a new `ByteReader` that reads the bytes of `view`, starting from the front.
    pub fn new(view: &'a [u8]) -> Self {
        Self {
            view,
            cursor: 0,
        }
    }

    /// Returns the current cursor position.
    pub fn cursor(&self) -> usize {
        self.cursor
    }

    /// Returns a mutable reference to the current cursor position.
    pub fn cursor_mut(&mut self) -> &mut usize {
        &mut self.cursor
    }

    read! {
        pub fn read_i8 => i8;
        pub fn read_i16 => i16;
        pub fn read_i32 => i32;
        pub fn read_i64 => i64;
        pub fn read_isize => isize;

        pub fn read_f32 => f32;
        pub fn read_f64 => f64;
    }
}

/// Primitive data that can be directly written and read from a [`ByteBuffer`] and [`ByteReader`].
///
/// This trait is mostly used by the `nt_client_macros` crate.
///
/// This trait is **sealed** and cannot be implemented by downstream crates.
pub trait BytePrimitive: private::Sealed {
    /// Writes this primitive to a byte buffer.
    fn write(self, buf: &mut ByteBuffer);

    /// Reads this primitive from a byte reader.
    fn read(read: &mut ByteReader) -> Option<Self> where Self: Sized;
}

primitives! {
    i8 as write_i8, read_i8;
    i16 as write_i16, read_i16;
    i32 as write_i32, read_i32;
    i64 as write_i64, read_i64;
    isize as write_isize, read_isize;
    f32 as write_f32, read_f32;
    f64 as write_f64, read_f64;
}

#[cfg(test)]
mod tests {
    use super::ByteBuffer;

    #[test]
    fn test_read_write() {
        let mut buf = ByteBuffer::new();

        buf.write_i8(28);
        buf.write_f32(-13.72);
        buf.write_i64(173_283_012_736);
        assert_eq!(buf.len(), 13);

        let mut read = buf.read();
        assert_eq!(read.read_i8(), Some(28));
        assert_eq!(read.read_f32(), Some(-13.72));
        assert_eq!(read.read_i64(), Some(173_283_012_736));
        assert_eq!(read.cursor(), buf.len());

        assert_eq!(read.read_i8(), None);
    }
}

