//! Port of `ExtLibs/GMap.NET.Core/GMap.NET.Internals/PureImage.cs`
//!
//! `abstract class PureImage` is the `PureImage` trait over a `PureImageFields` block that
//! holds the base class's public fields; the UI library supplies the concrete image type.
//! A C# `PureImage` reference that several holders share (a `Tile`'s overlay array, the map
//! control) is a `PureImageRef`. `abstract class PureImageProxy` is a trait whose
//! `FromArray` is a provided method.

#![allow(non_snake_case)]

use std::any::Any;
use std::sync::Arc;

use parking_lot::Mutex;

use crate::dotnet::{Exception, MemoryStream};

/// The public fields of `PureImage`.
#[derive(Debug, Clone, Default)]
pub struct PureImageFields {
    pub Data: Option<MemoryStream>,
    pub IsParent: bool,
    pub Ix: i64,
    pub Xoff: i64,
    pub Yoff: i64,
}

/// `public abstract class PureImage : IDisposable`: image abstraction
pub trait PureImage: Send + Sync + Any {
    /// the base class's fields
    fn fields(&self) -> &PureImageFields;
    fn fields_mut(&mut self) -> &mut PureImageFields;
    /// `public abstract void Dispose()`
    fn Dispose(&mut self);
    /// lets the UI library get its concrete image type back
    fn as_any(&self) -> &dyn Any;
}

/// A shared `PureImage` reference.
pub type PureImageRef = Arc<Mutex<Box<dyn PureImage>>>;

/// Wraps a freshly made image as a shared reference.
pub fn share(img: Box<dyn PureImage>) -> PureImageRef {
    Arc::new(Mutex::new(img))
}

/// `public abstract class PureImageProxy`: image abstraction proxy
pub trait PureImageProxy: Send + Sync {
    /// `abstract public PureImage FromStream(Stream stream)`
    fn FromStream(&self, stream: &mut MemoryStream) -> Result<Option<Box<dyn PureImage>>, Exception>;
    /// `abstract public bool Save(Stream stream, PureImage image)`
    fn Save(&self, stream: &mut MemoryStream, image: &dyn PureImage) -> Result<bool, Exception>;

    /// `public PureImage FromArray(byte[] data)`: decodes a copy of `data`; on success the
    /// image keeps that stream (rewound) as its `Data`, otherwise the stream is dropped.
    fn FromArray(&self, data: &[u8]) -> Result<Option<Box<dyn PureImage>>, Exception> {
        let mut m = MemoryStream::new(data.to_vec());
        let pi = self.FromStream(&mut m)?;
        match pi {
            Some(mut pi) => {
                m.pos = 0;
                pi.fields_mut().Data = Some(m);
                Ok(Some(pi))
            }
            None => Ok(None),
        }
    }
}

#[cfg(test)]
pub(crate) mod test_image {
    use super::*;

    /// A test image: "decodes" any stream whose first byte is 0x89 (PNG-like).
    #[derive(Debug, Default)]
    pub struct TestImage {
        pub f: PureImageFields,
        pub disposed: bool,
    }

    impl PureImage for TestImage {
        fn fields(&self) -> &PureImageFields {
            &self.f
        }
        fn fields_mut(&mut self) -> &mut PureImageFields {
            &mut self.f
        }
        fn Dispose(&mut self) {
            self.disposed = true;
        }
        fn as_any(&self) -> &dyn Any {
            self
        }
    }

    pub struct TestProxy;

    impl PureImageProxy for TestProxy {
        fn FromStream(&self, stream: &mut MemoryStream) -> Result<Option<Box<dyn PureImage>>, Exception> {
            let mut b = [0u8; 1];
            let n = stream.read_into(&mut b, 0, 1);
            Ok(if n == 1 && b[0] == 0x89 { Some(Box::new(TestImage::default())) } else { None })
        }
        fn Save(&self, stream: &mut MemoryStream, image: &dyn PureImage) -> Result<bool, Exception> {
            if let Some(d) = &image.fields().Data {
                stream.write_from(&d.data, 0, d.data.len());
            }
            Ok(true)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::test_image::*;
    use super::*;

    #[test]
    fn from_array_keeps_rewound_data() {
        let img = TestProxy.FromArray(&[0x89, 1, 2]).unwrap().unwrap();
        let d = img.fields().Data.as_ref().unwrap();
        assert_eq!(d.pos, 0);
        assert_eq!(d.data, vec![0x89, 1, 2]);
        assert!(!img.fields().IsParent);
        assert!(TestProxy.FromArray(&[0, 1]).unwrap().is_none());
        let r = share(img);
        r.lock().Dispose();
        assert!(r.lock().as_any().downcast_ref::<TestImage>().unwrap().disposed);
    }
}
