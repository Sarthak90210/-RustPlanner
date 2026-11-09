//! Port of `ExtLibs/Utilities/Crypto.cs`
//!
//! A `RijndaelManaged` (AES-256, CBC, PKCS7) whose key and IV are fixed byte arrays with the
//! first network interface's MAC address written over their start, so files it encrypts only
//! decrypt on the same machine. As in C#, the MAC is copied into the shared static arrays.

use std::sync::Mutex;

use base64::Engine;

use crate::dotnet::crypto::RijndaelManaged;
use crate::dotnet::environment::first_interface_physical_address;
use crate::dotnet::Exception;

static KEY: Mutex<[u8; 32]> = Mutex::new([
    0xd1, 0x3c, 0x35, 0x6f, 0xb5, 0xd, 0x87, 0xf0, 0x92, 0x07, 0x6d, 0xab, 0x76, 0x82, 0x36, 0xa, 0x13, 0x5a, 0x77, 0xfe, 0x77, 0xf3, 0x7f, 0xa8, 0xa4,
    0x04, 0x11, 0x46, 0x68, 0x2d, 0x48, 0xa1,
]);

static IV: Mutex<[u8; 16]> = Mutex::new([0x6d, 0x2d, 0xf5, 0x34, 0xc7, 0x60, 0xc5, 0x33, 0xe2, 0xa3, 0xd7, 0xc3, 0xf3, 0x39, 0xf2, 0x16]);

/// `public sealed class Crypto : IDisposable`
#[derive(Debug, Clone)]
pub struct Crypto {
    /// `public SymmetricAlgorithm algorithm`
    pub algorithm: RijndaelManaged,
}

impl Default for Crypto {
    fn default() -> Self {
        Self::new()
    }
}

impl Crypto {
    /// `Crypto()`: the MAC of the first network interface over the start of the key and IV
    /// (any failure leaves them as they are).
    pub fn new() -> Self {
        let mut key = KEY.lock().unwrap_or_else(|e| e.into_inner());
        let mut iv = IV.lock().unwrap_or_else(|e| e.into_inner());
        if let Some(bytes) = first_interface_physical_address() {
            // Array.Copy(bytes, IV, bytes.Length): throws (and is swallowed) past the array end
            if bytes.len() <= iv.len() {
                iv[..bytes.len()].copy_from_slice(&bytes);
                if bytes.len() <= key.len() {
                    key[..bytes.len()].copy_from_slice(&bytes);
                }
            }
        }
        Crypto { algorithm: RijndaelManaged::new(&key[..], &iv[..]) }
    }

    /// `Dispose()`: `algorithm.Clear()`.
    pub fn dispose(&mut self) {
        self.algorithm.key.iter_mut().for_each(|b| *b = 0);
        self.algorithm.iv.iter_mut().for_each(|b| *b = 0);
    }

    /// `SetBinaryKeys(Key, IV)`
    pub fn set_binary_keys(&mut self, key: &[u8], iv: &[u8]) {
        self.algorithm.key = key.to_vec();
        self.algorithm.iv = iv.to_vec();
    }

    /// `ExtractBinaryKeys(out Key, out IV)`
    pub fn extract_binary_keys(&self) -> (Vec<u8>, Vec<u8>) {
        (self.algorithm.key.clone(), self.algorithm.iv.clone())
    }

    /// `EncryptBuffer(cleanBuffer)`
    pub fn encrypt_buffer(&self, clean_buffer: &[u8]) -> Result<Vec<u8>, Exception> {
        self.algorithm.encrypt(clean_buffer)
    }

    /// `DecryptBuffer(cryptoBuffer)`
    pub fn decrypt_buffer(&self, crypto_buffer: &[u8]) -> Result<Vec<u8>, Exception> {
        self.algorithm.decrypt(crypto_buffer)
    }

    /// `EncryptString(plainText)`: UTF-8, encrypted, base64.
    pub fn encrypt_string(&self, plain_text: &str) -> Result<String, Exception> {
        Ok(base64::engine::general_purpose::STANDARD.encode(self.encrypt_buffer(plain_text.as_bytes())?))
    }

    /// `DecryptString(encyptedText)`
    pub fn decrypt_string(&self, encypted_text: &str) -> Result<String, Exception> {
        let data = base64::engine::general_purpose::STANDARD.decode(encypted_text.trim()).map_err(|_| {
            Exception::Format("The input is not a valid Base-64 string as it contains a non-base 64 character, more than two padding characters, or an illegal character among the padding characters.".into())
        })?;
        Ok(String::from_utf8_lossy(&self.decrypt_buffer(&data)?).into_owned())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn string_round_trip() {
        let c = Crypto::new();
        let e = c.encrypt_string("secret ✓").unwrap();
        assert_eq!(c.decrypt_string(&e).unwrap(), "secret ✓");
        assert!(c.decrypt_string("!!").is_err());
        assert_eq!(c.algorithm.key.len(), 32);
        assert_eq!(c.algorithm.iv.len(), 16);
    }
}
