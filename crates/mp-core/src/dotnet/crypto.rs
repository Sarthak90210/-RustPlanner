//! `System.Security.Cryptography.RijndaelManaged` with the defaults Mission Planner uses:
//! 128-bit blocks (so AES), CBC mode and PKCS7 padding, plus the `CryptoStream` round trip
//! `Crypto.Process` performs.

use super::Exception;

const SBOX: [u8; 256] = [
    0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76, 0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0,
    0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0, 0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15,
    0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75, 0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0,
    0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84, 0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf,
    0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8, 0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5,
    0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2, 0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73,
    0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb, 0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c,
    0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79, 0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08,
    0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a, 0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e,
    0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e, 0xe1, 0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf,
    0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16,
];

fn inv_sbox() -> [u8; 256] {
    let mut inv = [0u8; 256];
    for (i, &s) in SBOX.iter().enumerate() {
        inv[s as usize] = i as u8;
    }
    inv
}

fn xtime(x: u8) -> u8 {
    (x << 1) ^ if x & 0x80 != 0 { 0x1b } else { 0 }
}

fn mul(mut a: u8, mut b: u8) -> u8 {
    let mut r = 0u8;
    while b != 0 {
        if b & 1 != 0 {
            r ^= a;
        }
        a = xtime(a);
        b >>= 1;
    }
    r
}

/// AES with a 128-, 192- or 256-bit key.
struct Aes {
    round_keys: Vec<[u8; 16]>,
}

impl Aes {
    fn new(key: &[u8]) -> Result<Self, Exception> {
        let nk = key.len() / 4;
        if !matches!(key.len(), 16 | 24 | 32) {
            return Err(Exception::Cryptographic("Specified key is not a valid size for this algorithm.".into()));
        }
        let nr = nk + 6;
        let total = 4 * (nr + 1);
        let mut w: Vec<[u8; 4]> = Vec::with_capacity(total);
        for i in 0..nk {
            w.push([key[4 * i], key[4 * i + 1], key[4 * i + 2], key[4 * i + 3]]);
        }
        let mut rcon = 1u8;
        for i in nk..total {
            let mut t = w[i - 1];
            if i % nk == 0 {
                t = [SBOX[t[1] as usize] ^ rcon, SBOX[t[2] as usize], SBOX[t[3] as usize], SBOX[t[0] as usize]];
                rcon = xtime(rcon);
            } else if nk > 6 && i % nk == 4 {
                t = [SBOX[t[0] as usize], SBOX[t[1] as usize], SBOX[t[2] as usize], SBOX[t[3] as usize]];
            }
            let p = w[i - nk];
            w.push([p[0] ^ t[0], p[1] ^ t[1], p[2] ^ t[2], p[3] ^ t[3]]);
        }
        let round_keys = (0..=nr)
            .map(|r| {
                let mut k = [0u8; 16];
                for c in 0..4 {
                    k[4 * c..4 * c + 4].copy_from_slice(&w[4 * r + c]);
                }
                k
            })
            .collect();
        Ok(Aes { round_keys })
    }

    fn encrypt_block(&self, b: &mut [u8; 16]) {
        let nr = self.round_keys.len() - 1;
        add(b, &self.round_keys[0]);
        for r in 1..=nr {
            for x in b.iter_mut() {
                *x = SBOX[*x as usize];
            }
            shift_rows(b);
            if r != nr {
                mix_columns(b);
            }
            add(b, &self.round_keys[r]);
        }
    }

    fn decrypt_block(&self, b: &mut [u8; 16], inv: &[u8; 256]) {
        let nr = self.round_keys.len() - 1;
        add(b, &self.round_keys[nr]);
        for r in (0..nr).rev() {
            inv_shift_rows(b);
            for x in b.iter_mut() {
                *x = inv[*x as usize];
            }
            add(b, &self.round_keys[r]);
            if r != 0 {
                inv_mix_columns(b);
            }
        }
    }
}

fn add(b: &mut [u8; 16], k: &[u8; 16]) {
    for i in 0..16 {
        b[i] ^= k[i];
    }
}

fn shift_rows(b: &mut [u8; 16]) {
    let s = *b;
    for c in 0..4 {
        for r in 0..4 {
            b[4 * c + r] = s[4 * ((c + r) % 4) + r];
        }
    }
}

fn inv_shift_rows(b: &mut [u8; 16]) {
    let s = *b;
    for c in 0..4 {
        for r in 0..4 {
            b[4 * ((c + r) % 4) + r] = s[4 * c + r];
        }
    }
}

fn mix_columns(b: &mut [u8; 16]) {
    for c in 0..4 {
        let a = [b[4 * c], b[4 * c + 1], b[4 * c + 2], b[4 * c + 3]];
        b[4 * c] = mul(a[0], 2) ^ mul(a[1], 3) ^ a[2] ^ a[3];
        b[4 * c + 1] = a[0] ^ mul(a[1], 2) ^ mul(a[2], 3) ^ a[3];
        b[4 * c + 2] = a[0] ^ a[1] ^ mul(a[2], 2) ^ mul(a[3], 3);
        b[4 * c + 3] = mul(a[0], 3) ^ a[1] ^ a[2] ^ mul(a[3], 2);
    }
}

fn inv_mix_columns(b: &mut [u8; 16]) {
    for c in 0..4 {
        let a = [b[4 * c], b[4 * c + 1], b[4 * c + 2], b[4 * c + 3]];
        b[4 * c] = mul(a[0], 14) ^ mul(a[1], 11) ^ mul(a[2], 13) ^ mul(a[3], 9);
        b[4 * c + 1] = mul(a[0], 9) ^ mul(a[1], 14) ^ mul(a[2], 11) ^ mul(a[3], 13);
        b[4 * c + 2] = mul(a[0], 13) ^ mul(a[1], 9) ^ mul(a[2], 14) ^ mul(a[3], 11);
        b[4 * c + 3] = mul(a[0], 11) ^ mul(a[1], 13) ^ mul(a[2], 9) ^ mul(a[3], 14);
    }
}

/// `RijndaelManaged` (BlockSize 128, CBC, PKCS7).
#[derive(Debug, Clone)]
pub struct RijndaelManaged {
    pub key: Vec<u8>,
    pub iv: Vec<u8>,
}

impl RijndaelManaged {
    /// `BlockSize` in bits.
    pub const BLOCK_SIZE: i32 = 128;

    pub fn new(key: &[u8], iv: &[u8]) -> Self {
        RijndaelManaged { key: key.to_vec(), iv: iv.to_vec() }
    }

    fn checked_iv(&self) -> Result<[u8; 16], Exception> {
        if self.iv.len() != 16 {
            return Err(Exception::Cryptographic("Specified initialization vector (IV) does not match the block size for this algorithm.".into()));
        }
        let mut iv = [0u8; 16];
        iv.copy_from_slice(&self.iv);
        Ok(iv)
    }

    /// `CreateEncryptor()` + `CryptoStream.Write` + `FlushFinalBlock`.
    pub fn encrypt(&self, data: &[u8]) -> Result<Vec<u8>, Exception> {
        let aes = Aes::new(&self.key)?;
        let mut prev = self.checked_iv()?;
        let pad = 16 - data.len() % 16;
        let mut buf = data.to_vec();
        buf.extend(std::iter::repeat(pad as u8).take(pad));
        let mut out = Vec::with_capacity(buf.len());
        for chunk in buf.chunks(16) {
            let mut b = [0u8; 16];
            for i in 0..16 {
                b[i] = chunk[i] ^ prev[i];
            }
            aes.encrypt_block(&mut b);
            out.extend_from_slice(&b);
            prev = b;
        }
        Ok(out)
    }

    /// `CreateDecryptor()` + `CryptoStream` read to the end: "Padding is invalid and cannot be
    /// removed." or "Length of the data to decrypt is invalid." on bad input.
    pub fn decrypt(&self, data: &[u8]) -> Result<Vec<u8>, Exception> {
        let aes = Aes::new(&self.key)?;
        let mut prev = self.checked_iv()?;
        if data.len() % 16 != 0 || data.is_empty() {
            return Err(Exception::Cryptographic("Length of the data to decrypt is invalid.".into()));
        }
        let inv = inv_sbox();
        let mut out = Vec::with_capacity(data.len());
        for chunk in data.chunks(16) {
            let mut b = [0u8; 16];
            b.copy_from_slice(chunk);
            let c = b;
            aes.decrypt_block(&mut b, &inv);
            for i in 0..16 {
                b[i] ^= prev[i];
            }
            out.extend_from_slice(&b);
            prev = c;
        }
        let pad = *out.last().unwrap() as usize;
        if pad == 0 || pad > 16 || out[out.len() - pad..].iter().any(|&x| x as usize != pad) {
            return Err(Exception::Cryptographic("Padding is invalid and cannot be removed.".into()));
        }
        out.truncate(out.len() - pad);
        Ok(out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fips197_aes256_vector() {
        // FIPS-197 C.3
        let key: Vec<u8> = (0u8..32).collect();
        let aes = Aes::new(&key).unwrap();
        let mut b = [0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff];
        aes.encrypt_block(&mut b);
        assert_eq!(b, [0x8e, 0xa2, 0xb7, 0xca, 0x51, 0x67, 0x45, 0xbf, 0xea, 0xfc, 0x49, 0x90, 0x4b, 0x49, 0x60, 0x89]);
        aes.decrypt_block(&mut b, &inv_sbox());
        assert_eq!(b[0], 0x00);
        assert_eq!(b[15], 0xff);
    }

    #[test]
    fn cbc_pkcs7_round_trip() {
        let r = RijndaelManaged::new(&[7u8; 32], &[3u8; 16]);
        for n in [0usize, 1, 15, 16, 17, 100] {
            let data: Vec<u8> = (0..n as u8).collect();
            let e = r.encrypt(&data).unwrap();
            assert_eq!(e.len(), (n / 16 + 1) * 16);
            assert_eq!(r.decrypt(&e).unwrap(), data);
        }
        assert!(r.decrypt(&[0u8; 15]).is_err());
    }
}
