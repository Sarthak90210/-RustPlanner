//! Triple DES (EDE) in ECB mode with PKCS#7 padding, standing in for BouncyCastle's
//! `CipherUtilities.GetCipher("DESede/ECB/PKCS7Padding")` with a `DesEdeParameters` key.
//! A 16-byte key is two-key 3DES (K1, K2, K1); a 24-byte key is three-key.

use super::Exception;

const IP: [u8; 64] = [
    58, 50, 42, 34, 26, 18, 10, 2, 60, 52, 44, 36, 28, 20, 12, 4, 62, 54, 46, 38, 30, 22, 14, 6, 64, 56, 48, 40, 32, 24, 16, 8, 57, 49, 41, 33, 25, 17, 9, 1, 59, 51, 43, 35, 27,
    19, 11, 3, 61, 53, 45, 37, 29, 21, 13, 5, 63, 55, 47, 39, 31, 23, 15, 7,
];
const FP: [u8; 64] = [
    40, 8, 48, 16, 56, 24, 64, 32, 39, 7, 47, 15, 55, 23, 63, 31, 38, 6, 46, 14, 54, 22, 62, 30, 37, 5, 45, 13, 53, 21, 61, 29, 36, 4, 44, 12, 52, 20, 60, 28, 35, 3, 43, 11, 51,
    19, 59, 27, 34, 2, 42, 10, 50, 18, 58, 26, 33, 1, 41, 9, 49, 17, 57, 25,
];
const E: [u8; 48] = [
    32, 1, 2, 3, 4, 5, 4, 5, 6, 7, 8, 9, 8, 9, 10, 11, 12, 13, 12, 13, 14, 15, 16, 17, 16, 17, 18, 19, 20, 21, 20, 21, 22, 23, 24, 25, 24, 25, 26, 27, 28, 29, 28, 29, 30, 31, 32, 1,
];
const P: [u8; 32] = [16, 7, 20, 21, 29, 12, 28, 17, 1, 15, 23, 26, 5, 18, 31, 10, 2, 8, 24, 14, 32, 27, 3, 9, 19, 13, 30, 6, 22, 11, 4, 25];
const PC1: [u8; 56] = [
    57, 49, 41, 33, 25, 17, 9, 1, 58, 50, 42, 34, 26, 18, 10, 2, 59, 51, 43, 35, 27, 19, 11, 3, 60, 52, 44, 36, 63, 55, 47, 39, 31, 23, 15, 7, 62, 54, 46, 38, 30, 22, 14, 6, 61,
    53, 45, 37, 29, 21, 13, 5, 28, 20, 12, 4,
];
const PC2: [u8; 48] = [
    14, 17, 11, 24, 1, 5, 3, 28, 15, 6, 21, 10, 23, 19, 12, 4, 26, 8, 16, 7, 27, 20, 13, 2, 41, 52, 31, 37, 47, 55, 30, 40, 51, 45, 33, 48, 44, 49, 39, 56, 34, 53, 46, 42, 50, 36,
    29, 32,
];
const SHIFTS: [u32; 16] = [1, 1, 2, 2, 2, 2, 2, 2, 1, 2, 2, 2, 2, 2, 2, 1];
const S: [[u8; 64]; 8] = [
    [
        14, 4, 13, 1, 2, 15, 11, 8, 3, 10, 6, 12, 5, 9, 0, 7, 0, 15, 7, 4, 14, 2, 13, 1, 10, 6, 12, 11, 9, 5, 3, 8, 4, 1, 14, 8, 13, 6, 2, 11, 15, 12, 9, 7, 3, 10, 5, 0, 15, 12, 8,
        2, 4, 9, 1, 7, 5, 11, 3, 14, 10, 0, 6, 13,
    ],
    [
        15, 1, 8, 14, 6, 11, 3, 4, 9, 7, 2, 13, 12, 0, 5, 10, 3, 13, 4, 7, 15, 2, 8, 14, 12, 0, 1, 10, 6, 9, 11, 5, 0, 14, 7, 11, 10, 4, 13, 1, 5, 8, 12, 6, 9, 3, 2, 15, 13, 8, 10,
        1, 3, 15, 4, 2, 11, 6, 7, 12, 0, 5, 14, 9,
    ],
    [
        10, 0, 9, 14, 6, 3, 15, 5, 1, 13, 12, 7, 11, 4, 2, 8, 13, 7, 0, 9, 3, 4, 6, 10, 2, 8, 5, 14, 12, 11, 15, 1, 13, 6, 4, 9, 8, 15, 3, 0, 11, 1, 2, 12, 5, 10, 14, 7, 1, 10, 13,
        0, 6, 9, 8, 7, 4, 15, 14, 3, 11, 5, 2, 12,
    ],
    [
        7, 13, 14, 3, 0, 6, 9, 10, 1, 2, 8, 5, 11, 12, 4, 15, 13, 8, 11, 5, 6, 15, 0, 3, 4, 7, 2, 12, 1, 10, 14, 9, 10, 6, 9, 0, 12, 11, 7, 13, 15, 1, 3, 14, 5, 2, 8, 4, 3, 15, 0, 6,
        10, 1, 13, 8, 9, 4, 5, 11, 12, 7, 2, 14,
    ],
    [
        2, 12, 4, 1, 7, 10, 11, 6, 8, 5, 3, 15, 13, 0, 14, 9, 14, 11, 2, 12, 4, 7, 13, 1, 5, 0, 15, 10, 3, 9, 8, 6, 4, 2, 1, 11, 10, 13, 7, 8, 15, 9, 12, 5, 6, 3, 0, 14, 11, 8, 12,
        7, 1, 14, 2, 13, 6, 15, 0, 9, 10, 4, 5, 3,
    ],
    [
        12, 1, 10, 15, 9, 2, 6, 8, 0, 13, 3, 4, 14, 7, 5, 11, 10, 15, 4, 2, 7, 12, 9, 5, 6, 1, 13, 14, 0, 11, 3, 8, 9, 14, 15, 5, 2, 8, 12, 3, 7, 0, 4, 10, 1, 13, 11, 6, 4, 3, 2, 12,
        9, 5, 15, 10, 11, 14, 1, 7, 6, 0, 8, 13,
    ],
    [
        4, 11, 2, 14, 15, 0, 8, 13, 3, 12, 9, 7, 5, 10, 6, 1, 13, 0, 11, 7, 4, 9, 1, 10, 14, 3, 5, 12, 2, 15, 8, 6, 1, 4, 11, 13, 12, 3, 7, 14, 10, 15, 6, 8, 0, 5, 9, 2, 6, 11, 13,
        8, 1, 4, 10, 7, 9, 5, 0, 15, 14, 2, 3, 12,
    ],
    [
        13, 2, 8, 4, 6, 15, 11, 1, 10, 9, 3, 14, 5, 0, 12, 7, 1, 15, 13, 8, 10, 3, 7, 4, 12, 5, 6, 11, 0, 14, 9, 2, 7, 11, 4, 1, 9, 12, 14, 2, 0, 6, 10, 13, 15, 3, 5, 8, 2, 1, 14, 7,
        4, 10, 8, 13, 15, 12, 9, 0, 3, 5, 6, 11,
    ],
];

/// Permute the `n_in`-bit value `v` (bit 1 = most significant) by `table`.
fn permute(v: u64, n_in: u32, table: &[u8]) -> u64 {
    let mut r = 0u64;
    for &t in table {
        r = (r << 1) | ((v >> (n_in - t as u32)) & 1);
    }
    r
}

fn subkeys(key: &[u8]) -> [u64; 16] {
    let k = u64::from_be_bytes([key[0], key[1], key[2], key[3], key[4], key[5], key[6], key[7]]);
    let cd = permute(k, 64, &PC1);
    let mut c = (cd >> 28) & 0x0FFF_FFFF;
    let mut d = cd & 0x0FFF_FFFF;
    let mut out = [0u64; 16];
    for (i, s) in SHIFTS.iter().enumerate() {
        c = ((c << s) | (c >> (28 - s))) & 0x0FFF_FFFF;
        d = ((d << s) | (d >> (28 - s))) & 0x0FFF_FFFF;
        out[i] = permute((c << 28) | d, 56, &PC2);
    }
    out
}

fn feistel(r: u32, k: u64) -> u32 {
    let x = permute(r as u64, 32, &E) ^ k;
    let mut out = 0u32;
    for (i, sbox) in S.iter().enumerate() {
        let six = ((x >> (42 - 6 * i)) & 0x3F) as usize;
        let row = ((six & 0x20) >> 4) | (six & 1);
        let col = (six >> 1) & 0xF;
        out = (out << 4) | sbox[row * 16 + col] as u32;
    }
    permute(out as u64, 32, &P) as u32
}

fn des_block(block: u64, keys: &[u64; 16], decrypt: bool) -> u64 {
    let ip = permute(block, 64, &IP);
    let mut l = (ip >> 32) as u32;
    let mut r = ip as u32;
    for i in 0..16 {
        let k = if decrypt { keys[15 - i] } else { keys[i] };
        let t = r;
        r = l ^ feistel(r, k);
        l = t;
    }
    permute(((r as u64) << 32) | l as u64, 64, &FP)
}

fn key_schedule(key: &[u8]) -> Result<[[u64; 16]; 3], Exception> {
    match key.len() {
        16 => Ok([subkeys(&key[0..8]), subkeys(&key[8..16]), subkeys(&key[0..8])]),
        24 => Ok([subkeys(&key[0..8]), subkeys(&key[8..16]), subkeys(&key[16..24])]),
        _ => Err(Exception::Argument("key size must be 16 or 24 bytes.".into())),
    }
}

fn ede(block: u64, ks: &[[u64; 16]; 3], decrypt: bool) -> u64 {
    if decrypt {
        des_block(des_block(des_block(block, &ks[2], true), &ks[1], false), &ks[0], true)
    } else {
        des_block(des_block(des_block(block, &ks[0], false), &ks[1], true), &ks[2], false)
    }
}

/// `cp.Init(true, key); cp.DoFinal(data)`
pub fn encrypt_ecb_pkcs7(key: &[u8], data: &[u8]) -> Result<Vec<u8>, Exception> {
    let ks = key_schedule(key)?;
    let pad = 8 - data.len() % 8;
    let mut buf = data.to_vec();
    buf.extend(std::iter::repeat_n(pad as u8, pad));
    let mut out = Vec::with_capacity(buf.len());
    for c in buf.chunks(8) {
        let b = u64::from_be_bytes([c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7]]);
        out.extend_from_slice(&ede(b, &ks, false).to_be_bytes());
    }
    Ok(out)
}

/// `cp.Init(false, key); cp.DoFinal(data)`: BouncyCastle throws `DataLengthException`
/// ("last block incomplete in decryption") for a partial block and
/// `InvalidCipherTextException` ("pad block corrupted") for bad padding.
pub fn decrypt_ecb_pkcs7(key: &[u8], data: &[u8]) -> Result<Vec<u8>, Exception> {
    let ks = key_schedule(key)?;
    if !data.len().is_multiple_of(8) {
        return Err(Exception::Custom("Org.BouncyCastle.Crypto.DataLengthException", "last block incomplete in decryption".into()));
    }
    let mut out = Vec::with_capacity(data.len());
    for c in data.chunks(8) {
        let b = u64::from_be_bytes([c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7]]);
        out.extend_from_slice(&ede(b, &ks, true).to_be_bytes());
    }
    let corrupted = || Exception::Custom("Org.BouncyCastle.Crypto.InvalidCipherTextException", "pad block corrupted".into());
    let pad = *out.last().ok_or_else(corrupted)? as usize;
    if pad == 0 || pad > 8 || pad > out.len() || out[out.len() - pad..].iter().any(|&b| b as usize != pad) {
        return Err(corrupted());
    }
    out.truncate(out.len() - pad);
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn des_known_answer_and_round_trip() {
        // single DES vector via K1 = K2 = K3: key 133457799BBCDFF1, pt 0123456789ABCDEF
        let k = [0x13, 0x34, 0x57, 0x79, 0x9B, 0xBC, 0xDF, 0xF1];
        let ks = subkeys(&k);
        assert_eq!(des_block(0x0123456789ABCDEF, &ks, false), 0x85E813540F0AB405);
        assert_eq!(des_block(0x85E813540F0AB405, &ks, true), 0x0123456789ABCDEF);
        let key16: Vec<u8> = (1..=16).collect();
        let c = encrypt_ecb_pkcs7(&key16, b"hello world").unwrap();
        assert_eq!(c.len(), 16);
        assert_eq!(decrypt_ecb_pkcs7(&key16, &c).unwrap(), b"hello world");
        assert!(decrypt_ecb_pkcs7(&key16, &c[..15]).is_err());
    }
}
