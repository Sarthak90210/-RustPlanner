//! SignedFW: ArduPilot secure boot. Ed25519 keys, a bootloader with the public key
//! written into its key table, and .apj firmware signed in its app descriptor.

use base64::Engine;
use ed25519_dalek::pkcs8::{DecodePrivateKey, EncodePrivateKey};
pub use ed25519_dalek::SigningKey;
use ed25519_dalek::Signer;
use std::io::{Read, Write};

const B64: base64::engine::GeneralPurpose = base64::engine::general_purpose::STANDARD;

/// The three ArduPilot release keys MP always writes ahead of the user's key.
const AP_KEYS: [&str; 3] = [
    "WJbbpbjOz/yMB3JxnvqyTUInCQdZcStkA0qhn2ldhPI=",
    "X8jdVqxIIUmCuMSi8IhTZ40VkXW0gbRczzMtdSghqCI=",
    "snNHkX96F9A+/ISppHZrc1jjPo3jMNN+g2PToKhWSgA=",
];

pub fn generate_key() -> SigningKey {
    SigningKey::generate(&mut rand_core::OsRng)
}

/// "-----BEGIN PRIVATE KEY-----" (PKCS#8), as BouncyCastle's PemWriter writes it.
pub fn private_pem(key: &SigningKey) -> String {
    key.to_pkcs8_pem(Default::default()).map(|s| s.to_string()).unwrap_or_default()
}

pub fn private_dat(key: &SigningKey) -> String {
    format!("PRIVATE_KEYV1:{}", B64.encode(key.to_bytes()))
}

pub fn public_dat(key: &SigningKey) -> String {
    format!("PUBLIC_KEYV1:{}", public_b64(key))
}

pub fn public_b64(key: &SigningKey) -> String {
    B64.encode(key.verifying_key().to_bytes())
}

/// Read a key file: either PRIVATE_KEYV1:<base64 seed> (.dat) or a PKCS#8 PEM.
pub fn load_key(text: &str) -> Result<SigningKey, String> {
    if text.contains("PRIVATE_KEYV1") {
        let b = B64.decode(text.replace("PRIVATE_KEYV1:", "").trim()).map_err(|e| e.to_string())?;
        let seed: [u8; 32] = b.get(..32).and_then(|s| s.try_into().ok()).ok_or("Invalid private key length")?;
        return Ok(SigningKey::from_bytes(&seed));
    }
    SigningKey::from_pkcs8_pem(text).map_err(|e| e.to_string())
}

fn search(hay: &[u8], needle: &[u8]) -> Option<usize> {
    hay.windows(needle.len()).position(|w| w == needle)
}

/// CreateSignedBL: the ArduPilot keys and the user's public key written after the
/// bootloader's key descriptor.
pub fn sign_bootloader(key: &SigningKey, bl: &[u8]) -> Result<Vec<u8>, String> {
    const DESC: [u8; 8] = [0x4e, 0xcf, 0x4e, 0xa5, 0xa6, 0xb6, 0xf7, 0x29];
    let off = search(bl, &DESC).ok_or("Invalid bin, descriptor not found")? + 8;
    let mut out = bl.to_vec();
    let mut keys: Vec<u8> = Vec::new();
    for k in AP_KEYS {
        keys.extend(B64.decode(k).map_err(|e| e.to_string())?);
    }
    keys.extend(key.verifying_key().to_bytes());
    if out.len() < off + keys.len() {
        out.resize(off + keys.len(), 0);
    }
    out[off..off + keys.len()].copy_from_slice(&keys);
    Ok(out)
}

/// CreateSignedAPJ: sign the image (without its 92 byte descriptor) and store the
/// signature in the descriptor; returns the new .apj JSON.
pub fn sign_apj(key: &SigningKey, apj: &str) -> Result<Vec<u8>, String> {
    const DESC: [u8; 8] = [0x41, 0xa3, 0xe5, 0xf2, 0x65, 0x69, 0x92, 0x07];
    const DESC_LEN: usize = 92;
    const SIG_LEN: u32 = 64;
    const SIG_VERSION: u64 = 30437;
    let mut d: serde_json::Value = serde_json::from_str(apj).map_err(|e| e.to_string())?;
    let size = d["image_size"].as_u64().ok_or("image_size missing")? as usize;
    let b = B64.decode(d["image"].as_str().ok_or("image missing")?).map_err(|e| e.to_string())?;
    let mut img = Vec::with_capacity(size);
    flate2::read::ZlibDecoder::new(&b[..]).read_to_end(&mut img).map_err(|e| e.to_string())?;
    img.resize(size, 0);
    let off = search(&img, &DESC).ok_or("Invalid APJ, descriptor not found")? + 8;
    if img.len() < off + DESC_LEN {
        return Err("Invalid APJ, descriptor truncated".into());
    }
    let mut msg = img[..off].to_vec();
    msg.extend_from_slice(&img[off + DESC_LEN..]);
    let sig = key.sign(&msg).to_bytes();
    use sha2::Digest;
    let sha = sha2::Sha512::digest(&msg);
    img[off + 16..off + 20].copy_from_slice(&(SIG_LEN + 8).to_le_bytes());
    img[off + 20..off + 28].copy_from_slice(&SIG_VERSION.to_le_bytes());
    img[off + 28..off + 92].copy_from_slice(&sig);
    let mut z = flate2::write::ZlibEncoder::new(Vec::new(), flate2::Compression::best());
    z.write_all(&img).map_err(|e| e.to_string())?;
    let comp = z.finish().map_err(|e| e.to_string())?;
    d["image"] = B64.encode(comp).into();
    d["image_size"] = img.len().into();
    if let Some(t) = d["flash_total"].as_u64() {
        d["flash_free"] = (t as i64 - img.len() as i64).into();
    }
    d["signed_firmware"] = true.into();
    d["sha"] = B64.encode(sha).into();
    serde_json::to_vec_pretty(&d).map_err(|e| e.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;
    use ed25519_dalek::Verifier;

    #[test]
    fn key_files_round_trip() {
        let k = generate_key();
        assert_eq!(load_key(&private_dat(&k)).unwrap().to_bytes(), k.to_bytes());
        assert_eq!(load_key(&private_pem(&k)).unwrap().to_bytes(), k.to_bytes());
        assert!(public_dat(&k).starts_with("PUBLIC_KEYV1:"));
    }

    #[test]
    fn bootloader_gets_four_keys() {
        let k = generate_key();
        let mut bl = vec![0u8; 64];
        bl.extend([0x4e, 0xcf, 0x4e, 0xa5, 0xa6, 0xb6, 0xf7, 0x29]);
        bl.extend(vec![0u8; 200]);
        let out = sign_bootloader(&k, &bl).unwrap();
        assert_eq!(&out[72 + 96..72 + 128], &k.verifying_key().to_bytes());
        assert_eq!(&out[72..72 + 32], &B64.decode(AP_KEYS[0]).unwrap()[..]);
    }

    #[test]
    fn apj_signature_verifies() {
        let k = generate_key();
        let mut img: Vec<u8> = (0..300u32).map(|i| i as u8).collect();
        img.extend([0x41, 0xa3, 0xe5, 0xf2, 0x65, 0x69, 0x92, 0x07]);
        img.extend(vec![0u8; 92]);
        img.extend(vec![7u8; 100]);
        let mut z = flate2::write::ZlibEncoder::new(Vec::new(), flate2::Compression::default());
        z.write_all(&img).unwrap();
        let apj = serde_json::json!({"image": B64.encode(z.finish().unwrap()), "image_size": img.len(), "flash_total": 2000}).to_string();
        let out: serde_json::Value = serde_json::from_slice(&sign_apj(&k, &apj).unwrap()).unwrap();
        assert_eq!(out["signed_firmware"], true);
        assert_eq!(out["flash_free"], 2000 - img.len() as i64);
        let b = B64.decode(out["image"].as_str().unwrap()).unwrap();
        let mut signed = Vec::new();
        flate2::read::ZlibDecoder::new(&b[..]).read_to_end(&mut signed).unwrap();
        let off = 308;
        let sig = ed25519_dalek::Signature::from_bytes(signed[off + 28..off + 92].try_into().unwrap());
        let mut msg = signed[..off].to_vec();
        msg.extend_from_slice(&signed[off + 92..]);
        k.verifying_key().verify(&msg, &sig).unwrap();
    }
}
