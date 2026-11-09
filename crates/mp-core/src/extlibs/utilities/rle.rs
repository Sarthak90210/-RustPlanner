//! Port of `ExtLibs/Utilities/RLE.cs`
//!
//! Experimental log compressors: `RLE` (zero-run encoding), `HuffMan` (a Huffman tree over
//! byte frequencies) and `Delta` (byte deltas). The `test()` methods read the author's
//! hard-coded file, as in C#.

use crate::dotnet::collections::Dictionary;
use crate::dotnet::io::{console_write, console_write_line};
use crate::dotnet::json::J;
use crate::dotnet::Exception;

/// `public class RLE`
pub struct RLE;

impl RLE {
    /// `test()`: encodes the author's log file (FileNotFoundException elsewhere).
    pub fn test() -> Result<(), Exception> {
        let path = r"C:\Users\michael\Downloads\2020-05-30 11-31-08.bin";
        let data = std::fs::read(path).map_err(|e| crate::dotnet::io::file_error(path, &e))?;
        let _ = RLE::RunLengthEncode(&data);
        Ok(())
    }

    /// `RunLengthEncode(byte[] s)`: each run boundary writes the byte, and for zero bytes the
    /// run length first; runs are split at 250. The last byte is never written, and run
    /// lengths of non-zero bytes are dropped, as in the C#.
    #[allow(non_snake_case)]
    pub fn RunLengthEncode(s: &[u8]) -> Option<Vec<u8>> {
        let mut srle: Vec<u8> = vec![];
        let mut ccnt: i32 = 1; //char counter
        let n = s.len() as i64;
        let mut i: i64 = 0;
        while i < n - 1 {
            let (a, b) = (s[i as usize], s[i as usize + 1]);
            if a != b {
                //..a break in character repetition or the end of the string
                if a == 0 {
                    srle.extend_from_slice(&[ccnt as u8, a]);
                } else {
                    srle.push(a);
                }
                ccnt = 1; //reset char repetition counter
            } else {
                if ccnt == 250 {
                    srle.extend_from_slice(&[ccnt as u8, a]);
                    ccnt = 1;
                }
                ccnt += 1;
            }
            i += 1;
        }

        Some(srle)
    }
}

/// `HuffMan.Node`
#[allow(non_snake_case)]
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Node {
    pub Symbol: Option<u8>,
    pub Frequency: i32,
    pub Right: Option<Box<Node>>,
    pub Left: Option<Box<Node>>,
}

impl Node {
    /// `Traverse(symbol, data)`: the code for `symbol` below this node, or null.
    #[allow(non_snake_case)]
    pub fn Traverse(&self, symbol: u8, data: Vec<bool>) -> Option<Vec<bool>> {
        // Leaf
        if self.Right.is_none() && self.Left.is_none() {
            // symbol.Equals(this.Symbol): byte.Equals(object) with a boxed byte? (false for null)
            if self.Symbol == Some(symbol) {
                return Some(data);
            }
            return None;
        }
        let mut left = None;
        let mut right = None;

        if let Some(l) = &self.Left {
            let mut left_path_data = data.clone();
            left_path_data.push(false);

            left = l.Traverse(symbol, left_path_data);
        }

        if let Some(r) = &self.Right {
            let mut right_path_data = data.clone();
            right_path_data.push(true);
            right = r.Traverse(symbol, right_path_data);
        }

        if left.is_some() {
            left
        } else {
            right
        }
    }

    /// `ToJSON()` (Newtonsoft): Symbol, Frequency, Right, Left.
    pub fn to_json(&self) -> String {
        self.to_j().to_compact()
    }

    fn to_j(&self) -> J {
        J::Obj(vec![
            ("Symbol".into(), self.Symbol.map(|s| J::Int(s as i64)).unwrap_or(J::Null)),
            ("Frequency".into(), J::Int(self.Frequency as i64)),
            ("Right".into(), self.Right.as_ref().map(|n| n.to_j()).unwrap_or(J::Null)),
            ("Left".into(), self.Left.as_ref().map(|n| n.to_j()).unwrap_or(J::Null)),
        ])
    }

    /// `JsonConvert.DeserializeObject<HuffMan.Node>(text)`
    pub fn from_json(text: &str) -> Result<Option<Node>, Exception> {
        match crate::dotnet::json::parse(text)? {
            None => Ok(None),
            Some(v) => Self::from_v(&v, ""),
        }
    }

    fn from_v(v: &serde_json::Value, path: &str) -> Result<Option<Node>, Exception> {
        if v.is_null() {
            return Ok(None);
        }
        let o = crate::dotnet::json::JsonObj::new(v, "MissionPlanner.Utilities.HuffMan+Node", path)?;
        let symbol = match o.opt_int("Symbol")? {
            None => None,
            Some(s) if (0..=255).contains(&s) => Some(s as u8),
            Some(s) => {
                return Err(Exception::JsonSerialization(format!(
                    "Error converting value {s} to type 'System.Nullable`1[System.Byte]'. Path '{path}.Symbol'."
                )))
            }
        };
        let child = |name: &str| -> Result<Option<Box<Node>>, Exception> {
            match o.get(name) {
                None => Ok(None),
                Some((cv, p)) => Ok(Node::from_v(cv, &p)?.map(Box::new)),
            }
        };
        Ok(Some(Node { Symbol: symbol, Frequency: o.int("Frequency", 0)?, Right: child("Right")?, Left: child("Left")? }))
    }
}

/// `HuffMan.HuffmanTree`
#[allow(non_snake_case)]
#[derive(Debug, Clone, Default)]
pub struct HuffmanTree {
    nodes: Vec<Node>,
    pub Root: Option<Node>,
    pub SymbolFrequencies: Dictionary<u8, i32>,
}

impl HuffmanTree {
    pub fn new() -> Self {
        Self::default()
    }

    /// `BuildHuffmanTree(source)`: counts frequencies, then repeatedly joins the two least
    /// frequent nodes (a stable sort, so ties keep list order). `Root` is only assigned inside
    /// the loop, so a single-symbol source leaves it null.
    #[allow(non_snake_case)]
    pub fn BuildHuffmanTree(&mut self, source: &[u8]) {
        for &b in source {
            if !self.SymbolFrequencies.contains_key(&b) {
                self.SymbolFrequencies.insert(b, 0);
            }
            if let Some(f) = self.SymbolFrequencies.get_mut(&b) {
                *f = f.wrapping_add(1);
            }
        }

        // Step# 1: Create list of nodes with symbol and frequencies
        let freq: Vec<(u8, i32)> = self.SymbolFrequencies.iter().map(|(k, v)| (*k, *v)).collect();
        for (k, v) in freq {
            self.nodes.push(Node { Symbol: Some(k), Frequency: v, Right: None, Left: None });
        }

        // Join the lowest frequencies until only the root is left
        while self.nodes.len() >= 2 {
            // Step# 2: Sort the list of nodes based on its frequencies in ascending order (OrderBy is stable)
            let mut order: Vec<usize> = (0..self.nodes.len()).collect();
            order.sort_by_key(|&i| self.nodes[i].Frequency);

            if order.len() >= 2 {
                // Take first two items
                let (i0, i1) = (order[0], order[1]);
                let t0 = self.nodes[i0].clone();
                let t1 = self.nodes[i1].clone();

                // Create a parent node by combining the frequencies
                let parent = Node {
                    Symbol: None,
                    Frequency: t0.Frequency.wrapping_add(t1.Frequency),
                    Left: Some(Box::new(t0)),
                    Right: Some(Box::new(t1)),
                };
                //Remove left and right nodes and add their parent to the end of nodes list
                let (hi, lo) = if i0 > i1 { (i0, i1) } else { (i1, i0) };
                self.nodes.remove(hi);
                self.nodes.remove(lo);
                self.nodes.push(parent);
            }

            self.Root = self.nodes.first().cloned();
        }
    }

    /// `Encode(source)`: the concatenated codes. A symbol not in the tree makes `Traverse`
    /// return null, and `AddRange(null)` throws ArgumentNullException; a null root throws
    /// NullReferenceException.
    #[allow(non_snake_case)]
    pub fn Encode(&self, source: &[u8]) -> Result<Vec<bool>, Exception> {
        let root = self
            .Root
            .as_ref()
            .ok_or_else(|| Exception::NullReference("Object reference not set to an instance of an object.".into()))?;
        let mut encoded_source = vec![];

        for &b in source {
            let encoded_symbol = root
                .Traverse(b, vec![])
                .ok_or_else(|| Exception::ArgumentNull("Value cannot be null.\r\nParameter name: collection".into()))?;

            encoded_source.extend(encoded_symbol);
        }

        Ok(encoded_source)
    }

    /// `Decode(bits)`: walks the tree; a leaf emits its symbol and restarts at the root.
    #[allow(non_snake_case)]
    pub fn Decode(&self, bits: &[bool]) -> Result<Vec<u8>, Exception> {
        // Start from root of the huffman tree
        let root = self
            .Root
            .as_ref()
            .ok_or_else(|| Exception::NullReference("Object reference not set to an instance of an object.".into()))?;
        let mut current = root;
        let mut decoded = vec![];

        for &bit in bits {
            if bit {
                // If true then move right
                if let Some(r) = &current.Right {
                    current = r.as_ref();
                }
            } else if let Some(l) = &current.Left {
                // If false then move left
                current = l.as_ref();
            }

            // Every leaf node is a symbol so once you reach there then add it to decoded and then reset the current to the root of huffman tree
            if Self::IsLeaf(current) {
                decoded.push(current.Symbol.ok_or_else(|| Exception::InvalidOperation("Nullable object must have a value.".into()))?);
                current = root;
            }
        }

        Ok(decoded)
    }

    /// `IsLeaf(node)`
    #[allow(non_snake_case)]
    pub fn IsLeaf(node: &Node) -> bool {
        node.Left.is_none() && node.Right.is_none()
    }
}

/// `public class HuffMan`
pub struct HuffMan;

impl HuffMan {
    /// `test()`: builds a tree for the author's log file, round-trips it through JSON and
    /// prints every byte's code.
    pub fn test() -> Result<(), Exception> {
        let path = r"C:\Users\michael\Downloads\2020-05-30 11-31-08.bin";
        let data = std::fs::read(path).map_err(|e| crate::dotnet::io::file_error(path, &e))?;

        let mut huff = HuffmanTree::new();
        // Enumerable.Range(0, 257): (byte)256 is 0 again
        for a in 0..257 {
            huff.SymbolFrequencies.insert(a as u8, 0);
        }
        huff.BuildHuffmanTree(&data);

        let tree = huff.Root.as_ref().map(|r| r.to_json()).unwrap_or_else(|| "null".into());
        std::fs::write("huff.json", tree.as_bytes()).map_err(|e| crate::dotnet::io::file_error("huff.json", &e))?;

        let hufftree = crate::dotnet::io::read_all_text("huff.json")?;
        huff.Root = Node::from_json(&hufftree)?;

        let output = huff.Encode(&data)?;

        let _size = output.len() / 8;

        for item in 0..256 {
            let ans = huff.Encode(&[item as u8])?;
            console_write(&format!("{item} {item:X} = "));
            for bit in ans {
                console_write(if bit { "1" } else { "0" });
            }
            console_write_line("");
        }

        let _decode = huff.Decode(&output)?;
        Ok(())
    }
}

/// `public class Delta`
pub struct Delta;

impl Delta {
    /// `delta_encode(ref byte[] buffer)`
    pub fn delta_encode(buffer: &mut [u8]) {
        let mut last: u8 = 0;
        for b in buffer.iter_mut() {
            let current = *b;
            *b = current.wrapping_sub(last);
            last = current;
        }
    }

    /// `delta_decode(ref byte[] buffer)`
    pub fn delta_decode(buffer: &mut [u8]) {
        let mut last: u8 = 0;
        for b in buffer.iter_mut() {
            let delta = *b;
            *b = delta.wrapping_add(last);
            last = *b;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rle_huffman_delta() {
        assert_eq!(RLE::RunLengthEncode(&[0, 0, 0, 5, 5, 7]).unwrap(), vec![3, 0, 5]);
        let data = b"abracadabra";
        let mut h = HuffmanTree::new();
        h.BuildHuffmanTree(data);
        let bits = h.Encode(data).unwrap();
        assert_eq!(h.Decode(&bits).unwrap(), data.to_vec());
        let json = h.Root.as_ref().unwrap().to_json();
        assert_eq!(Node::from_json(&json).unwrap(), h.Root);
        let mut single = HuffmanTree::new();
        single.BuildHuffmanTree(b"aaa");
        assert!(single.Root.is_none());
        let mut d = vec![10u8, 12, 5];
        Delta::delta_encode(&mut d);
        assert_eq!(d, vec![10, 2, 249]);
        Delta::delta_decode(&mut d);
        assert_eq!(d, vec![10, 12, 5]);
    }
}
