//! `System.Xml.XmlTextReader` / `XmlTextWriter`, reduced to what the ported code reads and
//! writes: a sequential node stream (so a malformed document still yields the nodes before
//! the error, as the C# loops see them) and indented element output.

use super::Exception;

/// One node as `XmlTextReader.Read()` reports it (`WhitespaceHandling.All`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum XmlNode {
    /// `XmlNodeType.Element`; `empty` for `<a/>`.
    Element { name: String, empty: bool },
    /// `Text`, `CDATA` or `Whitespace`.
    Text(String),
    EndElement(String),
    /// Declarations, comments, processing instructions, DOCTYPE.
    Other,
}

fn xml_error(msg: &str, line: usize, pos: usize) -> Exception {
    Exception::Xml(format!("{msg} Line {line}, position {pos}."))
}

fn decode_entities(s: &str, line: usize) -> Result<String, Exception> {
    let mut out = String::with_capacity(s.len());
    let mut rest = s;
    while let Some(i) = rest.find('&') {
        out.push_str(&rest[..i]);
        let after = &rest[i + 1..];
        let Some(end) = after.find(';') else {
            return Err(xml_error("An error occurred while parsing EntityName.", line, 1));
        };
        let ent = &after[..end];
        let ch = match ent {
            "lt" => '<',
            "gt" => '>',
            "amp" => '&',
            "quot" => '"',
            "apos" => '\'',
            _ if ent.starts_with("#x") => char::from_u32(u32::from_str_radix(&ent[2..], 16).map_err(|_| xml_error("Invalid character reference.", line, 1))?)
                .ok_or_else(|| xml_error("Invalid character reference.", line, 1))?,
            _ if ent.starts_with('#') => char::from_u32(ent[1..].parse::<u32>().map_err(|_| xml_error("Invalid character reference.", line, 1))?)
                .ok_or_else(|| xml_error("Invalid character reference.", line, 1))?,
            _ => return Err(xml_error(&format!("Reference to undeclared entity '{ent}'."), line, 1)),
        };
        out.push(ch);
        rest = &after[end + 1..];
    }
    out.push_str(rest);
    Ok(out)
}

/// The node stream of `text`, and the exception `Read()` throws where the document stops
/// being well formed (if it does).
pub fn read_nodes(text: &str) -> (Vec<XmlNode>, Option<Exception>) {
    let text = text.strip_prefix('\u{feff}').unwrap_or(text);
    let mut nodes = vec![];
    let mut stack: Vec<String> = vec![];
    let mut seen_root = false;
    let mut i = 0usize;
    let b = text.as_bytes();
    let line_of = |at: usize| text[..at.min(text.len())].matches('\n').count() + 1;
    while i < b.len() {
        if b[i] != b'<' {
            let end = text[i..].find('<').map(|e| i + e).unwrap_or(b.len());
            let raw = &text[i..end];
            if stack.is_empty() && !raw.trim().is_empty() {
                return (nodes, Some(xml_error("Data at the root level is invalid.", line_of(i), 1)));
            }
            match decode_entities(raw, line_of(i)) {
                Ok(t) => nodes.push(XmlNode::Text(t)),
                Err(e) => return (nodes, Some(e)),
            }
            i = end;
            continue;
        }
        let rest = &text[i..];
        if rest.starts_with("<?") {
            match rest.find("?>") {
                Some(e) => {
                    nodes.push(XmlNode::Other);
                    i += e + 2;
                }
                None => return (nodes, Some(xml_error("Unexpected end of file.", line_of(i), 1))),
            }
        } else if rest.starts_with("<!--") {
            match rest.find("-->") {
                Some(e) => {
                    nodes.push(XmlNode::Other);
                    i += e + 3;
                }
                None => return (nodes, Some(xml_error("Unexpected end of file while parsing Comment.", line_of(i), 1))),
            }
        } else if rest.starts_with("<![CDATA[") {
            match rest.find("]]>") {
                Some(e) => {
                    nodes.push(XmlNode::Text(rest[9..e].to_string()));
                    i += e + 3;
                }
                None => return (nodes, Some(xml_error("Unexpected end of file while parsing CDATA.", line_of(i), 1))),
            }
        } else if rest.starts_with("<!") {
            match rest.find('>') {
                Some(e) => {
                    nodes.push(XmlNode::Other);
                    i += e + 1;
                }
                None => return (nodes, Some(xml_error("Unexpected end of file.", line_of(i), 1))),
            }
        } else if let Some(r) = rest.strip_prefix("</") {
            let Some(e) = r.find('>') else {
                return (nodes, Some(xml_error("Unexpected end of file.", line_of(i), 1)));
            };
            let name = r[..e].trim().to_string();
            match stack.pop() {
                Some(open) if open == name => nodes.push(XmlNode::EndElement(name)),
                Some(open) => {
                    return (nodes, Some(xml_error(&format!("The '{open}' start tag does not match the end tag of '{name}'."), line_of(i), 1)))
                }
                None => return (nodes, Some(xml_error("Unexpected end tag.", line_of(i), 1))),
            }
            i += 2 + e + 1;
        } else {
            // start tag: name, attributes (quoted values may contain '>'), optional '/'
            let mut j = 1;
            let mut quote: Option<u8> = None;
            while i + j < b.len() {
                let c = b[i + j];
                match quote {
                    Some(q) if c == q => quote = None,
                    Some(_) => {}
                    None if c == b'"' || c == b'\'' => quote = Some(c),
                    None if c == b'>' => break,
                    None => {}
                }
                j += 1;
            }
            if i + j >= b.len() {
                return (nodes, Some(xml_error("Unexpected end of file.", line_of(i), 1)));
            }
            let inner = &text[i + 1..i + j];
            let empty = inner.ends_with('/');
            let inner = inner.trim_end_matches('/');
            let name: String = inner.split(|c: char| c.is_whitespace()).next().unwrap_or("").to_string();
            if name.is_empty() || name.starts_with(|c: char| c.is_ascii_digit() || c == '-' || c == '.') {
                return (nodes, Some(xml_error("Name cannot begin with this character.", line_of(i), 2)));
            }
            if stack.is_empty() {
                if seen_root {
                    return (nodes, Some(xml_error("There are multiple root elements.", line_of(i), 2)));
                }
                seen_root = true;
            }
            nodes.push(XmlNode::Element { name: name.clone(), empty });
            if !empty {
                stack.push(name);
            }
            i += j + 1;
        }
    }
    if !stack.is_empty() {
        let open = stack.last().unwrap();
        return (nodes, Some(xml_error(&format!("Unexpected end of file has occurred. The following elements are not closed: {open}."), line_of(b.len()), 1)));
    }
    if !seen_root {
        return (nodes, Some(xml_error("Root element is missing.", 1, 1)));
    }
    (nodes, None)
}

/// `XmlTextWriter.WriteString` escaping for element content.
pub fn escape_text(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for c in s.chars() {
        match c {
            '&' => out.push_str("&amp;"),
            '<' => out.push_str("&lt;"),
            '>' => out.push_str("&gt;"),
            c if (c as u32) < 0x20 && c != '\t' && c != '\n' && c != '\r' => out.push_str(&format!("&#x{:X};", c as u32)),
            c => out.push(c),
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn node_stream_and_errors() {
        let (n, e) = read_nodes("<?xml version=\"1.0\"?>\n<Config><a>1 &amp; 2</a><b /></Config>");
        assert!(e.is_none());
        assert!(n.contains(&XmlNode::Text("1 & 2".into())));
        assert!(n.contains(&XmlNode::Element { name: "b".into(), empty: true }));
        let (n, e) = read_nodes("<Config><a>1</a><b>2</c></Config>");
        assert!(e.is_some());
        assert!(n.contains(&XmlNode::Text("1".into())), "nodes before the error are kept");
        assert_eq!(escape_text("a<b&\u{1}"), "a&lt;b&amp;&#x1;");
    }
}
