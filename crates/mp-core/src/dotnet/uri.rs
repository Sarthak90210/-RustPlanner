//! `System.Uri`, as far as the ported code reads it: `Scheme`, `UserInfo`, `Host`, `Port`,
//! `PathAndQuery`, plus `Uri.UnescapeDataString`.

use super::Exception;

fn bad(msg: &str) -> Exception {
    Exception::UriFormat(format!("Invalid URI: {msg}"))
}

/// `System.Uri`
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Uri {
    pub scheme: String,
    pub user_info: String,
    pub host: String,
    pub port: i32,
    pub path_and_query: String,
}

fn default_port(scheme: &str) -> i32 {
    match scheme {
        "http" | "ws" => 80,
        "https" | "wss" => 443,
        "ftp" => 21,
        "gopher" => 70,
        "nntp" => 119,
        "telnet" => 23,
        "ldap" => 389,
        _ => -1,
    }
}

impl Uri {
    /// `new Uri(string)`: an absolute URI.
    pub fn parse(s: &str) -> Result<Uri, Exception> {
        let s = s.trim();
        let Some(colon) = s.find(':') else {
            return Err(bad("The format of the URI could not be determined."));
        };
        let scheme = &s[..colon];
        if scheme.is_empty()
            || !scheme.chars().next().unwrap().is_ascii_alphabetic()
            || !scheme.chars().all(|c| c.is_ascii_alphanumeric() || c == '+' || c == '-' || c == '.')
        {
            return Err(bad("The format of the URI could not be determined."));
        }
        let scheme = scheme.to_ascii_lowercase();
        let rest = &s[colon + 1..];
        let Some(after) = rest.strip_prefix("//") else {
            // not hierarchical: everything after the scheme is the path
            let (path, query) = split_query(rest);
            return Ok(Uri { port: default_port(&scheme), scheme, user_info: String::new(), host: String::new(), path_and_query: format!("{path}{query}") });
        };
        let end = after.find(['/', '?', '#']).unwrap_or(after.len());
        let authority = &after[..end];
        let tail = &after[end..];
        let (user_info, hostport) = match authority.find('@') {
            Some(i) => (&authority[..i], &authority[i + 1..]),
            None => ("", authority),
        };
        if hostport.contains('@') {
            return Err(bad("The hostname could not be parsed."));
        }
        let (host, port_str) = if let Some(h) = hostport.strip_prefix('[') {
            match h.find(']') {
                Some(i) => (&hostport[..i + 2], h[i + 1..].strip_prefix(':')),
                None => return Err(bad("The hostname could not be parsed.")),
            }
        } else {
            match hostport.rfind(':') {
                Some(i) => (&hostport[..i], Some(&hostport[i + 1..])),
                None => (hostport, None),
            }
        };
        if host.is_empty() && default_port(&scheme) != -1 {
            return Err(bad("The hostname could not be parsed."));
        }
        let port = match port_str {
            None | Some("") => default_port(&scheme),
            Some(p) => {
                if !p.bytes().all(|b| b.is_ascii_digit()) {
                    return Err(bad("Invalid port specified."));
                }
                match p.parse::<i32>() {
                    Ok(v) if v <= 65535 => v,
                    _ => return Err(bad("Invalid port specified.")),
                }
            }
        };
        let (path, query) = split_query(tail);
        let path = if path.is_empty() { "/" } else { path };
        Ok(Uri {
            scheme,
            user_info: user_info.to_string(),
            host: host.to_ascii_lowercase(),
            port,
            path_and_query: format!("{path}{query}"),
        })
    }
}

/// (path, "?query") with any fragment dropped.
fn split_query(s: &str) -> (&str, &str) {
    let s = match s.find('#') {
        Some(i) => &s[..i],
        None => s,
    };
    match s.find('?') {
        Some(i) => (&s[..i], &s[i..]),
        None => (s, ""),
    }
}

/// `Uri.UnescapeDataString(s)`: `%XX` sequences decoded as UTF-8; invalid ones kept as text.
pub fn unescape_data_string(s: &str) -> String {
    let b = s.as_bytes();
    let mut out: Vec<u8> = Vec::with_capacity(b.len());
    let mut i = 0;
    while i < b.len() {
        if b[i] == b'%' && i + 2 < b.len() {
            let hex = std::str::from_utf8(&b[i + 1..i + 3]).ok().and_then(|h| u8::from_str_radix(h, 16).ok());
            if let Some(v) = hex {
                out.push(v);
                i += 3;
                continue;
            }
        }
        out.push(b[i]);
        i += 1;
    }
    match String::from_utf8(out) {
        Ok(v) => v,
        // an escaped sequence that is not valid UTF-8 stays escaped in .NET
        Err(_) => s.to_string(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_ntrip_style_urls() {
        let u = Uri::parse("http://user:p%40ss@Caster.Example.com:2101/MOUNT%3Fa").unwrap();
        assert_eq!(u.scheme, "http");
        assert_eq!(u.user_info, "user:p%40ss");
        assert_eq!(u.host, "caster.example.com");
        assert_eq!(u.port, 2101);
        assert_eq!(u.path_and_query, "/MOUNT%3Fa");
        let u = Uri::parse("https://h").unwrap();
        assert_eq!((u.port, u.path_and_query.as_str()), (443, "/"));
        assert!(Uri::parse("nonsense").is_err());
        assert!(Uri::parse("http://h:99999/").is_err());
        assert_eq!(unescape_data_string("p%40ss%2"), "p@ss%2");
    }
}
