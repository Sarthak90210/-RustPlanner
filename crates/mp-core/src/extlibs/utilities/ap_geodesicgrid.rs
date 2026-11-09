//! Port of `ExtLibs/Utilities/AP_GeodesicGrid.cs`
//!
//! ArduPilot's geodesic grid (AP_Math/AP_GeodesicGrid, Copyright (C) 2016 Intel Corporation,
//! GPLv3): the sections of an icosahedron subdivided once, used to bin compass samples by
//! direction. The tables were generated with libraries/AP_Math/tools/geodesic_grid/
//! geodesic_grid.py; the matrices are float literals widened to double, as in C#.

#![allow(non_snake_case, non_camel_case_types)]

use std::sync::OnceLock;

use super::matrix3::Matrix3;
use super::vector3::Vector3;

/// `public struct neighbor_umbrella`
#[derive(Debug, Clone, PartialEq)]
pub struct neighbor_umbrella {
    /// The umbrella's components: the icosahedron triangle index of the i-th component.
    /// For T_10 the opposite triangle, `(components[i] + 10) % 20`, is used.
    pub components: Vec<u8>,
    /// vi_cj: the index of the vector, in the icosahedron triangle pointed by
    /// `components[j]`, that matches the umbrella's i-th vertex (the same for T_10).
    pub v0_c0: u8,
    pub v1_c1: u8,
    pub v2_c1: u8,
    pub v4_c4: u8,
    pub v0_c4: u8,
}

impl neighbor_umbrella {
    /// `new neighbor_umbrella(byte[] a, byte b, byte c, byte d, byte e, byte f)`
    pub fn new(a: &[u8], b: u8, c: u8, d: u8, e: u8, f: u8) -> Self {
        neighbor_umbrella { components: a.to_vec(), v0_c0: b, v1_c1: c, v2_c1: d, v4_c4: e, v0_c4: f }
    }
}

/// `static private neighbor_umbrella[] _neighbor_umbrellas`
fn _neighbor_umbrellas() -> &'static [neighbor_umbrella; 3] {
    static T: OnceLock<[neighbor_umbrella; 3]> = OnceLock::new();
    T.get_or_init(|| {
        [
            neighbor_umbrella::new(&[9, 8, 7, 12, 14], 1, 2, 0, 0, 2),
            neighbor_umbrella::new(&[1, 2, 4, 5, 3], 0, 0, 2, 2, 0),
            neighbor_umbrella::new(&[16, 15, 13, 18, 17], 2, 2, 0, 2, 1),
        ]
    })
}

#[rustfmt::skip]
fn _inverses() -> &'static [Matrix3; 10] {
    static T: OnceLock<[Matrix3; 10]> = OnceLock::new();
    T.get_or_init(|| [
        Matrix3::from_floats(-0.309017, 0.500000, 0.190983,
             0.000000, 0.000000, -0.618034,
            -0.309017, -0.500000, 0.190983),
        Matrix3::from_floats(-0.190983, 0.309017, -0.500000,
            -0.500000, -0.190983, 0.309017,
             0.309017, -0.500000, -0.190983),
        Matrix3::from_floats(-0.618034, 0.000000, 0.000000,
             0.190983, -0.309017, -0.500000,
             0.190983, -0.309017, 0.500000),
        Matrix3::from_floats(-0.500000, 0.190983, -0.309017,
             0.000000, -0.618034, 0.000000,
             0.500000, 0.190983, -0.309017),
        Matrix3::from_floats(-0.190983, -0.309017, -0.500000,
            -0.190983, -0.309017, 0.500000,
             0.618034, 0.000000, 0.000000),
        Matrix3::from_floats(-0.309017, -0.500000, -0.190983,
             0.190983, 0.309017, -0.500000,
             0.500000, -0.190983, 0.309017),
        Matrix3::from_floats(0.309017, -0.500000, 0.190983,
             0.000000, 0.000000, -0.618034,
             0.309017, 0.500000, 0.190983),
        Matrix3::from_floats(0.190983, -0.309017, -0.500000,
             0.500000, 0.190983, 0.309017,
            -0.309017, 0.500000, -0.190983),
        Matrix3::from_floats(0.500000, -0.190983, -0.309017,
             0.000000, 0.618034, 0.000000,
            -0.500000, -0.190983, -0.309017),
        Matrix3::from_floats(0.309017, 0.500000, -0.190983,
            -0.500000, 0.190983, 0.309017,
            -0.190983, -0.309017, -0.500000),
    ])
}

#[rustfmt::skip]
fn _mid_inverses() -> &'static [Matrix3; 10] {
    static T: OnceLock<[Matrix3; 10]> = OnceLock::new();
    T.get_or_init(|| [
        Matrix3::from_floats(-0.000000, 1.000000, -0.618034,
             0.000000, -1.000000, -0.618034,
             -0.618034, 0.000000, 1.000000),
        Matrix3::from_floats(-1.000000, 0.618034, -0.000000,
             -0.000000, -1.000000, 0.618034,
             0.618034, -0.000000, -1.000000),
        Matrix3::from_floats(-0.618034, -0.000000, -1.000000,
            1.000000, -0.618034, -0.000000,
            -0.618034, 0.000000, 1.000000),
        Matrix3::from_floats(-1.000000, -0.618034, -0.000000,
            1.000000, -0.618034, 0.000000,
            -0.000000, 1.000000, -0.618034),
        Matrix3::from_floats(-1.000000, -0.618034, 0.000000,
            0.618034, 0.000000, 1.000000,
            0.618034, 0.000000, -1.000000),
        Matrix3::from_floats(-0.618034, -0.000000, -1.000000,
            1.000000, 0.618034, -0.000000,
            0.000000, -1.000000, 0.618034),
        Matrix3::from_floats(0.000000, -1.000000, -0.618034,
            0.000000, 1.000000, -0.618034,
            0.618034, -0.000000, 1.000000),
        Matrix3::from_floats(1.000000, -0.618034, -0.000000,
            0.000000, 1.000000, 0.618034,
            -0.618034, 0.000000, -1.000000),
        Matrix3::from_floats(1.000000, 0.618034, -0.000000,
            -1.000000, 0.618034, 0.000000,
            0.000000, -1.000000, -0.618034),
        Matrix3::from_floats(-0.000000, 1.000000, 0.618034,
            -1.000000, -0.618034, -0.000000,
            0.618034, 0.000000, -1.000000),
    ])
}

/// `Vector3[int]` for the 0..2 indices the tables hold.
fn idx(w: &Vector3, i: u8) -> f64 {
    match i {
        0 => w.x,
        1 => w.y,
        _ => w.z,
    }
}

/// `public static class AP_GeodesicGrid`
pub struct AP_GeodesicGrid;

impl AP_GeodesicGrid {
    /// `section(Vector3 v, bool inclusive)`: the section (0..79) crossed by `v`, or -1 (null
    /// vector, or a border when not `inclusive`).
    pub fn section(v: Vector3, inclusive: bool) -> i32 {
        let i = Self::_triangle_index(v, inclusive);
        if i < 0 {
            return -1;
        }

        let j = Self::_subtriangle_index(i, v, inclusive);
        if j < 0 {
            return -1;
        }

        4 * i + j
    }

    fn _neighbor_umbrella_component(idx: i32, comp_idx: usize) -> i32 {
        let u = _neighbor_umbrellas();
        if idx < 3 {
            return u[idx as usize].components[comp_idx] as i32;
        }
        (u[(idx % 3) as usize].components[comp_idx] as i32 + 10) % 20
    }

    fn _from_neighbor_umbrella(idx: i32, v: &Vector3, u: &Vector3, inclusive: bool) -> i32 {
        let um = &_neighbor_umbrellas()[(idx % 3) as usize];
        let inv = _inverses();

        /* The following comparisons between the umbrella's first and second
         * vertices' coefficients work for this algorithm because all vertices'
         * vectors are of the same length. */

        if Self::is_equal(u.x, u.y) {
            /* If the coefficients of the first and second vertices are equal, then
             * v crosses the first component or the edge formed by the umbrella's
             * pivot and forth vertex. */
            let comp = Self::_neighbor_umbrella_component(idx, 0);
            let mut w = inv[(comp % 10) as usize] * *v;
            if comp > 9 {
                w = -w;
            }
            let x0 = idx_f(&w, um.v0_c0);
            if Self::is_zero(x0 as f64) {
                if !inclusive {
                    return -1;
                }
                return comp;
            } else if x0 < 0.0 {
                if !inclusive {
                    return -1;
                }
                return Self::_neighbor_umbrella_component(idx, if u.x < u.y { 3 } else { 2 });
            }

            return comp;
        }

        if u.y > u.x {
            /* If the coefficient of the second vertex is greater than the first
             * one's, then v crosses the first, second or third component. */
            let comp = Self::_neighbor_umbrella_component(idx, 1);
            let mut w = inv[(comp % 10) as usize] * *v;
            if comp > 9 {
                w = -w;
            }
            let x1 = idx_f(&w, um.v1_c1);
            let x2 = idx_f(&w, um.v2_c1);

            if Self::is_zero(x1 as f64) {
                if !inclusive {
                    return -1;
                }
                return Self::_neighbor_umbrella_component(idx, if x1 < 0.0 { 2 } else { 1 });
            } else if x1 < 0.0 {
                return Self::_neighbor_umbrella_component(idx, 2);
            }

            if Self::is_zero(x2 as f64) {
                if !inclusive {
                    return -1;
                }
                return Self::_neighbor_umbrella_component(idx, if x2 > 0.0 { 1 } else { 0 });
            } else if x2 < 0.0 {
                return Self::_neighbor_umbrella_component(idx, 0);
            }

            comp
        } else {
            /* If the coefficient of the second vertex is lesser than the first
             * one's, then v crosses the first, fourth or fifth component. */
            let comp = Self::_neighbor_umbrella_component(idx, 4);
            let mut w = inv[(comp % 10) as usize] * *v;
            if comp > 9 {
                w = -w;
            }
            let x4 = idx_f(&w, um.v4_c4);
            let x0 = idx_f(&w, um.v0_c4);

            if Self::is_zero(x4 as f64) {
                if !inclusive {
                    return -1;
                }
                return Self::_neighbor_umbrella_component(idx, if x4 < 0.0 { 0 } else { 4 });
            } else if x4 < 0.0 {
                return Self::_neighbor_umbrella_component(idx, 0);
            }

            if Self::is_zero(x0 as f64) {
                if !inclusive {
                    return -1;
                }
                return Self::_neighbor_umbrella_component(idx, if x0 > 0.0 { 4 } else { 3 });
            } else if x0 < 0.0 {
                return Self::_neighbor_umbrella_component(idx, 3);
            }

            comp
        }
    }

    fn is_zero(x0: f64) -> bool {
        x0 == 0.0
    }

    fn is_equal(x: f64, y: f64) -> bool {
        x == y
    }

    fn _triangle_index(v: Vector3, inclusive: bool) -> i32 {
        /* w holds the coordinates of v with respect to the basis comprised by the
         * vectors of T_i */
        let mut w = _inverses()[0] * v;
        let mut zero_count = 0;
        let mut balance = 0;
        let mut umbrella = -1;

        for c in [w.x, w.y, w.z] {
            if Self::is_zero(c) {
                zero_count += 1;
            } else if c > 0.0 {
                balance += 1;
            } else {
                balance -= 1;
            }
        }

        match balance {
            /* All coefficients are positive, thus return the first triangle. */
            3 => return 0,
            /* All coefficients are negative: the first triangle's opposite. */
            -3 => return 10,
            /* Two coefficients are positive and one is zero: an edge of the first triangle. */
            2 => return if inclusive { 0 } else { -1 },
            /* Analogous, for the opposite of the first triangle. */
            -2 => return if inclusive { 10 } else { -1 },
            1 => {
                /* Two coefficients zero (a vertex of the first triangle), or two positive
                 * and one negative (a component of that edge's neighbor umbrella). */
                if zero_count == 2 {
                    return if inclusive { 0 } else { -1 };
                }

                umbrella = if !Self::is_zero(w.x) && w.x < 0.0 {
                    1
                } else if !Self::is_zero(w.y) && w.y < 0.0 {
                    2
                } else {
                    0
                };
            }
            -1 => {
                /* Analogous to the previous case, for the opposite of the first triangle. */
                if zero_count == 2 {
                    return if inclusive { 10 } else { -1 };
                }

                umbrella = if !Self::is_zero(w.x) && w.x > 0.0 {
                    4
                } else if !Self::is_zero(w.y) && w.y > 0.0 {
                    5
                } else {
                    3
                };
                w = -w;
            }
            0 => {
                /* The null vector crosses no section; otherwise one coefficient is zero,
                 * one positive and one negative, as for balance 1. */
                if zero_count == 3 {
                    return -1;
                }

                umbrella = if !Self::is_zero(w.x) && w.x < 0.0 {
                    1
                } else if !Self::is_zero(w.y) && w.y < 0.0 {
                    2
                } else {
                    0
                };
            }
            _ => {}
        }

        //assert(umbrella >= 0);

        match umbrella % 3 {
            0 => w.z = -w.z,
            1 => w = Vector3::new(w.y, w.z, -w.x),
            2 => w = Vector3::new(w.z, w.x, -w.y),
            _ => {}
        }

        Self::_from_neighbor_umbrella(umbrella, &v, &w, inclusive)
    }

    fn _subtriangle_index(triangle_index: i32, v: Vector3, inclusive: bool) -> i32 {
        /* w holds the coordinates of v with respect to the basis comprised by the
         * vectors of the middle triangle of T_i where i is triangle_index */
        let mut w = _mid_inverses()[(triangle_index % 10) as usize] * v;
        if triangle_index > 9 {
            w = -w;
        }

        if (Self::is_zero(w.x) || Self::is_zero(w.y) || Self::is_zero(w.z)) && !inclusive {
            return -1;
        }

        /* v crosses the icosahedron triangle; if it misses the middle triangle, one
         * coefficient is negative and v crosses that edge's neighbor sub-triangle. */
        if !Self::is_zero(w.x) && w.x < 0.0 {
            return 3;
        }
        if !Self::is_zero(w.y) && w.y < 0.0 {
            return 1;
        }
        if !Self::is_zero(w.z) && w.z < 0.0 {
            return 2;
        }

        /* If x >= 0 and y >= 0 and z >= 0, then v crosses the middle triangle. */
        0
    }
}

/// `(float) w[i]`
fn idx_f(w: &Vector3, i: u8) -> f32 {
    idx(w, i) as f32
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_direction_lands_in_one_of_80_sections() {
        assert_eq!(AP_GeodesicGrid::section(Vector3::new(0.0, 0.0, 0.0), true), -1);
        let mut seen = std::collections::BTreeSet::new();
        for i in 0..2000 {
            // a spiral of directions over the sphere
            let t = i as f64 / 2000.0;
            let z = 1.0 - 2.0 * t;
            let r = (1.0 - z * z).sqrt();
            let phi = i as f64 * 2.399963229728653;
            let s = AP_GeodesicGrid::section(Vector3::new(r * phi.cos(), r * phi.sin(), z), true);
            assert!((0..80).contains(&s), "{s}");
            seen.insert(s);
        }
        assert_eq!(seen.len(), 80);
        // opposite directions land in opposite triangles (T_i and T_i+10)
        let v = Vector3::new(0.3, -0.2, 0.9);
        let (a, b) = (AP_GeodesicGrid::section(v, true), AP_GeodesicGrid::section(-v, true));
        assert_eq!((a / 4 + 10) % 20, b / 4);
    }
}
