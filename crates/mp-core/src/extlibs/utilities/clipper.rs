//! Port of `ExtLibs/Utilities/clipper.cs`
//!
//! Angus Johnson's Clipper 6.4.2 (Boost Software License 1.0), namespace `ClipperLib`, built
//! with `use_lines` and 64-bit coordinates. The C# links its objects by reference (edges,
//! output points, output records, local minima, scanbeams, maxima, poly nodes); here each kind
//! lives in an arena owned by the clipper (or the tree) and a reference is an index, with
//! `NIL` for `null`. The algorithms and their order of operations are unchanged.

#![allow(non_snake_case, non_camel_case_types, non_upper_case_globals, clippy::all)]

use crate::dotnet::Exception;

/// `using cInt = Int64`
pub type cInt = i64;

/// A null reference into one of the arenas.
pub const NIL: usize = usize::MAX;

/// `public struct DoublePoint`
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct DoublePoint {
    pub X: f64,
    pub Y: f64,
}

impl DoublePoint {
    pub fn new(x: f64, y: f64) -> Self {
        DoublePoint { X: x, Y: y }
    }

    /// `new DoublePoint(IntPoint ip)`
    pub fn from_int(ip: IntPoint) -> Self {
        DoublePoint { X: ip.X as f64, Y: ip.Y as f64 }
    }
}

/// `public struct IntPoint`
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash)]
pub struct IntPoint {
    pub X: cInt,
    pub Y: cInt,
}

impl IntPoint {
    /// `new IntPoint(cInt X, cInt Y)`
    pub fn new(X: cInt, Y: cInt) -> Self {
        IntPoint { X, Y }
    }

    /// `new IntPoint(double x, double y)`: `(cInt)x`, truncating (NaN and out of range give
    /// long.MinValue, as the x64 conversion does).
    pub fn from_f64(x: f64, y: f64) -> Self {
        IntPoint { X: crate::dotnet::cast_i64(x), Y: crate::dotnet::cast_i64(y) }
    }
}

/// `using Path = List<IntPoint>`
pub type Path = Vec<IntPoint>;
/// `using Paths = List<List<IntPoint>>`
pub type Paths = Vec<Vec<IntPoint>>;

/// `public struct IntRect`
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct IntRect {
    pub left: cInt,
    pub top: cInt,
    pub right: cInt,
    pub bottom: cInt,
}

impl IntRect {
    pub fn new(l: cInt, t: cInt, r: cInt, b: cInt) -> Self {
        IntRect { left: l, top: t, right: r, bottom: b }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClipType {
    ctIntersection,
    ctUnion,
    ctDifference,
    ctXor,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PolyType {
    ptSubject,
    ptClip,
}

/// By far the most widely used winding rules for polygon filling are EvenOdd and NonZero
/// (GDI, GDI+, XLib, OpenGL, Cairo, AGG, Quartz, SVG, Gr32).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PolyFillType {
    pftEvenOdd,
    pftNonZero,
    pftPositive,
    pftNegative,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum JoinType {
    #[default]
    jtSquare,
    jtRound,
    jtMiter,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum EndType {
    #[default]
    etClosedPolygon,
    etClosedLine,
    etOpenButt,
    etOpenSquare,
    etOpenRound,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
enum EdgeSide {
    #[default]
    esLeft,
    esRight,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Direction {
    dRightToLeft,
    dLeftToRight,
}

/// `public class ClipperException : Exception`
fn clipper_exception(msg: &str) -> Exception {
    Exception::Custom("ClipperLib.ClipperException", msg.to_string())
}

//------------------------------------------------------------------------------
// PolyTree & PolyNode classes
//------------------------------------------------------------------------------

/// `public class PolyNode`: one node of a tree arena (`PolyTree`); node 0 is the root.
#[derive(Debug, Clone, Default)]
pub struct PolyNode {
    m_Parent: Option<usize>,
    m_polygon: Path,
    m_Index: i32,
    m_jointype: JoinType,
    m_endtype: EndType,
    m_Childs: Vec<usize>,
    /// `IsOpen`
    pub IsOpen: bool,
}

/// `public class PolyTree : PolyNode`, holding every node it owns; the tree itself is node 0.
#[derive(Debug, Clone)]
pub struct PolyTree {
    nodes: Vec<PolyNode>,
    m_AllPolys: Vec<usize>,
}

impl Default for PolyTree {
    fn default() -> Self {
        PolyTree { nodes: vec![PolyNode::default()], m_AllPolys: Vec::new() }
    }
}

impl PolyTree {
    /// `new PolyTree()`
    pub fn new() -> Self {
        Self::default()
    }

    /// the tree's own node
    pub const ROOT: usize = 0;

    fn new_node(&mut self) -> usize {
        self.nodes.push(PolyNode::default());
        self.nodes.len() - 1
    }

    /// `Clear()`
    pub fn Clear(&mut self) {
        self.m_AllPolys.clear();
        self.nodes[Self::ROOT].m_Childs.clear();
    }

    /// `GetFirst()`: the first child of the tree, or null.
    pub fn GetFirst(&self) -> Option<usize> {
        self.nodes[Self::ROOT].m_Childs.first().copied()
    }

    /// `Total`: every polygon, less the hidden outer one of a negative offset.
    pub fn Total(&self) -> i32 {
        let mut result = self.m_AllPolys.len() as i32;
        //with negative offsets, ignore the hidden outer polygon ...
        if result > 0 && self.nodes[Self::ROOT].m_Childs[0] != self.m_AllPolys[0] {
            result -= 1;
        }
        result
    }

    /// `node.IsHoleNode()`
    fn IsHoleNode(&self, node: usize) -> bool {
        let mut result = true;
        let mut n = self.nodes[node].m_Parent;
        while let Some(p) = n {
            result = !result;
            n = self.nodes[p].m_Parent;
        }
        result
    }

    /// `node.ChildCount` (`ChildCount` of the tree is `ChildCount(ROOT)`)
    pub fn ChildCount(&self, node: usize) -> i32 {
        self.nodes[node].m_Childs.len() as i32
    }

    /// `node.Contour`
    pub fn Contour(&self, node: usize) -> &Path {
        &self.nodes[node].m_polygon
    }

    /// `node.AddChild(child)`
    fn AddChild(&mut self, node: usize, child: usize) {
        let cnt = self.nodes[node].m_Childs.len() as i32;
        self.nodes[node].m_Childs.push(child);
        self.nodes[child].m_Parent = Some(node);
        self.nodes[child].m_Index = cnt;
    }

    /// `node.GetNext()`
    pub fn GetNext(&self, node: usize) -> Option<usize> {
        if !self.nodes[node].m_Childs.is_empty() {
            Some(self.nodes[node].m_Childs[0])
        } else {
            self.GetNextSiblingUp(node)
        }
    }

    fn GetNextSiblingUp(&self, node: usize) -> Option<usize> {
        let p = self.nodes[node].m_Parent?;
        if self.nodes[node].m_Index == self.nodes[p].m_Childs.len() as i32 - 1 {
            self.GetNextSiblingUp(p)
        } else {
            Some(self.nodes[p].m_Childs[(self.nodes[node].m_Index + 1) as usize])
        }
    }

    /// `node.Childs`
    pub fn Childs(&self, node: usize) -> &[usize] {
        &self.nodes[node].m_Childs
    }

    /// `node.Parent`
    pub fn Parent(&self, node: usize) -> Option<usize> {
        self.nodes[node].m_Parent
    }

    /// `node.IsHole`
    pub fn IsHole(&self, node: usize) -> bool {
        self.IsHoleNode(node)
    }

    /// `node.IsOpen`
    pub fn IsOpen(&self, node: usize) -> bool {
        self.nodes[node].IsOpen
    }
}

//------------------------------------------------------------------------------
// Int128 struct (enables safe math on signed 64bit integers)
//------------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Int128 {
    hi: i64,
    lo: u64,
}

// the comparison, subtraction and double operators are part of the C# struct, though nothing
// in this file calls them
#[allow(dead_code)]
impl Int128 {
    fn from_i64(_lo: i64) -> Self {
        Int128 { lo: _lo as u64, hi: if _lo < 0 { -1 } else { 0 } }
    }

    fn new(_hi: i64, _lo: u64) -> Self {
        Int128 { hi: _hi, lo: _lo }
    }

    fn IsNegative(&self) -> bool {
        self.hi < 0
    }

    fn gt(self, val2: Int128) -> bool {
        if self.hi != val2.hi {
            self.hi > val2.hi
        } else {
            self.lo > val2.lo
        }
    }

    fn lt(self, val2: Int128) -> bool {
        if self.hi != val2.hi {
            self.hi < val2.hi
        } else {
            self.lo < val2.lo
        }
    }

    fn add(mut self, rhs: Int128) -> Int128 {
        self.hi = self.hi.wrapping_add(rhs.hi);
        self.lo = self.lo.wrapping_add(rhs.lo);
        if self.lo < rhs.lo {
            self.hi = self.hi.wrapping_add(1);
        }
        self
    }

    fn sub(self, rhs: Int128) -> Int128 {
        self.add(rhs.neg())
    }

    fn neg(self) -> Int128 {
        if self.lo == 0 {
            Int128::new(self.hi.wrapping_neg(), 0)
        } else {
            Int128::new(!self.hi, (!self.lo).wrapping_add(1))
        }
    }

    /// `explicit operator double(Int128 val)`
    fn to_f64(self) -> f64 {
        const shift64: f64 = 18446744073709551616.0; //2^64
        if self.hi < 0 {
            if self.lo == 0 {
                self.hi as f64 * shift64
            } else {
                -((!self.lo) as f64 + (!self.hi) as f64 * shift64)
            }
        } else {
            self.lo as f64 + self.hi as f64 * shift64
        }
    }

    /// `Int128Mul(Int64 lhs, Int64 rhs)`
    fn Int128Mul(mut lhs: i64, mut rhs: i64) -> Int128 {
        let negate = (lhs < 0) != (rhs < 0);
        if lhs < 0 {
            lhs = lhs.wrapping_neg();
        }
        if rhs < 0 {
            rhs = rhs.wrapping_neg();
        }
        let int1Hi = (lhs as u64) >> 32;
        let int1Lo = (lhs as u64) & 0xFFFFFFFF;
        let int2Hi = (rhs as u64) >> 32;
        let int2Lo = (rhs as u64) & 0xFFFFFFFF;

        //nb: see comments in clipper.pas
        let a = int1Hi.wrapping_mul(int2Hi);
        let b = int1Lo.wrapping_mul(int2Lo);
        let c = int1Hi.wrapping_mul(int2Lo).wrapping_add(int1Lo.wrapping_mul(int2Hi));

        let mut hi = a.wrapping_add(c >> 32) as i64;
        let lo = (c << 32).wrapping_add(b);
        if lo < b {
            hi = hi.wrapping_add(1);
        }
        let result = Int128::new(hi, lo);
        if negate {
            result.neg()
        } else {
            result
        }
    }
}

//------------------------------------------------------------------------------
// internal classes, as arena records
//------------------------------------------------------------------------------

#[derive(Debug, Clone, Copy)]
struct TEdge {
    Bot: IntPoint,
    /// current (updated for every new scanbeam)
    Curr: IntPoint,
    Top: IntPoint,
    Delta: IntPoint,
    Dx: f64,
    PolyTyp: PolyType,
    /// side only refers to current side of solution poly
    Side: EdgeSide,
    /// 1 or -1 depending on winding direction
    WindDelta: i32,
    WindCnt: i32,
    /// winding count of the opposite polytype
    WindCnt2: i32,
    OutIdx: i32,
    Next: usize,
    Prev: usize,
    NextInLML: usize,
    NextInAEL: usize,
    PrevInAEL: usize,
    NextInSEL: usize,
    PrevInSEL: usize,
}

impl Default for TEdge {
    fn default() -> Self {
        TEdge {
            Bot: IntPoint::default(),
            Curr: IntPoint::default(),
            Top: IntPoint::default(),
            Delta: IntPoint::default(),
            Dx: 0.0,
            PolyTyp: PolyType::ptSubject,
            Side: EdgeSide::esLeft,
            WindDelta: 0,
            WindCnt: 0,
            WindCnt2: 0,
            OutIdx: 0,
            Next: NIL,
            Prev: NIL,
            NextInLML: NIL,
            NextInAEL: NIL,
            PrevInAEL: NIL,
            NextInSEL: NIL,
            PrevInSEL: NIL,
        }
    }
}

/// `public class IntersectNode`
#[derive(Debug, Clone, Copy, Default)]
struct IntersectNode {
    Edge1: usize,
    Edge2: usize,
    Pt: IntPoint,
}

/// `MyIntersectNodeSort.Compare`: descending Y.
fn MyIntersectNodeSort(node1: &IntersectNode, node2: &IntersectNode) -> i32 {
    let i = node2.Pt.Y.wrapping_sub(node1.Pt.Y);
    if i > 0 {
        1
    } else if i < 0 {
        -1
    } else {
        0
    }
}

#[derive(Debug, Clone, Copy)]
struct LocalMinima {
    Y: cInt,
    LeftBound: usize,
    RightBound: usize,
    Next: usize,
}

#[derive(Debug, Clone, Copy)]
struct Scanbeam {
    Y: cInt,
    Next: usize,
}

#[derive(Debug, Clone, Copy)]
struct Maxima {
    X: cInt,
    Next: usize,
    Prev: usize,
}

/// `OutRec`: a path in the clipping solution. Edges in the AEL carry its index (`OutIdx`)
/// when they are part of the clipping solution.
#[derive(Debug, Clone, Copy)]
struct OutRec {
    Idx: i32,
    IsHole: bool,
    IsOpen: bool,
    /// see comments in clipper.pas
    FirstLeft: usize,
    Pts: usize,
    BottomPt: usize,
    PolyNode: usize,
}

#[derive(Debug, Clone, Copy)]
struct OutPt {
    Idx: i32,
    Pt: IntPoint,
    Next: usize,
    Prev: usize,
}

#[derive(Debug, Clone, Copy)]
struct Join {
    OutPt1: usize,
    OutPt2: usize,
    OffPt: IntPoint,
}

//------------------------------------------------------------------------------
// ClipperBase + Clipper (one struct: ClipperBase is only ever a Clipper's base)
//------------------------------------------------------------------------------

const horizontal: f64 = -3.4E+38;
const Skip: i32 = -2;
const Unassigned: i32 = -1;
const tolerance: f64 = 1.0E-20;

fn near_zero(val: f64) -> bool {
    (val > -tolerance) && (val < tolerance)
}

/// `ClipperBase.loRange`
pub const loRange: cInt = 0x3FFFFFFF;
/// `ClipperBase.hiRange`
pub const hiRange: cInt = 0x3FFFFFFFFFFFFFFF;

/// `public class Clipper : ClipperBase`
pub struct Clipper {
    // --- arenas ---
    edges: Vec<TEdge>,
    lms: Vec<LocalMinima>,
    sbs: Vec<Scanbeam>,
    maxs: Vec<Maxima>,
    outrecs: Vec<OutRec>,
    outpts: Vec<OutPt>,

    // --- ClipperBase ---
    m_MinimaList: usize,
    m_CurrentLM: usize,
    m_edges: Vec<Vec<usize>>,
    m_Scanbeam: usize,
    /// the output records by index (null once disposed)
    m_PolyOuts: Vec<usize>,
    m_ActiveEdges: usize,
    m_UseFullRange: bool,
    m_HasOpenPaths: bool,
    /// `PreserveCollinear`
    pub PreserveCollinear: bool,

    // --- Clipper ---
    m_ClipType: ClipType,
    m_Maxima: usize,
    m_SortedEdges: usize,
    m_IntersectList: Vec<IntersectNode>,
    /// `m_IntersectList.Capacity`: List<T> grows 0, 4, 8, 16 ... and Clear() keeps it; the
    /// sort's backing length
    m_IntersectCapacity: usize,
    m_ExecuteLocked: bool,
    m_ClipFillType: PolyFillType,
    m_SubjFillType: PolyFillType,
    m_Joins: Vec<Join>,
    m_GhostJoins: Vec<Join>,
    m_UsingPolyTree: bool,
    /// `ReverseSolution`
    pub ReverseSolution: bool,
    /// `StrictlySimple`
    pub StrictlySimple: bool,
}

impl Clipper {
    /// `ioReverseSolution`
    pub const ioReverseSolution: i32 = 1;
    /// `ioStrictlySimple`
    pub const ioStrictlySimple: i32 = 2;
    /// `ioPreserveCollinear`
    pub const ioPreserveCollinear: i32 = 4;

    /// `new Clipper(int InitOptions = 0)`
    pub fn new(InitOptions: i32) -> Self {
        Clipper {
            edges: Vec::new(),
            lms: Vec::new(),
            sbs: Vec::new(),
            maxs: Vec::new(),
            outrecs: Vec::new(),
            outpts: Vec::new(),
            m_MinimaList: NIL,
            m_CurrentLM: NIL,
            m_edges: Vec::new(),
            m_Scanbeam: NIL,
            m_PolyOuts: Vec::new(),
            m_ActiveEdges: NIL,
            m_UseFullRange: false,
            m_HasOpenPaths: false,
            PreserveCollinear: (Self::ioPreserveCollinear & InitOptions) != 0,
            m_ClipType: ClipType::ctIntersection,
            m_Maxima: NIL,
            m_SortedEdges: NIL,
            m_IntersectList: Vec::new(),
            m_IntersectCapacity: 0,
            m_ExecuteLocked: false,
            m_ClipFillType: PolyFillType::pftEvenOdd,
            m_SubjFillType: PolyFillType::pftEvenOdd,
            m_Joins: Vec::new(),
            m_GhostJoins: Vec::new(),
            m_UsingPolyTree: false,
            ReverseSolution: (Self::ioReverseSolution & InitOptions) != 0,
            StrictlySimple: (Self::ioStrictlySimple & InitOptions) != 0,
        }
    }

    // ---------------------------------------------------------------- ClipperBase

    fn IsHorizontal(&self, e: usize) -> bool {
        self.edges[e].Delta.Y == 0
    }

    #[allow(dead_code)]
    fn PointIsVertex(&self, pt: IntPoint, pp: usize) -> bool {
        let mut pp2 = pp;
        loop {
            if self.outpts[pp2].Pt == pt {
                return true;
            }
            pp2 = self.outpts[pp2].Next;
            if pp2 == pp {
                break;
            }
        }
        false
    }

    #[allow(dead_code)]
    fn PointOnLineSegment(pt: IntPoint, linePt1: IntPoint, linePt2: IntPoint, UseFullRange: bool) -> bool {
        if UseFullRange {
            ((pt.X == linePt1.X) && (pt.Y == linePt1.Y))
                || ((pt.X == linePt2.X) && (pt.Y == linePt2.Y))
                || (((pt.X > linePt1.X) == (pt.X < linePt2.X))
                    && ((pt.Y > linePt1.Y) == (pt.Y < linePt2.Y))
                    && (Int128::Int128Mul(pt.X.wrapping_sub(linePt1.X), linePt2.Y.wrapping_sub(linePt1.Y))
                        == Int128::Int128Mul(linePt2.X.wrapping_sub(linePt1.X), pt.Y.wrapping_sub(linePt1.Y))))
        } else {
            ((pt.X == linePt1.X) && (pt.Y == linePt1.Y))
                || ((pt.X == linePt2.X) && (pt.Y == linePt2.Y))
                || (((pt.X > linePt1.X) == (pt.X < linePt2.X))
                    && ((pt.Y > linePt1.Y) == (pt.Y < linePt2.Y))
                    && (pt.X.wrapping_sub(linePt1.X).wrapping_mul(linePt2.Y.wrapping_sub(linePt1.Y))
                        == linePt2.X.wrapping_sub(linePt1.X).wrapping_mul(pt.Y.wrapping_sub(linePt1.Y))))
        }
    }

    #[allow(dead_code)]
    fn PointOnPolygon(&self, pt: IntPoint, pp: usize, UseFullRange: bool) -> bool {
        let mut pp2 = pp;
        loop {
            let next = self.outpts[pp2].Next;
            if Self::PointOnLineSegment(pt, self.outpts[pp2].Pt, self.outpts[next].Pt, UseFullRange) {
                return true;
            }
            pp2 = next;
            if pp2 == pp {
                break;
            }
        }
        false
    }

    fn SlopesEqualE(&self, e1: usize, e2: usize, UseFullRange: bool) -> bool {
        let (d1, d2) = (self.edges[e1].Delta, self.edges[e2].Delta);
        if UseFullRange {
            Int128::Int128Mul(d1.Y, d2.X) == Int128::Int128Mul(d1.X, d2.Y)
        } else {
            d1.Y.wrapping_mul(d2.X) == d1.X.wrapping_mul(d2.Y)
        }
    }

    fn SlopesEqual3(pt1: IntPoint, pt2: IntPoint, pt3: IntPoint, UseFullRange: bool) -> bool {
        if UseFullRange {
            Int128::Int128Mul(pt1.Y.wrapping_sub(pt2.Y), pt2.X.wrapping_sub(pt3.X)) == Int128::Int128Mul(pt1.X.wrapping_sub(pt2.X), pt2.Y.wrapping_sub(pt3.Y))
        } else {
            pt1.Y
                .wrapping_sub(pt2.Y)
                .wrapping_mul(pt2.X.wrapping_sub(pt3.X))
                .wrapping_sub(pt1.X.wrapping_sub(pt2.X).wrapping_mul(pt2.Y.wrapping_sub(pt3.Y)))
                == 0
        }
    }

    fn SlopesEqual4(pt1: IntPoint, pt2: IntPoint, pt3: IntPoint, pt4: IntPoint, UseFullRange: bool) -> bool {
        if UseFullRange {
            Int128::Int128Mul(pt1.Y.wrapping_sub(pt2.Y), pt3.X.wrapping_sub(pt4.X)) == Int128::Int128Mul(pt1.X.wrapping_sub(pt2.X), pt3.Y.wrapping_sub(pt4.Y))
        } else {
            pt1.Y
                .wrapping_sub(pt2.Y)
                .wrapping_mul(pt3.X.wrapping_sub(pt4.X))
                .wrapping_sub(pt1.X.wrapping_sub(pt2.X).wrapping_mul(pt3.Y.wrapping_sub(pt4.Y)))
                == 0
        }
    }

    /// `ClipperBase.Clear()` (Clipper does not override it)
    pub fn Clear(&mut self) {
        self.DisposeLocalMinimaList();
        self.m_edges.clear();
        self.edges.clear();
        self.m_UseFullRange = false;
        self.m_HasOpenPaths = false;
    }

    fn DisposeLocalMinimaList(&mut self) {
        self.m_MinimaList = NIL;
        self.lms.clear();
        self.m_CurrentLM = NIL;
    }

    fn RangeTest(Pt: IntPoint, useFullRange: &mut bool) -> Result<(), Exception> {
        if *useFullRange {
            if Pt.X > hiRange || Pt.Y > hiRange || Pt.X.wrapping_neg() > hiRange || Pt.Y.wrapping_neg() > hiRange {
                return Err(clipper_exception("Coordinate outside allowed range"));
            }
        } else if Pt.X > loRange || Pt.Y > loRange || Pt.X.wrapping_neg() > loRange || Pt.Y.wrapping_neg() > loRange {
            *useFullRange = true;
            Self::RangeTest(Pt, useFullRange)?;
        }
        Ok(())
    }

    fn InitEdge(&mut self, e: usize, eNext: usize, ePrev: usize, pt: IntPoint) {
        let ed = &mut self.edges[e];
        ed.Next = eNext;
        ed.Prev = ePrev;
        ed.Curr = pt;
        ed.OutIdx = Unassigned;
    }

    fn InitEdge2(&mut self, e: usize, polyType: PolyType) {
        let next = self.edges[e].Next;
        let nextCurr = self.edges[next].Curr;
        let ed = &mut self.edges[e];
        if ed.Curr.Y >= nextCurr.Y {
            ed.Bot = ed.Curr;
            ed.Top = nextCurr;
        } else {
            ed.Top = ed.Curr;
            ed.Bot = nextCurr;
        }
        self.SetDx(e);
        self.edges[e].PolyTyp = polyType;
    }

    fn FindNextLocMin(&self, mut E: usize) -> usize {
        let ed = &self.edges;
        loop {
            while ed[E].Bot != ed[ed[E].Prev].Bot || ed[E].Curr == ed[E].Top {
                E = ed[E].Next;
            }
            if ed[E].Dx != horizontal && ed[ed[E].Prev].Dx != horizontal {
                break;
            }
            while ed[ed[E].Prev].Dx == horizontal {
                E = ed[E].Prev;
            }
            let E2 = E;
            while ed[E].Dx == horizontal {
                E = ed[E].Next;
            }
            if ed[E].Top.Y == ed[ed[E].Prev].Bot.Y {
                continue; //ie just an intermediate horz.
            }
            if ed[ed[E2].Prev].Bot.X < ed[E].Bot.X {
                E = E2;
            }
            break;
        }
        E
    }

    fn ProcessBound(&mut self, mut E: usize, LeftBoundIsForward: bool) -> usize {
        let mut Result = E;

        if self.edges[Result].OutIdx == Skip {
            //check if there are edges beyond the skip edge in the bound and if so
            //create another LocMin and calling ProcessBound once more ...
            E = Result;
            if LeftBoundIsForward {
                while self.edges[E].Top.Y == self.edges[self.edges[E].Next].Bot.Y {
                    E = self.edges[E].Next;
                }
                while E != Result && self.edges[E].Dx == horizontal {
                    E = self.edges[E].Prev;
                }
            } else {
                while self.edges[E].Top.Y == self.edges[self.edges[E].Prev].Bot.Y {
                    E = self.edges[E].Prev;
                }
                while E != Result && self.edges[E].Dx == horizontal {
                    E = self.edges[E].Next;
                }
            }
            if E == Result {
                if LeftBoundIsForward {
                    Result = self.edges[E].Next;
                } else {
                    Result = self.edges[E].Prev;
                }
            } else {
                //there are more edges in the bound beyond result starting with E
                if LeftBoundIsForward {
                    E = self.edges[Result].Next;
                } else {
                    E = self.edges[Result].Prev;
                }
                let locMin = self.new_lm(LocalMinima { Y: self.edges[E].Bot.Y, LeftBound: NIL, RightBound: E, Next: NIL });
                self.edges[E].WindDelta = 0;
                Result = self.ProcessBound(E, LeftBoundIsForward);
                self.InsertLocalMinima(locMin);
            }
            return Result;
        }

        let mut EStart;
        if self.edges[E].Dx == horizontal {
            //We need to be careful with open paths because this may not be a
            //true local minima (ie E may be following a skip edge).
            //Also, consecutive horz. edges may start heading left before going right.
            if LeftBoundIsForward {
                EStart = self.edges[E].Prev;
            } else {
                EStart = self.edges[E].Next;
            }
            if self.edges[EStart].Dx == horizontal {
                //ie an adjoining horizontal skip edge
                if self.edges[EStart].Bot.X != self.edges[E].Bot.X && self.edges[EStart].Top.X != self.edges[E].Bot.X {
                    self.ReverseHorizontal(E);
                }
            } else if self.edges[EStart].Bot.X != self.edges[E].Bot.X {
                self.ReverseHorizontal(E);
            }
        }

        EStart = E;
        if LeftBoundIsForward {
            while self.edges[Result].Top.Y == self.edges[self.edges[Result].Next].Bot.Y && self.edges[self.edges[Result].Next].OutIdx != Skip {
                Result = self.edges[Result].Next;
            }
            if self.edges[Result].Dx == horizontal && self.edges[self.edges[Result].Next].OutIdx != Skip {
                //nb: at the top of a bound, horizontals are added to the bound
                //only when the preceding edge attaches to the horizontal's left vertex
                //unless a Skip edge is encountered when that becomes the top divide
                let mut Horz = Result;
                while self.edges[self.edges[Horz].Prev].Dx == horizontal {
                    Horz = self.edges[Horz].Prev;
                }
                if self.edges[self.edges[Horz].Prev].Top.X > self.edges[self.edges[Result].Next].Top.X {
                    Result = self.edges[Horz].Prev;
                }
            }
            while E != Result {
                self.edges[E].NextInLML = self.edges[E].Next;
                if self.edges[E].Dx == horizontal && E != EStart && self.edges[E].Bot.X != self.edges[self.edges[E].Prev].Top.X {
                    self.ReverseHorizontal(E);
                }
                E = self.edges[E].Next;
            }
            if self.edges[E].Dx == horizontal && E != EStart && self.edges[E].Bot.X != self.edges[self.edges[E].Prev].Top.X {
                self.ReverseHorizontal(E);
            }
            Result = self.edges[Result].Next; //move to the edge just beyond current bound
        } else {
            while self.edges[Result].Top.Y == self.edges[self.edges[Result].Prev].Bot.Y && self.edges[self.edges[Result].Prev].OutIdx != Skip {
                Result = self.edges[Result].Prev;
            }
            if self.edges[Result].Dx == horizontal && self.edges[self.edges[Result].Prev].OutIdx != Skip {
                let mut Horz = Result;
                while self.edges[self.edges[Horz].Next].Dx == horizontal {
                    Horz = self.edges[Horz].Next;
                }
                let hn = self.edges[self.edges[Horz].Next].Top.X;
                let rp = self.edges[self.edges[Result].Prev].Top.X;
                if hn == rp || hn > rp {
                    Result = self.edges[Horz].Next;
                }
            }

            while E != Result {
                self.edges[E].NextInLML = self.edges[E].Prev;
                if self.edges[E].Dx == horizontal && E != EStart && self.edges[E].Bot.X != self.edges[self.edges[E].Next].Top.X {
                    self.ReverseHorizontal(E);
                }
                E = self.edges[E].Prev;
            }
            if self.edges[E].Dx == horizontal && E != EStart && self.edges[E].Bot.X != self.edges[self.edges[E].Next].Top.X {
                self.ReverseHorizontal(E);
            }
            Result = self.edges[Result].Prev; //move to the edge just beyond current bound
        }
        Result
    }

    fn new_lm(&mut self, lm: LocalMinima) -> usize {
        self.lms.push(lm);
        self.lms.len() - 1
    }

    /// `AddPath(Path pg, PolyType polyType, bool Closed)`
    pub fn AddPath(&mut self, pg: &Path, polyType: PolyType, Closed: bool) -> Result<bool, Exception> {
        if !Closed && polyType == PolyType::ptClip {
            return Err(clipper_exception("AddPath: Open paths must be subject."));
        }

        let mut highI = pg.len() as i32 - 1;
        if Closed {
            while highI > 0 && (pg[highI as usize] == pg[0]) {
                highI -= 1;
            }
        }
        while highI > 0 && (pg[highI as usize] == pg[highI as usize - 1]) {
            highI -= 1;
        }
        if (Closed && highI < 2) || (!Closed && highI < 1) {
            return Ok(false);
        }
        let highI = highI as usize;

        //create a new edge array ...
        let base = self.edges.len();
        for _ in 0..=highI {
            self.edges.push(TEdge::default());
        }
        let edges: Vec<usize> = (base..=base + highI).collect();

        let mut IsFlat = true;

        //1. Basic (first) edge initialization ...
        self.edges[edges[1]].Curr = pg[1];
        let mut ufr = self.m_UseFullRange;
        let r = (|| -> Result<(), Exception> {
            Self::RangeTest(pg[0], &mut ufr)?;
            Self::RangeTest(pg[highI], &mut ufr)?;
            Ok(())
        })();
        self.m_UseFullRange = ufr;
        if let Err(e) = r {
            self.edges.truncate(base);
            return Err(e);
        }
        self.InitEdge(edges[0], edges[1], edges[highI], pg[0]);
        self.InitEdge(edges[highI], edges[0], edges[highI - 1], pg[highI]);
        for i in (1..highI).rev() {
            let mut ufr = self.m_UseFullRange;
            let r = Self::RangeTest(pg[i], &mut ufr);
            self.m_UseFullRange = ufr;
            if let Err(e) = r {
                self.edges.truncate(base);
                return Err(e);
            }
            self.InitEdge(edges[i], edges[i + 1], edges[i - 1], pg[i]);
        }
        let mut eStart = edges[0];

        //2. Remove duplicate vertices, and (when closed) collinear edges ...
        let mut E = eStart;
        let mut eLoopStop = eStart;
        loop {
            //nb: allows matching start and end points when not Closed ...
            if self.edges[E].Curr == self.edges[self.edges[E].Next].Curr && (Closed || self.edges[E].Next != eStart) {
                if E == self.edges[E].Next {
                    break;
                }
                if E == eStart {
                    eStart = self.edges[E].Next;
                }
                E = self.RemoveEdge(E);
                eLoopStop = E;
                continue;
            }
            if self.edges[E].Prev == self.edges[E].Next {
                break; //only two vertices
            } else if Closed
                && Self::SlopesEqual3(self.edges[self.edges[E].Prev].Curr, self.edges[E].Curr, self.edges[self.edges[E].Next].Curr, self.m_UseFullRange)
                && (!self.PreserveCollinear
                    || !Self::Pt2IsBetweenPt1AndPt3(self.edges[self.edges[E].Prev].Curr, self.edges[E].Curr, self.edges[self.edges[E].Next].Curr))
            {
                //Collinear edges are allowed for open paths but in closed paths
                //the default is to merge adjacent collinear edges into a single edge.
                //However, if the PreserveCollinear property is enabled, only overlapping
                //collinear edges (ie spikes) will be removed from closed paths.
                if E == eStart {
                    eStart = self.edges[E].Next;
                }
                E = self.RemoveEdge(E);
                E = self.edges[E].Prev;
                eLoopStop = E;
                continue;
            }
            E = self.edges[E].Next;
            if (E == eLoopStop) || (!Closed && self.edges[E].Next == eStart) {
                break;
            }
        }

        if (!Closed && (E == self.edges[E].Next)) || (Closed && (self.edges[E].Prev == self.edges[E].Next)) {
            return Ok(false);
        }

        if !Closed {
            self.m_HasOpenPaths = true;
            let p = self.edges[eStart].Prev;
            self.edges[p].OutIdx = Skip;
        }

        //3. Do second stage of edge initialization ...
        E = eStart;
        loop {
            self.InitEdge2(E, polyType);
            E = self.edges[E].Next;
            if IsFlat && self.edges[E].Curr.Y != self.edges[eStart].Curr.Y {
                IsFlat = false;
            }
            if E == eStart {
                break;
            }
        }

        //4. Finally, add edge bounds to LocalMinima list ...

        //Totally flat paths must be handled differently when adding them
        //to LocalMinima list to avoid endless loops etc ...
        if IsFlat {
            if Closed {
                return Ok(false);
            }
            let p = self.edges[E].Prev;
            self.edges[p].OutIdx = Skip;
            let locMin = self.new_lm(LocalMinima { Y: self.edges[E].Bot.Y, LeftBound: NIL, RightBound: E, Next: NIL });
            self.edges[E].Side = EdgeSide::esRight;
            self.edges[E].WindDelta = 0;
            loop {
                if self.edges[E].Bot.X != self.edges[self.edges[E].Prev].Top.X {
                    self.ReverseHorizontal(E);
                }
                if self.edges[self.edges[E].Next].OutIdx == Skip {
                    break;
                }
                self.edges[E].NextInLML = self.edges[E].Next;
                E = self.edges[E].Next;
            }
            self.InsertLocalMinima(locMin);
            self.m_edges.push(edges);
            return Ok(true);
        }

        self.m_edges.push(edges);
        let mut leftBoundIsForward;
        let mut EMin = NIL;

        //workaround to avoid an endless loop in the while loop below when
        //open paths have matching start and end points ...
        if self.edges[self.edges[E].Prev].Bot == self.edges[self.edges[E].Prev].Top {
            E = self.edges[E].Next;
        }

        loop {
            E = self.FindNextLocMin(E);
            if E == EMin {
                break;
            } else if EMin == NIL {
                EMin = E;
            }

            //E and E.Prev now share a local minima (left aligned if horizontal).
            //Compare their slopes to find which starts which bound ...
            let (lb, rb);
            if self.edges[E].Dx < self.edges[self.edges[E].Prev].Dx {
                lb = self.edges[E].Prev;
                rb = E;
                leftBoundIsForward = false; //Q.nextInLML = Q.prev
            } else {
                lb = E;
                rb = self.edges[E].Prev;
                leftBoundIsForward = true; //Q.nextInLML = Q.next
            }
            let locMin = self.new_lm(LocalMinima { Y: self.edges[E].Bot.Y, LeftBound: lb, RightBound: rb, Next: NIL });
            self.edges[lb].Side = EdgeSide::esLeft;
            self.edges[rb].Side = EdgeSide::esRight;

            if !Closed {
                self.edges[lb].WindDelta = 0;
            } else if self.edges[lb].Next == rb {
                self.edges[lb].WindDelta = -1;
            } else {
                self.edges[lb].WindDelta = 1;
            }
            self.edges[rb].WindDelta = -self.edges[lb].WindDelta;

            E = self.ProcessBound(lb, leftBoundIsForward);
            if self.edges[E].OutIdx == Skip {
                E = self.ProcessBound(E, leftBoundIsForward);
            }

            let mut E2 = self.ProcessBound(rb, !leftBoundIsForward);
            if self.edges[E2].OutIdx == Skip {
                E2 = self.ProcessBound(E2, !leftBoundIsForward);
            }

            if self.edges[lb].OutIdx == Skip {
                self.lms[locMin].LeftBound = NIL;
            } else if self.edges[rb].OutIdx == Skip {
                self.lms[locMin].RightBound = NIL;
            }
            self.InsertLocalMinima(locMin);
            if !leftBoundIsForward {
                E = E2;
            }
        }
        Ok(true)
    }

    /// `AddPaths(Paths ppg, PolyType polyType, bool closed)`
    pub fn AddPaths(&mut self, ppg: &Paths, polyType: PolyType, closed: bool) -> Result<bool, Exception> {
        let mut result = false;
        for p in ppg {
            if self.AddPath(p, polyType, closed)? {
                result = true;
            }
        }
        Ok(result)
    }

    fn Pt2IsBetweenPt1AndPt3(pt1: IntPoint, pt2: IntPoint, pt3: IntPoint) -> bool {
        if (pt1 == pt3) || (pt1 == pt2) || (pt3 == pt2) {
            false
        } else if pt1.X != pt3.X {
            (pt2.X > pt1.X) == (pt2.X < pt3.X)
        } else {
            (pt2.Y > pt1.Y) == (pt2.Y < pt3.Y)
        }
    }

    /// removes e from double_linked_list (but without removing from memory)
    fn RemoveEdge(&mut self, e: usize) -> usize {
        let (prev, next) = (self.edges[e].Prev, self.edges[e].Next);
        self.edges[prev].Next = next;
        self.edges[next].Prev = prev;
        let result = next;
        self.edges[e].Prev = NIL; //flag as removed (see ClipperBase.Clear)
        result
    }

    fn SetDx(&mut self, e: usize) {
        let ed = &mut self.edges[e];
        ed.Delta.X = ed.Top.X.wrapping_sub(ed.Bot.X);
        ed.Delta.Y = ed.Top.Y.wrapping_sub(ed.Bot.Y);
        if ed.Delta.Y == 0 {
            ed.Dx = horizontal;
        } else {
            ed.Dx = (ed.Delta.X as f64) / (ed.Delta.Y as f64);
        }
    }

    fn InsertLocalMinima(&mut self, newLm: usize) {
        if self.m_MinimaList == NIL {
            self.m_MinimaList = newLm;
        } else if self.lms[newLm].Y >= self.lms[self.m_MinimaList].Y {
            self.lms[newLm].Next = self.m_MinimaList;
            self.m_MinimaList = newLm;
        } else {
            let mut tmpLm = self.m_MinimaList;
            while self.lms[tmpLm].Next != NIL && (self.lms[newLm].Y < self.lms[self.lms[tmpLm].Next].Y) {
                tmpLm = self.lms[tmpLm].Next;
            }
            self.lms[newLm].Next = self.lms[tmpLm].Next;
            self.lms[tmpLm].Next = newLm;
        }
    }

    fn PopLocalMinima(&mut self, Y: cInt, current: &mut usize) -> bool {
        *current = self.m_CurrentLM;
        if self.m_CurrentLM != NIL && self.lms[self.m_CurrentLM].Y == Y {
            self.m_CurrentLM = self.lms[self.m_CurrentLM].Next;
            return true;
        }
        false
    }

    /// swap horizontal edges' top and bottom x's so they follow the natural progression of
    /// the bounds - ie so their xbots will align with the adjoining lower edge.
    fn ReverseHorizontal(&mut self, e: usize) {
        let ed = &mut self.edges[e];
        std::mem::swap(&mut ed.Top.X, &mut ed.Bot.X);
    }

    /// `ClipperBase.Reset()`
    fn ResetBase(&mut self) {
        self.m_CurrentLM = self.m_MinimaList;
        if self.m_CurrentLM == NIL {
            return; //ie nothing to process
        }

        //reset all edges ...
        self.m_Scanbeam = NIL;
        let mut lm = self.m_MinimaList;
        while lm != NIL {
            let l = self.lms[lm];
            self.InsertScanbeam(l.Y);
            if l.LeftBound != NIL {
                let ed = &mut self.edges[l.LeftBound];
                ed.Curr = ed.Bot;
                ed.OutIdx = Unassigned;
            }
            if l.RightBound != NIL {
                let ed = &mut self.edges[l.RightBound];
                ed.Curr = ed.Bot;
                ed.OutIdx = Unassigned;
            }
            lm = l.Next;
        }
        self.m_ActiveEdges = NIL;
    }

    /// `ClipperBase.GetBounds(Paths paths)`
    pub fn GetBounds(paths: &Paths) -> IntRect {
        let cnt = paths.len();
        let mut i = 0;
        while i < cnt && paths[i].is_empty() {
            i += 1;
        }
        if i == cnt {
            return IntRect::new(0, 0, 0, 0);
        }
        let mut result = IntRect::default();
        result.left = paths[i][0].X;
        result.right = result.left;
        result.top = paths[i][0].Y;
        result.bottom = result.top;
        while i < cnt {
            for p in &paths[i] {
                if p.X < result.left {
                    result.left = p.X;
                } else if p.X > result.right {
                    result.right = p.X;
                }
                if p.Y < result.top {
                    result.top = p.Y;
                } else if p.Y > result.bottom {
                    result.bottom = p.Y;
                }
            }
            i += 1;
        }
        result
    }

    /// single-linked list: sorted descending, ignoring dups.
    fn InsertScanbeam(&mut self, Y: cInt) {
        if self.m_Scanbeam == NIL {
            self.sbs.push(Scanbeam { Y, Next: NIL });
            self.m_Scanbeam = self.sbs.len() - 1;
        } else if Y > self.sbs[self.m_Scanbeam].Y {
            self.sbs.push(Scanbeam { Y, Next: self.m_Scanbeam });
            self.m_Scanbeam = self.sbs.len() - 1;
        } else {
            let mut sb2 = self.m_Scanbeam;
            while self.sbs[sb2].Next != NIL && (Y <= self.sbs[self.sbs[sb2].Next].Y) {
                sb2 = self.sbs[sb2].Next;
            }
            if Y == self.sbs[sb2].Y {
                return; //ie ignores duplicates
            }
            self.sbs.push(Scanbeam { Y, Next: self.sbs[sb2].Next });
            let n = self.sbs.len() - 1;
            self.sbs[sb2].Next = n;
        }
    }

    fn PopScanbeam(&mut self, Y: &mut cInt) -> bool {
        if self.m_Scanbeam == NIL {
            *Y = 0;
            return false;
        }
        *Y = self.sbs[self.m_Scanbeam].Y;
        self.m_Scanbeam = self.sbs[self.m_Scanbeam].Next;
        true
    }

    fn LocalMinimaPending(&self) -> bool {
        self.m_CurrentLM != NIL
    }

    fn CreateOutRec(&mut self) -> usize {
        self.outrecs.push(OutRec { Idx: Unassigned, IsHole: false, IsOpen: false, FirstLeft: NIL, Pts: NIL, BottomPt: NIL, PolyNode: NIL });
        let r = self.outrecs.len() - 1;
        self.m_PolyOuts.push(r);
        self.outrecs[r].Idx = self.m_PolyOuts.len() as i32 - 1;
        r
    }

    fn DisposeOutRec(&mut self, index: usize) {
        let o = self.m_PolyOuts[index];
        if o != NIL {
            self.outrecs[o].Pts = NIL;
        }
        self.m_PolyOuts[index] = NIL;
    }

    fn UpdateEdgeIntoAEL(&mut self, e: &mut usize) -> Result<(), Exception> {
        let ee = *e;
        let nl = self.edges[ee].NextInLML;
        if nl == NIL {
            return Err(clipper_exception("UpdateEdgeIntoAEL: invalid call"));
        }
        let AelPrev = self.edges[ee].PrevInAEL;
        let AelNext = self.edges[ee].NextInAEL;
        self.edges[nl].OutIdx = self.edges[ee].OutIdx;
        if AelPrev != NIL {
            self.edges[AelPrev].NextInAEL = nl;
        } else {
            self.m_ActiveEdges = nl;
        }
        if AelNext != NIL {
            self.edges[AelNext].PrevInAEL = nl;
        }
        self.edges[nl].Side = self.edges[ee].Side;
        self.edges[nl].WindDelta = self.edges[ee].WindDelta;
        self.edges[nl].WindCnt = self.edges[ee].WindCnt;
        self.edges[nl].WindCnt2 = self.edges[ee].WindCnt2;
        *e = nl;
        let ed = &mut self.edges[nl];
        ed.Curr = ed.Bot;
        ed.PrevInAEL = AelPrev;
        ed.NextInAEL = AelNext;
        if !self.IsHorizontal(nl) {
            let y = self.edges[nl].Top.Y;
            self.InsertScanbeam(y);
        }
        Ok(())
    }

    fn SwapPositionsInAEL(&mut self, edge1: usize, edge2: usize) {
        //check that one or other edge hasn't already been removed from AEL ...
        if self.edges[edge1].NextInAEL == self.edges[edge1].PrevInAEL || self.edges[edge2].NextInAEL == self.edges[edge2].PrevInAEL {
            return;
        }

        if self.edges[edge1].NextInAEL == edge2 {
            let next = self.edges[edge2].NextInAEL;
            if next != NIL {
                self.edges[next].PrevInAEL = edge1;
            }
            let prev = self.edges[edge1].PrevInAEL;
            if prev != NIL {
                self.edges[prev].NextInAEL = edge2;
            }
            self.edges[edge2].PrevInAEL = prev;
            self.edges[edge2].NextInAEL = edge1;
            self.edges[edge1].PrevInAEL = edge2;
            self.edges[edge1].NextInAEL = next;
        } else if self.edges[edge2].NextInAEL == edge1 {
            let next = self.edges[edge1].NextInAEL;
            if next != NIL {
                self.edges[next].PrevInAEL = edge2;
            }
            let prev = self.edges[edge2].PrevInAEL;
            if prev != NIL {
                self.edges[prev].NextInAEL = edge1;
            }
            self.edges[edge1].PrevInAEL = prev;
            self.edges[edge1].NextInAEL = edge2;
            self.edges[edge2].PrevInAEL = edge1;
            self.edges[edge2].NextInAEL = next;
        } else {
            let next = self.edges[edge1].NextInAEL;
            let prev = self.edges[edge1].PrevInAEL;
            self.edges[edge1].NextInAEL = self.edges[edge2].NextInAEL;
            if self.edges[edge1].NextInAEL != NIL {
                let n = self.edges[edge1].NextInAEL;
                self.edges[n].PrevInAEL = edge1;
            }
            self.edges[edge1].PrevInAEL = self.edges[edge2].PrevInAEL;
            if self.edges[edge1].PrevInAEL != NIL {
                let p = self.edges[edge1].PrevInAEL;
                self.edges[p].NextInAEL = edge1;
            }
            self.edges[edge2].NextInAEL = next;
            if self.edges[edge2].NextInAEL != NIL {
                let n = self.edges[edge2].NextInAEL;
                self.edges[n].PrevInAEL = edge2;
            }
            self.edges[edge2].PrevInAEL = prev;
            if self.edges[edge2].PrevInAEL != NIL {
                let p = self.edges[edge2].PrevInAEL;
                self.edges[p].NextInAEL = edge2;
            }
        }

        if self.edges[edge1].PrevInAEL == NIL {
            self.m_ActiveEdges = edge1;
        } else if self.edges[edge2].PrevInAEL == NIL {
            self.m_ActiveEdges = edge2;
        }
    }

    fn DeleteFromAEL(&mut self, e: usize) {
        let AelPrev = self.edges[e].PrevInAEL;
        let AelNext = self.edges[e].NextInAEL;
        if AelPrev == NIL && AelNext == NIL && (e != self.m_ActiveEdges) {
            return; //already deleted
        }
        if AelPrev != NIL {
            self.edges[AelPrev].NextInAEL = AelNext;
        } else {
            self.m_ActiveEdges = AelNext;
        }
        if AelNext != NIL {
            self.edges[AelNext].PrevInAEL = AelPrev;
        }
        self.edges[e].NextInAEL = NIL;
        self.edges[e].PrevInAEL = NIL;
    }

    // ---------------------------------------------------------------- Clipper

    /// double-linked list: sorted ascending, ignoring dups.
    fn InsertMaxima(&mut self, X: cInt) {
        self.maxs.push(Maxima { X, Next: NIL, Prev: NIL });
        let newMax = self.maxs.len() - 1;
        if self.m_Maxima == NIL {
            self.m_Maxima = newMax;
        } else if X < self.maxs[self.m_Maxima].X {
            self.maxs[newMax].Next = self.m_Maxima;
            self.maxs[newMax].Prev = NIL;
            self.m_Maxima = newMax;
        } else {
            let mut m = self.m_Maxima;
            while self.maxs[m].Next != NIL && (X >= self.maxs[self.maxs[m].Next].X) {
                m = self.maxs[m].Next;
            }
            if X == self.maxs[m].X {
                return; //ie ignores duplicates
            }
            //insert newMax between m and m.Next ...
            let mn = self.maxs[m].Next;
            self.maxs[newMax].Next = mn;
            self.maxs[newMax].Prev = m;
            if mn != NIL {
                self.maxs[mn].Prev = newMax;
            }
            self.maxs[m].Next = newMax;
        }
    }

    /// `Execute(ClipType clipType, Paths solution, PolyFillType FillType = pftEvenOdd)`
    pub fn Execute(&mut self, clipType: ClipType, solution: &mut Paths, FillType: PolyFillType) -> Result<bool, Exception> {
        self.Execute_paths(clipType, solution, FillType, FillType)
    }

    /// `Execute(ClipType clipType, PolyTree polytree, PolyFillType FillType = pftEvenOdd)`
    pub fn Execute_tree(&mut self, clipType: ClipType, polytree: &mut PolyTree, FillType: PolyFillType) -> Result<bool, Exception> {
        self.Execute_tree_fill(clipType, polytree, FillType, FillType)
    }

    /// `Execute(ClipType clipType, Paths solution, PolyFillType subjFillType, PolyFillType clipFillType)`
    pub fn Execute_paths(&mut self, clipType: ClipType, solution: &mut Paths, subjFillType: PolyFillType, clipFillType: PolyFillType) -> Result<bool, Exception> {
        if self.m_ExecuteLocked {
            return Ok(false);
        }
        if self.m_HasOpenPaths {
            return Err(clipper_exception("Error: PolyTree struct is needed for open path clipping."));
        }

        self.m_ExecuteLocked = true;
        solution.clear();
        self.m_SubjFillType = subjFillType;
        self.m_ClipFillType = clipFillType;
        self.m_ClipType = clipType;
        self.m_UsingPolyTree = false;
        let r = (|| -> Result<bool, Exception> {
            let succeeded = self.ExecuteInternal()?;
            //build the return polygons ...
            if succeeded {
                self.BuildResult(solution);
            }
            Ok(succeeded)
        })();
        // finally
        self.DisposeAllPolyPts();
        self.m_ExecuteLocked = false;
        r
    }

    /// `Execute(ClipType clipType, PolyTree polytree, PolyFillType subjFillType, PolyFillType clipFillType)`
    pub fn Execute_tree_fill(&mut self, clipType: ClipType, polytree: &mut PolyTree, subjFillType: PolyFillType, clipFillType: PolyFillType) -> Result<bool, Exception> {
        if self.m_ExecuteLocked {
            return Ok(false);
        }
        self.m_ExecuteLocked = true;
        self.m_SubjFillType = subjFillType;
        self.m_ClipFillType = clipFillType;
        self.m_ClipType = clipType;
        self.m_UsingPolyTree = true;
        let r = (|| -> Result<bool, Exception> {
            let succeeded = self.ExecuteInternal()?;
            //build the return polygons ...
            if succeeded {
                self.BuildResult2(polytree);
            }
            Ok(succeeded)
        })();
        // finally
        self.DisposeAllPolyPts();
        self.m_ExecuteLocked = false;
        r
    }

    fn FixHoleLinkage(&mut self, outRec: usize) {
        //skip if an outermost polygon or
        //already already points to the correct FirstLeft ...
        let fl = self.outrecs[outRec].FirstLeft;
        if fl == NIL || (self.outrecs[outRec].IsHole != self.outrecs[fl].IsHole && self.outrecs[fl].Pts != NIL) {
            return;
        }

        let mut orfl = fl;
        while orfl != NIL && ((self.outrecs[orfl].IsHole == self.outrecs[outRec].IsHole) || self.outrecs[orfl].Pts == NIL) {
            orfl = self.outrecs[orfl].FirstLeft;
        }
        self.outrecs[outRec].FirstLeft = orfl;
    }

    fn ExecuteInternal(&mut self) -> Result<bool, Exception> {
        let r = (|| -> Result<bool, Exception> {
            self.ResetBase();
            self.m_SortedEdges = NIL;
            self.m_Maxima = NIL;

            let mut botY: cInt = 0;
            let mut topY: cInt = 0;
            if !self.PopScanbeam(&mut botY) {
                return Ok(false);
            }
            self.InsertLocalMinimaIntoAEL(botY)?;
            while self.PopScanbeam(&mut topY) || self.LocalMinimaPending() {
                self.ProcessHorizontals()?;
                self.m_GhostJoins.clear();
                if !self.ProcessIntersections(topY)? {
                    return Ok(false);
                }
                self.ProcessEdgesAtTopOfScanbeam(topY)?;
                botY = topY;
                self.InsertLocalMinimaIntoAEL(botY)?;
            }

            //fix orientations ...
            for i in 0..self.m_PolyOuts.len() {
                let outRec = self.m_PolyOuts[i];
                if outRec == NIL || self.outrecs[outRec].Pts == NIL || self.outrecs[outRec].IsOpen {
                    continue;
                }
                if (self.outrecs[outRec].IsHole ^ self.ReverseSolution) == (self.AreaRec(outRec) > 0.0) {
                    let p = self.outrecs[outRec].Pts;
                    self.ReversePolyPtLinks(p);
                }
            }

            self.JoinCommonEdges();

            for i in 0..self.m_PolyOuts.len() {
                let outRec = self.m_PolyOuts[i];
                if outRec == NIL || self.outrecs[outRec].Pts == NIL {
                    continue;
                } else if self.outrecs[outRec].IsOpen {
                    self.FixupOutPolyline(outRec);
                } else {
                    self.FixupOutPolygon(outRec);
                }
            }

            if self.StrictlySimple {
                self.DoSimplePolygons();
            }
            Ok(true)
        })();
        // finally
        self.m_Joins.clear();
        self.m_GhostJoins.clear();
        r
    }

    fn DisposeAllPolyPts(&mut self) {
        for i in 0..self.m_PolyOuts.len() {
            self.DisposeOutRec(i);
        }
        self.m_PolyOuts.clear();
        self.outrecs.clear();
        self.outpts.clear();
        self.maxs.clear();
        self.sbs.clear();
    }

    fn AddJoin(&mut self, Op1: usize, Op2: usize, OffPt: IntPoint) {
        self.m_Joins.push(Join { OutPt1: Op1, OutPt2: Op2, OffPt });
    }

    fn AddGhostJoin(&mut self, Op: usize, OffPt: IntPoint) {
        self.m_GhostJoins.push(Join { OutPt1: Op, OutPt2: NIL, OffPt });
    }

    fn InsertLocalMinimaIntoAEL(&mut self, botY: cInt) -> Result<(), Exception> {
        let mut lm = NIL;
        while self.PopLocalMinima(botY, &mut lm) {
            let lb = self.lms[lm].LeftBound;
            let rb = self.lms[lm].RightBound;

            let mut Op1 = NIL;
            if lb == NIL {
                self.InsertEdgeIntoAEL(rb, NIL);
                self.SetWindingCount(rb);
                if self.IsContributing(rb) {
                    let b = self.edges[rb].Bot;
                    Op1 = self.AddOutPt(rb, b);
                }
            } else if rb == NIL {
                self.InsertEdgeIntoAEL(lb, NIL);
                self.SetWindingCount(lb);
                if self.IsContributing(lb) {
                    let b = self.edges[lb].Bot;
                    Op1 = self.AddOutPt(lb, b);
                }
                let y = self.edges[lb].Top.Y;
                self.InsertScanbeam(y);
            } else {
                self.InsertEdgeIntoAEL(lb, NIL);
                self.InsertEdgeIntoAEL(rb, lb);
                self.SetWindingCount(lb);
                self.edges[rb].WindCnt = self.edges[lb].WindCnt;
                self.edges[rb].WindCnt2 = self.edges[lb].WindCnt2;
                if self.IsContributing(lb) {
                    let b = self.edges[lb].Bot;
                    Op1 = self.AddLocalMinPoly(lb, rb, b);
                }
                let y = self.edges[lb].Top.Y;
                self.InsertScanbeam(y);
            }

            if rb != NIL {
                if self.IsHorizontal(rb) {
                    let nl = self.edges[rb].NextInLML;
                    if nl != NIL {
                        let y = self.edges[nl].Top.Y;
                        self.InsertScanbeam(y);
                    }
                    self.AddEdgeToSEL(rb);
                } else {
                    let y = self.edges[rb].Top.Y;
                    self.InsertScanbeam(y);
                }
            }

            if lb == NIL || rb == NIL {
                continue;
            }

            //if output polygons share an Edge with a horizontal rb, they'll need joining later ...
            if Op1 != NIL && self.IsHorizontal(rb) && !self.m_GhostJoins.is_empty() && self.edges[rb].WindDelta != 0 {
                for i in 0..self.m_GhostJoins.len() {
                    //if the horizontal Rb and a 'ghost' horizontal overlap, then convert
                    //the 'ghost' join to a real join ready for later ...
                    let j = self.m_GhostJoins[i];
                    if Self::HorzSegmentsOverlap(self.outpts[j.OutPt1].Pt.X, j.OffPt.X, self.edges[rb].Bot.X, self.edges[rb].Top.X) {
                        self.AddJoin(j.OutPt1, Op1, j.OffPt);
                    }
                }
            }

            let lbp = self.edges[lb].PrevInAEL;
            if self.edges[lb].OutIdx >= 0
                && lbp != NIL
                && self.edges[lbp].Curr.X == self.edges[lb].Bot.X
                && self.edges[lbp].OutIdx >= 0
                && Self::SlopesEqual4(self.edges[lbp].Curr, self.edges[lbp].Top, self.edges[lb].Curr, self.edges[lb].Top, self.m_UseFullRange)
                && self.edges[lb].WindDelta != 0
                && self.edges[lbp].WindDelta != 0
            {
                let b = self.edges[lb].Bot;
                let Op2 = self.AddOutPt(lbp, b);
                let t = self.edges[lb].Top;
                self.AddJoin(Op1, Op2, t);
            }

            if self.edges[lb].NextInAEL != rb {
                let rbp = self.edges[rb].PrevInAEL;
                if self.edges[rb].OutIdx >= 0
                    && self.edges[rbp].OutIdx >= 0
                    && Self::SlopesEqual4(self.edges[rbp].Curr, self.edges[rbp].Top, self.edges[rb].Curr, self.edges[rb].Top, self.m_UseFullRange)
                    && self.edges[rb].WindDelta != 0
                    && self.edges[rbp].WindDelta != 0
                {
                    let b = self.edges[rb].Bot;
                    let Op2 = self.AddOutPt(rbp, b);
                    let t = self.edges[rb].Top;
                    self.AddJoin(Op1, Op2, t);
                }

                let mut e = self.edges[lb].NextInAEL;
                if e != NIL {
                    while e != rb {
                        //nb: For calculating winding counts etc, IntersectEdges() assumes
                        //that param1 will be to the right of param2 ABOVE the intersection ...
                        let c = self.edges[lb].Curr;
                        self.IntersectEdges(rb, e, c); //order important here
                        e = self.edges[e].NextInAEL;
                    }
                }
            }
        }
        Ok(())
    }

    fn InsertEdgeIntoAEL(&mut self, edge: usize, mut startEdge: usize) {
        if self.m_ActiveEdges == NIL {
            self.edges[edge].PrevInAEL = NIL;
            self.edges[edge].NextInAEL = NIL;
            self.m_ActiveEdges = edge;
        } else if startEdge == NIL && self.E2InsertsBeforeE1(self.m_ActiveEdges, edge) {
            self.edges[edge].PrevInAEL = NIL;
            self.edges[edge].NextInAEL = self.m_ActiveEdges;
            let ae = self.m_ActiveEdges;
            self.edges[ae].PrevInAEL = edge;
            self.m_ActiveEdges = edge;
        } else {
            if startEdge == NIL {
                startEdge = self.m_ActiveEdges;
            }
            while self.edges[startEdge].NextInAEL != NIL && !self.E2InsertsBeforeE1(self.edges[startEdge].NextInAEL, edge) {
                startEdge = self.edges[startEdge].NextInAEL;
            }
            self.edges[edge].NextInAEL = self.edges[startEdge].NextInAEL;
            let sn = self.edges[startEdge].NextInAEL;
            if sn != NIL {
                self.edges[sn].PrevInAEL = edge;
            }
            self.edges[edge].PrevInAEL = startEdge;
            self.edges[startEdge].NextInAEL = edge;
        }
    }

    fn E2InsertsBeforeE1(&self, e1: usize, e2: usize) -> bool {
        let (a, b) = (&self.edges[e1], &self.edges[e2]);
        if b.Curr.X == a.Curr.X {
            if b.Top.Y > a.Top.Y {
                b.Top.X < Self::TopX(a, b.Top.Y)
            } else {
                a.Top.X > Self::TopX(b, a.Top.Y)
            }
        } else {
            b.Curr.X < a.Curr.X
        }
    }

    fn IsEvenOddFillType(&self, edge: usize) -> bool {
        if self.edges[edge].PolyTyp == PolyType::ptSubject {
            self.m_SubjFillType == PolyFillType::pftEvenOdd
        } else {
            self.m_ClipFillType == PolyFillType::pftEvenOdd
        }
    }

    fn IsEvenOddAltFillType(&self, edge: usize) -> bool {
        if self.edges[edge].PolyTyp == PolyType::ptSubject {
            self.m_ClipFillType == PolyFillType::pftEvenOdd
        } else {
            self.m_SubjFillType == PolyFillType::pftEvenOdd
        }
    }

    fn IsContributing(&self, edge: usize) -> bool {
        let e = &self.edges[edge];
        let (pft, pft2) = if e.PolyTyp == PolyType::ptSubject {
            (self.m_SubjFillType, self.m_ClipFillType)
        } else {
            (self.m_ClipFillType, self.m_SubjFillType)
        };

        match pft {
            PolyFillType::pftEvenOdd => {
                //return false if a subj line has been flagged as inside a subj polygon
                if e.WindDelta == 0 && e.WindCnt != 1 {
                    return false;
                }
            }
            PolyFillType::pftNonZero => {
                if e.WindCnt.wrapping_abs() != 1 {
                    return false;
                }
            }
            PolyFillType::pftPositive => {
                if e.WindCnt != 1 {
                    return false;
                }
            }
            PolyFillType::pftNegative => {
                if e.WindCnt != -1 {
                    return false;
                }
            }
        }

        let zero_side = |pft2: PolyFillType| match pft2 {
            PolyFillType::pftEvenOdd | PolyFillType::pftNonZero => e.WindCnt2 == 0,
            PolyFillType::pftPositive => e.WindCnt2 <= 0,
            PolyFillType::pftNegative => e.WindCnt2 >= 0,
        };
        let inside = |pft2: PolyFillType| match pft2 {
            PolyFillType::pftEvenOdd | PolyFillType::pftNonZero => e.WindCnt2 != 0,
            PolyFillType::pftPositive => e.WindCnt2 > 0,
            PolyFillType::pftNegative => e.WindCnt2 < 0,
        };
        match self.m_ClipType {
            ClipType::ctIntersection => inside(pft2),
            ClipType::ctUnion => zero_side(pft2),
            ClipType::ctDifference => {
                if e.PolyTyp == PolyType::ptSubject {
                    zero_side(pft2)
                } else {
                    inside(pft2)
                }
            }
            ClipType::ctXor => {
                if e.WindDelta == 0 {
                    //XOr always contributing unless open
                    zero_side(pft2)
                } else {
                    true
                }
            }
        }
    }

    fn SetWindingCount(&mut self, edge: usize) {
        let mut e = self.edges[edge].PrevInAEL;
        //find the edge of the same polytype that immediately preceeds 'edge' in AEL
        while e != NIL && ((self.edges[e].PolyTyp != self.edges[edge].PolyTyp) || (self.edges[e].WindDelta == 0)) {
            e = self.edges[e].PrevInAEL;
        }
        if e == NIL {
            let pft = if self.edges[edge].PolyTyp == PolyType::ptSubject { self.m_SubjFillType } else { self.m_ClipFillType };
            if self.edges[edge].WindDelta == 0 {
                self.edges[edge].WindCnt = if pft == PolyFillType::pftNegative { -1 } else { 1 };
            } else {
                self.edges[edge].WindCnt = self.edges[edge].WindDelta;
            }
            self.edges[edge].WindCnt2 = 0;
            e = self.m_ActiveEdges; //ie get ready to calc WindCnt2
        } else if self.edges[edge].WindDelta == 0 && self.m_ClipType != ClipType::ctUnion {
            self.edges[edge].WindCnt = 1;
            self.edges[edge].WindCnt2 = self.edges[e].WindCnt2;
            e = self.edges[e].NextInAEL; //ie get ready to calc WindCnt2
        } else if self.IsEvenOddFillType(edge) {
            //EvenOdd filling ...
            if self.edges[edge].WindDelta == 0 {
                //are we inside a subj polygon ...
                let mut Inside = true;
                let mut e2 = self.edges[e].PrevInAEL;
                while e2 != NIL {
                    if self.edges[e2].PolyTyp == self.edges[e].PolyTyp && self.edges[e2].WindDelta != 0 {
                        Inside = !Inside;
                    }
                    e2 = self.edges[e2].PrevInAEL;
                }
                self.edges[edge].WindCnt = if Inside { 0 } else { 1 };
            } else {
                self.edges[edge].WindCnt = self.edges[edge].WindDelta;
            }
            self.edges[edge].WindCnt2 = self.edges[e].WindCnt2;
            e = self.edges[e].NextInAEL; //ie get ready to calc WindCnt2
        } else {
            //nonZero, Positive or Negative filling ...
            let (ewc, ewd, wd) = (self.edges[e].WindCnt, self.edges[e].WindDelta, self.edges[edge].WindDelta);
            if ewc.wrapping_mul(ewd) < 0 {
                //prev edge is 'decreasing' WindCount (WC) toward zero
                //so we're outside the previous polygon ...
                if ewc.wrapping_abs() > 1 {
                    //outside prev poly but still inside another.
                    //when reversing direction of prev poly use the same WC
                    if ewd.wrapping_mul(wd) < 0 {
                        self.edges[edge].WindCnt = ewc;
                    } else {
                        //otherwise continue to 'decrease' WC ...
                        self.edges[edge].WindCnt = ewc.wrapping_add(wd);
                    }
                } else {
                    //now outside all polys of same polytype so set own WC ...
                    self.edges[edge].WindCnt = if wd == 0 { 1 } else { wd };
                }
            } else {
                //prev edge is 'increasing' WindCount (WC) away from zero
                //so we're inside the previous polygon ...
                if wd == 0 {
                    self.edges[edge].WindCnt = if ewc < 0 { ewc.wrapping_sub(1) } else { ewc.wrapping_add(1) };
                }
                //if wind direction is reversing prev then use same WC
                else if ewd.wrapping_mul(wd) < 0 {
                    self.edges[edge].WindCnt = ewc;
                }
                //otherwise add to WC ...
                else {
                    self.edges[edge].WindCnt = ewc.wrapping_add(wd);
                }
            }
            self.edges[edge].WindCnt2 = self.edges[e].WindCnt2;
            e = self.edges[e].NextInAEL; //ie get ready to calc WindCnt2
        }

        //update WindCnt2 ...
        if self.IsEvenOddAltFillType(edge) {
            //EvenOdd filling ...
            while e != edge {
                if self.edges[e].WindDelta != 0 {
                    self.edges[edge].WindCnt2 = if self.edges[edge].WindCnt2 == 0 { 1 } else { 0 };
                }
                e = self.edges[e].NextInAEL;
            }
        } else {
            //nonZero, Positive or Negative filling ...
            while e != edge {
                self.edges[edge].WindCnt2 = self.edges[edge].WindCnt2.wrapping_add(self.edges[e].WindDelta);
                e = self.edges[e].NextInAEL;
            }
        }
    }

    /// SEL pointers in PEdge are use to build transient lists of horizontal edges. However,
    /// since we don't need to worry about processing order, all additions are made to the
    /// front of the list ...
    fn AddEdgeToSEL(&mut self, edge: usize) {
        if self.m_SortedEdges == NIL {
            self.m_SortedEdges = edge;
            self.edges[edge].PrevInSEL = NIL;
            self.edges[edge].NextInSEL = NIL;
        } else {
            self.edges[edge].NextInSEL = self.m_SortedEdges;
            self.edges[edge].PrevInSEL = NIL;
            let se = self.m_SortedEdges;
            self.edges[se].PrevInSEL = edge;
            self.m_SortedEdges = edge;
        }
    }

    /// Pop edge from front of SEL (ie SEL is a FILO list)
    fn PopEdgeFromSEL(&mut self, e: &mut usize) -> bool {
        *e = self.m_SortedEdges;
        if *e == NIL {
            return false;
        }
        let oldE = *e;
        self.m_SortedEdges = self.edges[oldE].NextInSEL;
        if self.m_SortedEdges != NIL {
            let se = self.m_SortedEdges;
            self.edges[se].PrevInSEL = NIL;
        }
        self.edges[oldE].NextInSEL = NIL;
        self.edges[oldE].PrevInSEL = NIL;
        true
    }

    fn CopyAELToSEL(&mut self) {
        let mut e = self.m_ActiveEdges;
        self.m_SortedEdges = e;
        while e != NIL {
            self.edges[e].PrevInSEL = self.edges[e].PrevInAEL;
            self.edges[e].NextInSEL = self.edges[e].NextInAEL;
            e = self.edges[e].NextInAEL;
        }
    }

    fn SwapPositionsInSEL(&mut self, edge1: usize, edge2: usize) {
        if self.edges[edge1].NextInSEL == NIL && self.edges[edge1].PrevInSEL == NIL {
            return;
        }
        if self.edges[edge2].NextInSEL == NIL && self.edges[edge2].PrevInSEL == NIL {
            return;
        }

        if self.edges[edge1].NextInSEL == edge2 {
            let next = self.edges[edge2].NextInSEL;
            if next != NIL {
                self.edges[next].PrevInSEL = edge1;
            }
            let prev = self.edges[edge1].PrevInSEL;
            if prev != NIL {
                self.edges[prev].NextInSEL = edge2;
            }
            self.edges[edge2].PrevInSEL = prev;
            self.edges[edge2].NextInSEL = edge1;
            self.edges[edge1].PrevInSEL = edge2;
            self.edges[edge1].NextInSEL = next;
        } else if self.edges[edge2].NextInSEL == edge1 {
            let next = self.edges[edge1].NextInSEL;
            if next != NIL {
                self.edges[next].PrevInSEL = edge2;
            }
            let prev = self.edges[edge2].PrevInSEL;
            if prev != NIL {
                self.edges[prev].NextInSEL = edge1;
            }
            self.edges[edge1].PrevInSEL = prev;
            self.edges[edge1].NextInSEL = edge2;
            self.edges[edge2].PrevInSEL = edge1;
            self.edges[edge2].NextInSEL = next;
        } else {
            let next = self.edges[edge1].NextInSEL;
            let prev = self.edges[edge1].PrevInSEL;
            self.edges[edge1].NextInSEL = self.edges[edge2].NextInSEL;
            let n = self.edges[edge1].NextInSEL;
            if n != NIL {
                self.edges[n].PrevInSEL = edge1;
            }
            self.edges[edge1].PrevInSEL = self.edges[edge2].PrevInSEL;
            let p = self.edges[edge1].PrevInSEL;
            if p != NIL {
                self.edges[p].NextInSEL = edge1;
            }
            self.edges[edge2].NextInSEL = next;
            if next != NIL {
                self.edges[next].PrevInSEL = edge2;
            }
            self.edges[edge2].PrevInSEL = prev;
            if prev != NIL {
                self.edges[prev].NextInSEL = edge2;
            }
        }

        if self.edges[edge1].PrevInSEL == NIL {
            self.m_SortedEdges = edge1;
        } else if self.edges[edge2].PrevInSEL == NIL {
            self.m_SortedEdges = edge2;
        }
    }

    fn AddLocalMaxPoly(&mut self, e1: usize, e2: usize, pt: IntPoint) {
        self.AddOutPt(e1, pt);
        if self.edges[e2].WindDelta == 0 {
            self.AddOutPt(e2, pt);
        }
        if self.edges[e1].OutIdx == self.edges[e2].OutIdx {
            self.edges[e1].OutIdx = Unassigned;
            self.edges[e2].OutIdx = Unassigned;
        } else if self.edges[e1].OutIdx < self.edges[e2].OutIdx {
            self.AppendPolygon(e1, e2);
        } else {
            self.AppendPolygon(e2, e1);
        }
    }

    fn AddLocalMinPoly(&mut self, e1: usize, e2: usize, pt: IntPoint) -> usize {
        let result;
        let e;
        let prevE;
        if self.IsHorizontal(e2) || (self.edges[e1].Dx > self.edges[e2].Dx) {
            result = self.AddOutPt(e1, pt);
            self.edges[e2].OutIdx = self.edges[e1].OutIdx;
            self.edges[e1].Side = EdgeSide::esLeft;
            self.edges[e2].Side = EdgeSide::esRight;
            e = e1;
            if self.edges[e].PrevInAEL == e2 {
                prevE = self.edges[e2].PrevInAEL;
            } else {
                prevE = self.edges[e].PrevInAEL;
            }
        } else {
            result = self.AddOutPt(e2, pt);
            self.edges[e1].OutIdx = self.edges[e2].OutIdx;
            self.edges[e1].Side = EdgeSide::esRight;
            self.edges[e2].Side = EdgeSide::esLeft;
            e = e2;
            if self.edges[e].PrevInAEL == e1 {
                prevE = self.edges[e1].PrevInAEL;
            } else {
                prevE = self.edges[e].PrevInAEL;
            }
        }

        if prevE != NIL && self.edges[prevE].OutIdx >= 0 && self.edges[prevE].Top.Y < pt.Y && self.edges[e].Top.Y < pt.Y {
            let xPrev = Self::TopX(&self.edges[prevE], pt.Y);
            let xE = Self::TopX(&self.edges[e], pt.Y);
            if (xPrev == xE)
                && (self.edges[e].WindDelta != 0)
                && (self.edges[prevE].WindDelta != 0)
                && Self::SlopesEqual4(IntPoint::new(xPrev, pt.Y), self.edges[prevE].Top, IntPoint::new(xE, pt.Y), self.edges[e].Top, self.m_UseFullRange)
            {
                let outPt = self.AddOutPt(prevE, pt);
                let t = self.edges[e].Top;
                self.AddJoin(result, outPt, t);
            }
        }
        result
    }

    fn new_outpt(&mut self, op: OutPt) -> usize {
        self.outpts.push(op);
        self.outpts.len() - 1
    }

    fn AddOutPt(&mut self, e: usize, pt: IntPoint) -> usize {
        if self.edges[e].OutIdx < 0 {
            let outRec = self.CreateOutRec();
            self.outrecs[outRec].IsOpen = self.edges[e].WindDelta == 0;
            let idx = self.outrecs[outRec].Idx;
            let newOp = self.new_outpt(OutPt { Idx: idx, Pt: pt, Next: NIL, Prev: NIL });
            self.outrecs[outRec].Pts = newOp;
            self.outpts[newOp].Next = newOp;
            self.outpts[newOp].Prev = newOp;
            if !self.outrecs[outRec].IsOpen {
                self.SetHoleState(e, outRec);
            }
            self.edges[e].OutIdx = idx; //nb: do this after SetZ !
            newOp
        } else {
            let outRec = self.m_PolyOuts[self.edges[e].OutIdx as usize];
            //OutRec.Pts is the 'Left-most' point & OutRec.Pts.Prev is the 'Right-most'
            let op = self.outrecs[outRec].Pts;
            let ToFront = self.edges[e].Side == EdgeSide::esLeft;
            if ToFront && pt == self.outpts[op].Pt {
                return op;
            } else if !ToFront && pt == self.outpts[self.outpts[op].Prev].Pt {
                return self.outpts[op].Prev;
            }

            let opPrev = self.outpts[op].Prev;
            let idx = self.outrecs[outRec].Idx;
            let newOp = self.new_outpt(OutPt { Idx: idx, Pt: pt, Next: op, Prev: opPrev });
            self.outpts[opPrev].Next = newOp;
            self.outpts[op].Prev = newOp;
            if ToFront {
                self.outrecs[outRec].Pts = newOp;
            }
            newOp
        }
    }

    fn GetLastOutPt(&self, e: usize) -> usize {
        let outRec = self.m_PolyOuts[self.edges[e].OutIdx as usize];
        if self.edges[e].Side == EdgeSide::esLeft {
            self.outrecs[outRec].Pts
        } else {
            self.outpts[self.outrecs[outRec].Pts].Prev
        }
    }

    fn HorzSegmentsOverlap(mut seg1a: cInt, mut seg1b: cInt, mut seg2a: cInt, mut seg2b: cInt) -> bool {
        if seg1a > seg1b {
            std::mem::swap(&mut seg1a, &mut seg1b);
        }
        if seg2a > seg2b {
            std::mem::swap(&mut seg2a, &mut seg2b);
        }
        (seg1a < seg2b) && (seg2a < seg1b)
    }

    fn SetHoleState(&mut self, e: usize, outRec: usize) {
        let mut e2 = self.edges[e].PrevInAEL;
        let mut eTmp = NIL;
        while e2 != NIL {
            if self.edges[e2].OutIdx >= 0 && self.edges[e2].WindDelta != 0 {
                if eTmp == NIL {
                    eTmp = e2;
                } else if self.edges[eTmp].OutIdx == self.edges[e2].OutIdx {
                    eTmp = NIL; //paired
                }
            }
            e2 = self.edges[e2].PrevInAEL;
        }

        if eTmp == NIL {
            self.outrecs[outRec].FirstLeft = NIL;
            self.outrecs[outRec].IsHole = false;
        } else {
            let fl = self.m_PolyOuts[self.edges[eTmp].OutIdx as usize];
            self.outrecs[outRec].FirstLeft = fl;
            self.outrecs[outRec].IsHole = !self.outrecs[fl].IsHole;
        }
    }

    fn GetDx(pt1: IntPoint, pt2: IntPoint) -> f64 {
        if pt1.Y == pt2.Y {
            horizontal
        } else {
            (pt2.X.wrapping_sub(pt1.X)) as f64 / (pt2.Y.wrapping_sub(pt1.Y)) as f64
        }
    }

    fn FirstIsBottomPt(&self, btmPt1: usize, btmPt2: usize) -> bool {
        let o = &self.outpts;
        let mut p = o[btmPt1].Prev;
        while (o[p].Pt == o[btmPt1].Pt) && (p != btmPt1) {
            p = o[p].Prev;
        }
        let dx1p = Self::GetDx(o[btmPt1].Pt, o[p].Pt).abs();
        p = o[btmPt1].Next;
        while (o[p].Pt == o[btmPt1].Pt) && (p != btmPt1) {
            p = o[p].Next;
        }
        let dx1n = Self::GetDx(o[btmPt1].Pt, o[p].Pt).abs();

        p = o[btmPt2].Prev;
        while (o[p].Pt == o[btmPt2].Pt) && (p != btmPt2) {
            p = o[p].Prev;
        }
        let dx2p = Self::GetDx(o[btmPt2].Pt, o[p].Pt).abs();
        p = o[btmPt2].Next;
        while (o[p].Pt == o[btmPt2].Pt) && (p != btmPt2) {
            p = o[p].Next;
        }
        let dx2n = Self::GetDx(o[btmPt2].Pt, o[p].Pt).abs();

        if crate::dotnet::math_max(dx1p, dx1n) == crate::dotnet::math_max(dx2p, dx2n)
            && crate::dotnet::math_min(dx1p, dx1n) == crate::dotnet::math_min(dx2p, dx2n)
        {
            self.AreaOp(btmPt1) > 0.0 //if otherwise identical use orientation
        } else {
            (dx1p >= dx2p && dx1p >= dx2n) || (dx1n >= dx2p && dx1n >= dx2n)
        }
    }

    fn GetBottomPt(&self, mut pp: usize) -> usize {
        let o = &self.outpts;
        let mut dups = NIL;
        let mut p = o[pp].Next;
        while p != pp {
            if o[p].Pt.Y > o[pp].Pt.Y {
                pp = p;
                dups = NIL;
            } else if o[p].Pt.Y == o[pp].Pt.Y && o[p].Pt.X <= o[pp].Pt.X {
                if o[p].Pt.X < o[pp].Pt.X {
                    dups = NIL;
                    pp = p;
                } else if o[p].Next != pp && o[p].Prev != pp {
                    dups = p;
                }
            }
            p = o[p].Next;
        }
        if dups != NIL {
            //there appears to be at least 2 vertices at bottomPt so ...
            while dups != p {
                if !self.FirstIsBottomPt(p, dups) {
                    pp = dups;
                }
                dups = o[dups].Next;
                while o[dups].Pt != o[pp].Pt {
                    dups = o[dups].Next;
                }
            }
        }
        pp
    }

    fn GetLowermostRec(&mut self, outRec1: usize, outRec2: usize) -> usize {
        //work out which polygon fragment has the correct hole state ...
        if self.outrecs[outRec1].BottomPt == NIL {
            let b = self.GetBottomPt(self.outrecs[outRec1].Pts);
            self.outrecs[outRec1].BottomPt = b;
        }
        if self.outrecs[outRec2].BottomPt == NIL {
            let b = self.GetBottomPt(self.outrecs[outRec2].Pts);
            self.outrecs[outRec2].BottomPt = b;
        }
        let bPt1 = self.outrecs[outRec1].BottomPt;
        let bPt2 = self.outrecs[outRec2].BottomPt;
        let (p1, p2) = (self.outpts[bPt1].Pt, self.outpts[bPt2].Pt);
        if p1.Y > p2.Y {
            outRec1
        } else if p1.Y < p2.Y {
            outRec2
        } else if p1.X < p2.X {
            outRec1
        } else if p1.X > p2.X {
            outRec2
        } else if self.outpts[bPt1].Next == bPt1 {
            outRec2
        } else if self.outpts[bPt2].Next == bPt2 {
            outRec1
        } else if self.FirstIsBottomPt(bPt1, bPt2) {
            outRec1
        } else {
            outRec2
        }
    }

    fn OutRec1RightOfOutRec2(&self, mut outRec1: usize, outRec2: usize) -> bool {
        loop {
            outRec1 = self.outrecs[outRec1].FirstLeft;
            if outRec1 == outRec2 {
                return true;
            }
            if outRec1 == NIL {
                break;
            }
        }
        false
    }

    fn GetOutRec(&self, idx: usize) -> usize {
        let mut outrec = self.m_PolyOuts[idx];
        while outrec != self.m_PolyOuts[self.outrecs[outrec].Idx as usize] {
            outrec = self.m_PolyOuts[self.outrecs[outrec].Idx as usize];
        }
        outrec
    }

    fn AppendPolygon(&mut self, e1: usize, e2: usize) {
        let outRec1 = self.m_PolyOuts[self.edges[e1].OutIdx as usize];
        let outRec2 = self.m_PolyOuts[self.edges[e2].OutIdx as usize];

        let holeStateRec = if self.OutRec1RightOfOutRec2(outRec1, outRec2) {
            outRec2
        } else if self.OutRec1RightOfOutRec2(outRec2, outRec1) {
            outRec1
        } else {
            self.GetLowermostRec(outRec1, outRec2)
        };

        //get the start and ends of both output polygons and
        //join E2 poly onto E1 poly and delete pointers to E2 ...
        let p1_lft = self.outrecs[outRec1].Pts;
        let p1_rt = self.outpts[p1_lft].Prev;
        let p2_lft = self.outrecs[outRec2].Pts;
        let p2_rt = self.outpts[p2_lft].Prev;

        //join e2 poly onto e1 poly and delete pointers to e2 ...
        if self.edges[e1].Side == EdgeSide::esLeft {
            if self.edges[e2].Side == EdgeSide::esLeft {
                //z y x a b c
                self.ReversePolyPtLinks(p2_lft);
                self.outpts[p2_lft].Next = p1_lft;
                self.outpts[p1_lft].Prev = p2_lft;
                self.outpts[p1_rt].Next = p2_rt;
                self.outpts[p2_rt].Prev = p1_rt;
                self.outrecs[outRec1].Pts = p2_rt;
            } else {
                //x y z a b c
                self.outpts[p2_rt].Next = p1_lft;
                self.outpts[p1_lft].Prev = p2_rt;
                self.outpts[p2_lft].Prev = p1_rt;
                self.outpts[p1_rt].Next = p2_lft;
                self.outrecs[outRec1].Pts = p2_lft;
            }
        } else if self.edges[e2].Side == EdgeSide::esRight {
            //a b c z y x
            self.ReversePolyPtLinks(p2_lft);
            self.outpts[p1_rt].Next = p2_rt;
            self.outpts[p2_rt].Prev = p1_rt;
            self.outpts[p2_lft].Next = p1_lft;
            self.outpts[p1_lft].Prev = p2_lft;
        } else {
            //a b c x y z
            self.outpts[p1_rt].Next = p2_lft;
            self.outpts[p2_lft].Prev = p1_rt;
            self.outpts[p1_lft].Prev = p2_rt;
            self.outpts[p2_rt].Next = p1_lft;
        }

        self.outrecs[outRec1].BottomPt = NIL;
        if holeStateRec == outRec2 {
            if self.outrecs[outRec2].FirstLeft != outRec1 {
                self.outrecs[outRec1].FirstLeft = self.outrecs[outRec2].FirstLeft;
            }
            self.outrecs[outRec1].IsHole = self.outrecs[outRec2].IsHole;
        }
        self.outrecs[outRec2].Pts = NIL;
        self.outrecs[outRec2].BottomPt = NIL;

        self.outrecs[outRec2].FirstLeft = outRec1;

        let OKIdx = self.edges[e1].OutIdx;
        let ObsoleteIdx = self.edges[e2].OutIdx;

        self.edges[e1].OutIdx = Unassigned; //nb: safe because we only get here via AddLocalMaxPoly
        self.edges[e2].OutIdx = Unassigned;

        let mut e = self.m_ActiveEdges;
        while e != NIL {
            if self.edges[e].OutIdx == ObsoleteIdx {
                self.edges[e].OutIdx = OKIdx;
                self.edges[e].Side = self.edges[e1].Side;
                break;
            }
            e = self.edges[e].NextInAEL;
        }
        self.outrecs[outRec2].Idx = self.outrecs[outRec1].Idx;
    }

    fn ReversePolyPtLinks(&mut self, pp: usize) {
        if pp == NIL {
            return;
        }
        let mut pp1 = pp;
        loop {
            let pp2 = self.outpts[pp1].Next;
            self.outpts[pp1].Next = self.outpts[pp1].Prev;
            self.outpts[pp1].Prev = pp2;
            pp1 = pp2;
            if pp1 == pp {
                break;
            }
        }
    }

    fn SwapSides(&mut self, edge1: usize, edge2: usize) {
        let side = self.edges[edge1].Side;
        self.edges[edge1].Side = self.edges[edge2].Side;
        self.edges[edge2].Side = side;
    }

    fn SwapPolyIndexes(&mut self, edge1: usize, edge2: usize) {
        let outIdx = self.edges[edge1].OutIdx;
        self.edges[edge1].OutIdx = self.edges[edge2].OutIdx;
        self.edges[edge2].OutIdx = outIdx;
    }

    /// e1 will be to the left of e2 BELOW the intersection. Therefore e1 is before e2 in AEL
    /// except when e1 is being inserted at the intersection point ...
    fn IntersectEdges(&mut self, e1: usize, e2: usize, pt: IntPoint) {
        let e1Contributing = self.edges[e1].OutIdx >= 0;
        let e2Contributing = self.edges[e2].OutIdx >= 0;

        //if either edge is on an OPEN path ...
        if self.edges[e1].WindDelta == 0 || self.edges[e2].WindDelta == 0 {
            let (a, b) = (self.edges[e1], self.edges[e2]);
            //ignore subject-subject open path intersections UNLESS they
            //are both open paths, AND they are both 'contributing maximas' ...
            if a.WindDelta == 0 && b.WindDelta == 0 {
                return;
            }
            //if intersecting a subj line with a subj poly ...
            else if a.PolyTyp == b.PolyTyp && a.WindDelta != b.WindDelta && self.m_ClipType == ClipType::ctUnion {
                if a.WindDelta == 0 {
                    if e2Contributing {
                        self.AddOutPt(e1, pt);
                        if e1Contributing {
                            self.edges[e1].OutIdx = Unassigned;
                        }
                    }
                } else if e1Contributing {
                    self.AddOutPt(e2, pt);
                    if e2Contributing {
                        self.edges[e2].OutIdx = Unassigned;
                    }
                }
            } else if a.PolyTyp != b.PolyTyp {
                if (a.WindDelta == 0) && b.WindCnt.wrapping_abs() == 1 && (self.m_ClipType != ClipType::ctUnion || b.WindCnt2 == 0) {
                    self.AddOutPt(e1, pt);
                    if e1Contributing {
                        self.edges[e1].OutIdx = Unassigned;
                    }
                } else if (b.WindDelta == 0) && (a.WindCnt.wrapping_abs() == 1) && (self.m_ClipType != ClipType::ctUnion || a.WindCnt2 == 0) {
                    self.AddOutPt(e2, pt);
                    if e2Contributing {
                        self.edges[e2].OutIdx = Unassigned;
                    }
                }
            }
            return;
        }

        //update winding counts...
        //assumes that e1 will be to the Right of e2 ABOVE the intersection
        if self.edges[e1].PolyTyp == self.edges[e2].PolyTyp {
            if self.IsEvenOddFillType(e1) {
                let oldE1WindCnt = self.edges[e1].WindCnt;
                self.edges[e1].WindCnt = self.edges[e2].WindCnt;
                self.edges[e2].WindCnt = oldE1WindCnt;
            } else {
                let (e2wd, e1wd) = (self.edges[e2].WindDelta, self.edges[e1].WindDelta);
                if self.edges[e1].WindCnt.wrapping_add(e2wd) == 0 {
                    self.edges[e1].WindCnt = self.edges[e1].WindCnt.wrapping_neg();
                } else {
                    self.edges[e1].WindCnt = self.edges[e1].WindCnt.wrapping_add(e2wd);
                }
                if self.edges[e2].WindCnt.wrapping_sub(e1wd) == 0 {
                    self.edges[e2].WindCnt = self.edges[e2].WindCnt.wrapping_neg();
                } else {
                    self.edges[e2].WindCnt = self.edges[e2].WindCnt.wrapping_sub(e1wd);
                }
            }
        } else {
            if !self.IsEvenOddFillType(e2) {
                self.edges[e1].WindCnt2 = self.edges[e1].WindCnt2.wrapping_add(self.edges[e2].WindDelta);
            } else {
                self.edges[e1].WindCnt2 = if self.edges[e1].WindCnt2 == 0 { 1 } else { 0 };
            }
            if !self.IsEvenOddFillType(e1) {
                self.edges[e2].WindCnt2 = self.edges[e2].WindCnt2.wrapping_sub(self.edges[e1].WindDelta);
            } else {
                self.edges[e2].WindCnt2 = if self.edges[e2].WindCnt2 == 0 { 1 } else { 0 };
            }
        }

        let (e1FillType, e1FillType2) = if self.edges[e1].PolyTyp == PolyType::ptSubject {
            (self.m_SubjFillType, self.m_ClipFillType)
        } else {
            (self.m_ClipFillType, self.m_SubjFillType)
        };
        let (e2FillType, e2FillType2) = if self.edges[e2].PolyTyp == PolyType::ptSubject {
            (self.m_SubjFillType, self.m_ClipFillType)
        } else {
            (self.m_ClipFillType, self.m_SubjFillType)
        };

        let wc = |ft: PolyFillType, c: i32| -> i32 {
            match ft {
                PolyFillType::pftPositive => c,
                PolyFillType::pftNegative => c.wrapping_neg(),
                _ => c.wrapping_abs(),
            }
        };
        let e1Wc = wc(e1FillType, self.edges[e1].WindCnt);
        let e2Wc = wc(e2FillType, self.edges[e2].WindCnt);

        if e1Contributing && e2Contributing {
            if (e1Wc != 0 && e1Wc != 1) || (e2Wc != 0 && e2Wc != 1) || (self.edges[e1].PolyTyp != self.edges[e2].PolyTyp && self.m_ClipType != ClipType::ctXor) {
                self.AddLocalMaxPoly(e1, e2, pt);
            } else {
                self.AddOutPt(e1, pt);
                self.AddOutPt(e2, pt);
                self.SwapSides(e1, e2);
                self.SwapPolyIndexes(e1, e2);
            }
        } else if e1Contributing {
            if e2Wc == 0 || e2Wc == 1 {
                self.AddOutPt(e1, pt);
                self.SwapSides(e1, e2);
                self.SwapPolyIndexes(e1, e2);
            }
        } else if e2Contributing {
            if e1Wc == 0 || e1Wc == 1 {
                self.AddOutPt(e2, pt);
                self.SwapSides(e1, e2);
                self.SwapPolyIndexes(e1, e2);
            }
        } else if (e1Wc == 0 || e1Wc == 1) && (e2Wc == 0 || e2Wc == 1) {
            //neither edge is currently contributing ...
            let e1Wc2 = wc(e1FillType2, self.edges[e1].WindCnt2) as cInt;
            let e2Wc2 = wc(e2FillType2, self.edges[e2].WindCnt2) as cInt;

            if self.edges[e1].PolyTyp != self.edges[e2].PolyTyp {
                self.AddLocalMinPoly(e1, e2, pt);
            } else if e1Wc == 1 && e2Wc == 1 {
                match self.m_ClipType {
                    ClipType::ctIntersection => {
                        if e1Wc2 > 0 && e2Wc2 > 0 {
                            self.AddLocalMinPoly(e1, e2, pt);
                        }
                    }
                    ClipType::ctUnion => {
                        if e1Wc2 <= 0 && e2Wc2 <= 0 {
                            self.AddLocalMinPoly(e1, e2, pt);
                        }
                    }
                    ClipType::ctDifference => {
                        let t = self.edges[e1].PolyTyp;
                        if ((t == PolyType::ptClip) && (e1Wc2 > 0) && (e2Wc2 > 0)) || ((t == PolyType::ptSubject) && (e1Wc2 <= 0) && (e2Wc2 <= 0)) {
                            self.AddLocalMinPoly(e1, e2, pt);
                        }
                    }
                    ClipType::ctXor => {
                        self.AddLocalMinPoly(e1, e2, pt);
                    }
                }
            } else {
                self.SwapSides(e1, e2);
            }
        }
    }

    #[allow(dead_code)]
    fn DeleteFromSEL(&mut self, e: usize) {
        let SelPrev = self.edges[e].PrevInSEL;
        let SelNext = self.edges[e].NextInSEL;
        if SelPrev == NIL && SelNext == NIL && (e != self.m_SortedEdges) {
            return; //already deleted
        }
        if SelPrev != NIL {
            self.edges[SelPrev].NextInSEL = SelNext;
        } else {
            self.m_SortedEdges = SelNext;
        }
        if SelNext != NIL {
            self.edges[SelNext].PrevInSEL = SelPrev;
        }
        self.edges[e].NextInSEL = NIL;
        self.edges[e].PrevInSEL = NIL;
    }

    fn ProcessHorizontals(&mut self) -> Result<(), Exception> {
        let mut horzEdge = NIL; //m_SortedEdges;
        while self.PopEdgeFromSEL(&mut horzEdge) {
            self.ProcessHorizontal(horzEdge)?;
        }
        Ok(())
    }

    fn GetHorzDirection(&self, HorzEdge: usize, Dir: &mut Direction, Left: &mut cInt, Right: &mut cInt) {
        let h = &self.edges[HorzEdge];
        if h.Bot.X < h.Top.X {
            *Left = h.Bot.X;
            *Right = h.Top.X;
            *Dir = Direction::dLeftToRight;
        } else {
            *Left = h.Top.X;
            *Right = h.Bot.X;
            *Dir = Direction::dRightToLeft;
        }
    }

    fn ProcessHorizontal(&mut self, mut horzEdge: usize) -> Result<(), Exception> {
        let mut dir = Direction::dLeftToRight;
        let (mut horzLeft, mut horzRight): (cInt, cInt) = (0, 0);
        let IsOpen = self.edges[horzEdge].WindDelta == 0;

        self.GetHorzDirection(horzEdge, &mut dir, &mut horzLeft, &mut horzRight);

        let mut eLastHorz = horzEdge;
        let mut eMaxPair = NIL;
        while self.edges[eLastHorz].NextInLML != NIL && self.IsHorizontal(self.edges[eLastHorz].NextInLML) {
            eLastHorz = self.edges[eLastHorz].NextInLML;
        }
        if self.edges[eLastHorz].NextInLML == NIL {
            eMaxPair = self.GetMaximaPair(eLastHorz);
        }

        let mut currMax = self.m_Maxima;
        if currMax != NIL {
            //get the first maxima in range (X) ...
            if dir == Direction::dLeftToRight {
                while currMax != NIL && self.maxs[currMax].X <= self.edges[horzEdge].Bot.X {
                    currMax = self.maxs[currMax].Next;
                }
                if currMax != NIL && self.maxs[currMax].X >= self.edges[eLastHorz].Top.X {
                    currMax = NIL;
                }
            } else {
                while self.maxs[currMax].Next != NIL && self.maxs[self.maxs[currMax].Next].X < self.edges[horzEdge].Bot.X {
                    currMax = self.maxs[currMax].Next;
                }
                if self.maxs[currMax].X <= self.edges[eLastHorz].Top.X {
                    currMax = NIL;
                }
            }
        }

        let mut op1 = NIL;
        loop {
            //loop through consec. horizontal edges
            let IsLastHorz = horzEdge == eLastHorz;
            let mut e = self.GetNextInAEL(horzEdge, dir);
            while e != NIL {
                //this code block inserts extra coords into horizontal edges (in output
                //polygons) whereever maxima touch these horizontal edges. This helps
                //'simplifying' polygons (ie if the Simplify property is set).
                if currMax != NIL {
                    if dir == Direction::dLeftToRight {
                        while currMax != NIL && self.maxs[currMax].X < self.edges[e].Curr.X {
                            if self.edges[horzEdge].OutIdx >= 0 && !IsOpen {
                                let p = IntPoint::new(self.maxs[currMax].X, self.edges[horzEdge].Bot.Y);
                                self.AddOutPt(horzEdge, p);
                            }
                            currMax = self.maxs[currMax].Next;
                        }
                    } else {
                        while currMax != NIL && self.maxs[currMax].X > self.edges[e].Curr.X {
                            if self.edges[horzEdge].OutIdx >= 0 && !IsOpen {
                                let p = IntPoint::new(self.maxs[currMax].X, self.edges[horzEdge].Bot.Y);
                                self.AddOutPt(horzEdge, p);
                            }
                            currMax = self.maxs[currMax].Prev;
                        }
                    }
                }

                if (dir == Direction::dLeftToRight && self.edges[e].Curr.X > horzRight) || (dir == Direction::dRightToLeft && self.edges[e].Curr.X < horzLeft) {
                    break;
                }

                //Also break if we've got to the end of an intermediate horizontal edge ...
                //nb: Smaller Dx's are to the right of larger Dx's ABOVE the horizontal.
                let hnl = self.edges[horzEdge].NextInLML;
                if self.edges[e].Curr.X == self.edges[horzEdge].Top.X && hnl != NIL && self.edges[e].Dx < self.edges[hnl].Dx {
                    break;
                }

                if self.edges[horzEdge].OutIdx >= 0 && !IsOpen {
                    //note: may be done multiple times
                    let c = self.edges[e].Curr;
                    op1 = self.AddOutPt(horzEdge, c);
                    let mut eNextHorz = self.m_SortedEdges;
                    while eNextHorz != NIL {
                        if self.edges[eNextHorz].OutIdx >= 0
                            && Self::HorzSegmentsOverlap(self.edges[horzEdge].Bot.X, self.edges[horzEdge].Top.X, self.edges[eNextHorz].Bot.X, self.edges[eNextHorz].Top.X)
                        {
                            let op2 = self.GetLastOutPt(eNextHorz);
                            let t = self.edges[eNextHorz].Top;
                            self.AddJoin(op2, op1, t);
                        }
                        eNextHorz = self.edges[eNextHorz].NextInSEL;
                    }
                    let b = self.edges[horzEdge].Bot;
                    self.AddGhostJoin(op1, b);
                }

                //OK, so far we're still in range of the horizontal Edge  but make sure
                //we're at the last of consec. horizontals when matching with eMaxPair
                if e == eMaxPair && IsLastHorz {
                    if self.edges[horzEdge].OutIdx >= 0 {
                        let t = self.edges[horzEdge].Top;
                        self.AddLocalMaxPoly(horzEdge, eMaxPair, t);
                    }
                    self.DeleteFromAEL(horzEdge);
                    self.DeleteFromAEL(eMaxPair);
                    return Ok(());
                }

                let Pt = IntPoint::new(self.edges[e].Curr.X, self.edges[horzEdge].Curr.Y);
                if dir == Direction::dLeftToRight {
                    self.IntersectEdges(horzEdge, e, Pt);
                } else {
                    self.IntersectEdges(e, horzEdge, Pt);
                }
                let eNext = self.GetNextInAEL(e, dir);
                self.SwapPositionsInAEL(horzEdge, e);
                e = eNext;
            } //end while(e != null)

            //Break out of loop if HorzEdge.NextInLML is not also horizontal ...
            let hnl = self.edges[horzEdge].NextInLML;
            if hnl == NIL || !self.IsHorizontal(hnl) {
                break;
            }

            self.UpdateEdgeIntoAEL(&mut horzEdge)?;
            if self.edges[horzEdge].OutIdx >= 0 {
                let b = self.edges[horzEdge].Bot;
                self.AddOutPt(horzEdge, b);
            }
            self.GetHorzDirection(horzEdge, &mut dir, &mut horzLeft, &mut horzRight);
        } //end for (;;)

        if self.edges[horzEdge].OutIdx >= 0 && op1 == NIL {
            op1 = self.GetLastOutPt(horzEdge);
            let mut eNextHorz = self.m_SortedEdges;
            while eNextHorz != NIL {
                if self.edges[eNextHorz].OutIdx >= 0
                    && Self::HorzSegmentsOverlap(self.edges[horzEdge].Bot.X, self.edges[horzEdge].Top.X, self.edges[eNextHorz].Bot.X, self.edges[eNextHorz].Top.X)
                {
                    let op2 = self.GetLastOutPt(eNextHorz);
                    let t = self.edges[eNextHorz].Top;
                    self.AddJoin(op2, op1, t);
                }
                eNextHorz = self.edges[eNextHorz].NextInSEL;
            }
            let t = self.edges[horzEdge].Top;
            self.AddGhostJoin(op1, t);
        }

        if self.edges[horzEdge].NextInLML != NIL {
            if self.edges[horzEdge].OutIdx >= 0 {
                let t = self.edges[horzEdge].Top;
                op1 = self.AddOutPt(horzEdge, t);

                self.UpdateEdgeIntoAEL(&mut horzEdge)?;
                if self.edges[horzEdge].WindDelta == 0 {
                    return Ok(());
                }
                //nb: HorzEdge is no longer horizontal here
                let ePrev = self.edges[horzEdge].PrevInAEL;
                let eNext = self.edges[horzEdge].NextInAEL;
                let h = self.edges[horzEdge];
                if ePrev != NIL
                    && self.edges[ePrev].Curr.X == h.Bot.X
                    && self.edges[ePrev].Curr.Y == h.Bot.Y
                    && self.edges[ePrev].WindDelta != 0
                    && (self.edges[ePrev].OutIdx >= 0 && self.edges[ePrev].Curr.Y > self.edges[ePrev].Top.Y && self.SlopesEqualE(horzEdge, ePrev, self.m_UseFullRange))
                {
                    let op2 = self.AddOutPt(ePrev, h.Bot);
                    self.AddJoin(op1, op2, h.Top);
                } else if eNext != NIL
                    && self.edges[eNext].Curr.X == h.Bot.X
                    && self.edges[eNext].Curr.Y == h.Bot.Y
                    && self.edges[eNext].WindDelta != 0
                    && self.edges[eNext].OutIdx >= 0
                    && self.edges[eNext].Curr.Y > self.edges[eNext].Top.Y
                    && self.SlopesEqualE(horzEdge, eNext, self.m_UseFullRange)
                {
                    let op2 = self.AddOutPt(eNext, h.Bot);
                    self.AddJoin(op1, op2, h.Top);
                }
            } else {
                self.UpdateEdgeIntoAEL(&mut horzEdge)?;
            }
        } else {
            if self.edges[horzEdge].OutIdx >= 0 {
                let t = self.edges[horzEdge].Top;
                self.AddOutPt(horzEdge, t);
            }
            self.DeleteFromAEL(horzEdge);
        }
        Ok(())
    }

    fn GetNextInAEL(&self, e: usize, Direction: Direction) -> usize {
        if Direction == Direction::dLeftToRight {
            self.edges[e].NextInAEL
        } else {
            self.edges[e].PrevInAEL
        }
    }

    #[allow(dead_code)]
    fn IsMinima(&self, e: usize) -> bool {
        e != NIL && (self.edges[self.edges[e].Prev].NextInLML != e) && (self.edges[self.edges[e].Next].NextInLML != e)
    }

    /// `IsMaxima(TEdge e, double Y)`: the long Top.Y compared as a double.
    fn IsMaxima(&self, e: usize, Y: f64) -> bool {
        e != NIL && self.edges[e].Top.Y as f64 == Y && self.edges[e].NextInLML == NIL
    }

    fn IsIntermediate(&self, e: usize, Y: f64) -> bool {
        self.edges[e].Top.Y as f64 == Y && self.edges[e].NextInLML != NIL
    }

    fn GetMaximaPair(&self, e: usize) -> usize {
        let (n, p) = (self.edges[e].Next, self.edges[e].Prev);
        if (self.edges[n].Top == self.edges[e].Top) && self.edges[n].NextInLML == NIL {
            n
        } else if (self.edges[p].Top == self.edges[e].Top) && self.edges[p].NextInLML == NIL {
            p
        } else {
            NIL
        }
    }

    /// as above but returns null if MaxPair isn't in AEL (unless it's horizontal)
    fn GetMaximaPairEx(&self, e: usize) -> usize {
        let result = self.GetMaximaPair(e);
        if result == NIL
            || self.edges[result].OutIdx == Skip
            || ((self.edges[result].NextInAEL == self.edges[result].PrevInAEL) && !self.IsHorizontal(result))
        {
            return NIL;
        }
        result
    }

    fn ProcessIntersections(&mut self, topY: cInt) -> Result<bool, Exception> {
        if self.m_ActiveEdges == NIL {
            return Ok(true);
        }
        let r = (|| -> Result<Option<bool>, Exception> {
            self.BuildIntersectList(topY);
            if self.m_IntersectList.is_empty() {
                return Ok(Some(true));
            }
            if self.m_IntersectList.len() == 1 || self.FixupIntersectionOrder()? {
                self.ProcessIntersectList();
            } else {
                return Ok(Some(false));
            }
            Ok(None)
        })();
        match r {
            Ok(Some(v)) => Ok(v),
            Ok(None) => {
                self.m_SortedEdges = NIL;
                Ok(true)
            }
            Err(_) => {
                self.m_SortedEdges = NIL;
                self.m_IntersectList.clear();
                Err(clipper_exception("ProcessIntersections error"))
            }
        }
    }

    fn BuildIntersectList(&mut self, topY: cInt) {
        if self.m_ActiveEdges == NIL {
            return;
        }

        //prepare for sorting ...
        let mut e = self.m_ActiveEdges;
        self.m_SortedEdges = e;
        while e != NIL {
            self.edges[e].PrevInSEL = self.edges[e].PrevInAEL;
            self.edges[e].NextInSEL = self.edges[e].NextInAEL;
            self.edges[e].Curr.X = Self::TopX(&self.edges[e], topY);
            e = self.edges[e].NextInAEL;
        }

        //bubblesort ...
        let mut isModified = true;
        while isModified && self.m_SortedEdges != NIL {
            isModified = false;
            e = self.m_SortedEdges;
            while self.edges[e].NextInSEL != NIL {
                let eNext = self.edges[e].NextInSEL;
                if self.edges[e].Curr.X > self.edges[eNext].Curr.X {
                    let mut pt = self.IntersectPoint(e, eNext);
                    if pt.Y < topY {
                        pt = IntPoint::new(Self::TopX(&self.edges[e], topY), topY);
                    }
                    if self.m_IntersectList.len() == self.m_IntersectCapacity {
                        self.m_IntersectCapacity = if self.m_IntersectCapacity == 0 { 4 } else { self.m_IntersectCapacity * 2 };
                    }
                    self.m_IntersectList.push(IntersectNode { Edge1: e, Edge2: eNext, Pt: pt });

                    self.SwapPositionsInSEL(e, eNext);
                    isModified = true;
                } else {
                    e = eNext;
                }
            }
            let p = self.edges[e].PrevInSEL;
            if p != NIL {
                self.edges[p].NextInSEL = NIL;
            } else {
                break;
            }
        }
        self.m_SortedEdges = NIL;
    }

    fn EdgesAdjacent(&self, inode: &IntersectNode) -> bool {
        (self.edges[inode.Edge1].NextInSEL == inode.Edge2) || (self.edges[inode.Edge1].PrevInSEL == inode.Edge2)
    }

    /// pre-condition: intersections are sorted bottom-most first. Now it's crucial that
    /// intersections are made only between adjacent edges, so to ensure this the order of
    /// intersections may need adjusting ...
    fn FixupIntersectionOrder(&mut self) -> Result<bool, Exception> {
        // List<T>.Sort(IComparer): .NET's introspective sort, not a stable one
        let n = self.m_IntersectCapacity;
        crate::dotnet::collections::introspective_sort(&mut self.m_IntersectList, n, |a: &IntersectNode, b: &IntersectNode| Ok(MyIntersectNodeSort(a, b)))?;

        self.CopyAELToSEL();
        let cnt = self.m_IntersectList.len();
        for i in 0..cnt {
            if !self.EdgesAdjacent(&self.m_IntersectList[i]) {
                let mut j = i + 1;
                while j < cnt && !self.EdgesAdjacent(&self.m_IntersectList[j]) {
                    j += 1;
                }
                if j == cnt {
                    return Ok(false);
                }

                self.m_IntersectList.swap(i, j);
            }
            let (a, b) = (self.m_IntersectList[i].Edge1, self.m_IntersectList[i].Edge2);
            self.SwapPositionsInSEL(a, b);
        }
        Ok(true)
    }

    fn ProcessIntersectList(&mut self) {
        for i in 0..self.m_IntersectList.len() {
            let iNode = self.m_IntersectList[i];
            self.IntersectEdges(iNode.Edge1, iNode.Edge2, iNode.Pt);
            self.SwapPositionsInAEL(iNode.Edge1, iNode.Edge2);
        }
        self.m_IntersectList.clear();
    }

    /// `Round(double value)`: half away from zero, then `(cInt)` truncation.
    pub(crate) fn Round(value: f64) -> cInt {
        if value < 0.0 {
            crate::dotnet::cast_i64(value - 0.5)
        } else {
            crate::dotnet::cast_i64(value + 0.5)
        }
    }

    fn TopX(edge: &TEdge, currentY: cInt) -> cInt {
        if currentY == edge.Top.Y {
            return edge.Top.X;
        }
        edge.Bot.X.wrapping_add(Self::Round(edge.Dx * (currentY.wrapping_sub(edge.Bot.Y)) as f64))
    }

    fn IntersectPoint(&self, e1: usize, e2: usize) -> IntPoint {
        let (edge1, edge2) = (&self.edges[e1], &self.edges[e2]);
        let mut ip = IntPoint::default();
        //nb: with very large coordinate values, it's possible for SlopesEqual() to
        //return false but for the edge.Dx value be equal due to double precision rounding.
        if edge1.Dx == edge2.Dx {
            ip.Y = edge1.Curr.Y;
            ip.X = Self::TopX(edge1, ip.Y);
            return ip;
        }

        if edge1.Delta.X == 0 {
            ip.X = edge1.Bot.X;
            if edge2.Delta.Y == 0 {
                ip.Y = edge2.Bot.Y;
            } else {
                let b2 = edge2.Bot.Y as f64 - (edge2.Bot.X as f64 / edge2.Dx);
                ip.Y = Self::Round(ip.X as f64 / edge2.Dx + b2);
            }
        } else if edge2.Delta.X == 0 {
            ip.X = edge2.Bot.X;
            if edge1.Delta.Y == 0 {
                ip.Y = edge1.Bot.Y;
            } else {
                let b1 = edge1.Bot.Y as f64 - (edge1.Bot.X as f64 / edge1.Dx);
                ip.Y = Self::Round(ip.X as f64 / edge1.Dx + b1);
            }
        } else {
            let b1 = edge1.Bot.X as f64 - edge1.Bot.Y as f64 * edge1.Dx;
            let b2 = edge2.Bot.X as f64 - edge2.Bot.Y as f64 * edge2.Dx;
            let q = (b2 - b1) / (edge1.Dx - edge2.Dx);
            ip.Y = Self::Round(q);
            if edge1.Dx.abs() < edge2.Dx.abs() {
                ip.X = Self::Round(edge1.Dx * q + b1);
            } else {
                ip.X = Self::Round(edge2.Dx * q + b2);
            }
        }

        if ip.Y < edge1.Top.Y || ip.Y < edge2.Top.Y {
            if edge1.Top.Y > edge2.Top.Y {
                ip.Y = edge1.Top.Y;
            } else {
                ip.Y = edge2.Top.Y;
            }
            if edge1.Dx.abs() < edge2.Dx.abs() {
                ip.X = Self::TopX(edge1, ip.Y);
            } else {
                ip.X = Self::TopX(edge2, ip.Y);
            }
        }
        //finally, don't allow 'ip' to be BELOW curr.Y (ie bottom of scanbeam) ...
        if ip.Y > edge1.Curr.Y {
            ip.Y = edge1.Curr.Y;
            //better to use the more vertical edge to derive X ...
            if edge1.Dx.abs() > edge2.Dx.abs() {
                ip.X = Self::TopX(edge2, ip.Y);
            } else {
                ip.X = Self::TopX(edge1, ip.Y);
            }
        }
        ip
    }

    fn ProcessEdgesAtTopOfScanbeam(&mut self, topY: cInt) -> Result<(), Exception> {
        let mut e = self.m_ActiveEdges;
        while e != NIL {
            //1. process maxima, treating them as if they're 'bent' horizontal edges,
            //   but exclude maxima with horizontal edges. nb: e can't be a horizontal.
            let mut IsMaximaEdge = self.IsMaxima(e, topY as f64);

            if IsMaximaEdge {
                let eMaxPair = self.GetMaximaPairEx(e);
                IsMaximaEdge = eMaxPair == NIL || !self.IsHorizontal(eMaxPair);
            }

            if IsMaximaEdge {
                if self.StrictlySimple {
                    let x = self.edges[e].Top.X;
                    self.InsertMaxima(x);
                }
                let ePrev = self.edges[e].PrevInAEL;
                self.DoMaxima(e)?;
                if ePrev == NIL {
                    e = self.m_ActiveEdges;
                } else {
                    e = self.edges[ePrev].NextInAEL;
                }
            } else {
                //2. promote horizontal edges, otherwise update Curr.X and Curr.Y ...
                if self.IsIntermediate(e, topY as f64) && self.IsHorizontal(self.edges[e].NextInLML) {
                    self.UpdateEdgeIntoAEL(&mut e)?;
                    if self.edges[e].OutIdx >= 0 {
                        let b = self.edges[e].Bot;
                        self.AddOutPt(e, b);
                    }
                    self.AddEdgeToSEL(e);
                } else {
                    self.edges[e].Curr.X = Self::TopX(&self.edges[e], topY);
                    self.edges[e].Curr.Y = topY;
                }
                //When StrictlySimple and 'e' is being touched by another edge, then
                //make sure both edges have a vertex here ...
                if self.StrictlySimple {
                    let ePrev = self.edges[e].PrevInAEL;
                    if (self.edges[e].OutIdx >= 0)
                        && (self.edges[e].WindDelta != 0)
                        && ePrev != NIL
                        && (self.edges[ePrev].OutIdx >= 0)
                        && (self.edges[ePrev].Curr.X == self.edges[e].Curr.X)
                        && (self.edges[ePrev].WindDelta != 0)
                    {
                        let ip = self.edges[e].Curr;
                        let op = self.AddOutPt(ePrev, ip);
                        let op2 = self.AddOutPt(e, ip);
                        self.AddJoin(op, op2, ip); //StrictlySimple (type-3) join
                    }
                }

                e = self.edges[e].NextInAEL;
            }
        }

        //3. Process horizontals at the Top of the scanbeam ...
        self.ProcessHorizontals()?;
        self.m_Maxima = NIL;

        //4. Promote intermediate vertices ...
        e = self.m_ActiveEdges;
        while e != NIL {
            if self.IsIntermediate(e, topY as f64) {
                let mut op = NIL;
                if self.edges[e].OutIdx >= 0 {
                    let t = self.edges[e].Top;
                    op = self.AddOutPt(e, t);
                }
                self.UpdateEdgeIntoAEL(&mut e)?;

                //if output polygons share an edge, they'll need joining later ...
                let ePrev = self.edges[e].PrevInAEL;
                let eNext = self.edges[e].NextInAEL;
                let ee = self.edges[e];
                if ePrev != NIL
                    && self.edges[ePrev].Curr.X == ee.Bot.X
                    && self.edges[ePrev].Curr.Y == ee.Bot.Y
                    && op != NIL
                    && self.edges[ePrev].OutIdx >= 0
                    && self.edges[ePrev].Curr.Y > self.edges[ePrev].Top.Y
                    && Self::SlopesEqual4(ee.Curr, ee.Top, self.edges[ePrev].Curr, self.edges[ePrev].Top, self.m_UseFullRange)
                    && (ee.WindDelta != 0)
                    && (self.edges[ePrev].WindDelta != 0)
                {
                    let op2 = self.AddOutPt(ePrev, ee.Bot);
                    self.AddJoin(op, op2, ee.Top);
                } else if eNext != NIL
                    && self.edges[eNext].Curr.X == ee.Bot.X
                    && self.edges[eNext].Curr.Y == ee.Bot.Y
                    && op != NIL
                    && self.edges[eNext].OutIdx >= 0
                    && self.edges[eNext].Curr.Y > self.edges[eNext].Top.Y
                    && Self::SlopesEqual4(ee.Curr, ee.Top, self.edges[eNext].Curr, self.edges[eNext].Top, self.m_UseFullRange)
                    && (ee.WindDelta != 0)
                    && (self.edges[eNext].WindDelta != 0)
                {
                    let op2 = self.AddOutPt(eNext, ee.Bot);
                    self.AddJoin(op, op2, ee.Top);
                }
            }
            e = self.edges[e].NextInAEL;
        }
        Ok(())
    }

    fn DoMaxima(&mut self, e: usize) -> Result<(), Exception> {
        let eMaxPair = self.GetMaximaPairEx(e);
        if eMaxPair == NIL {
            if self.edges[e].OutIdx >= 0 {
                let t = self.edges[e].Top;
                self.AddOutPt(e, t);
            }
            self.DeleteFromAEL(e);
            return Ok(());
        }

        let mut eNext = self.edges[e].NextInAEL;
        while eNext != NIL && eNext != eMaxPair {
            let t = self.edges[e].Top;
            self.IntersectEdges(e, eNext, t);
            self.SwapPositionsInAEL(e, eNext);
            eNext = self.edges[e].NextInAEL;
        }

        if self.edges[e].OutIdx == Unassigned && self.edges[eMaxPair].OutIdx == Unassigned {
            self.DeleteFromAEL(e);
            self.DeleteFromAEL(eMaxPair);
        } else if self.edges[e].OutIdx >= 0 && self.edges[eMaxPair].OutIdx >= 0 {
            if self.edges[e].OutIdx >= 0 {
                let t = self.edges[e].Top;
                self.AddLocalMaxPoly(e, eMaxPair, t);
            }
            self.DeleteFromAEL(e);
            self.DeleteFromAEL(eMaxPair);
        } else if self.edges[e].WindDelta == 0 {
            if self.edges[e].OutIdx >= 0 {
                let t = self.edges[e].Top;
                self.AddOutPt(e, t);
                self.edges[e].OutIdx = Unassigned;
            }
            self.DeleteFromAEL(e);

            if self.edges[eMaxPair].OutIdx >= 0 {
                let t = self.edges[e].Top;
                self.AddOutPt(eMaxPair, t);
                self.edges[eMaxPair].OutIdx = Unassigned;
            }
            self.DeleteFromAEL(eMaxPair);
        } else {
            return Err(clipper_exception("DoMaxima error"));
        }
        Ok(())
    }

    /// `ReversePaths(Paths polys)`
    pub fn ReversePaths(polys: &mut Paths) {
        for poly in polys.iter_mut() {
            poly.reverse();
        }
    }

    /// `Orientation(Path poly)`
    pub fn Orientation(poly: &Path) -> bool {
        Self::Area(poly) >= 0.0
    }

    fn PointCount(&self, pts: usize) -> i32 {
        if pts == NIL {
            return 0;
        }
        let mut result = 0;
        let mut p = pts;
        loop {
            result += 1;
            p = self.outpts[p].Next;
            if p == pts {
                break;
            }
        }
        result
    }

    fn BuildResult(&self, polyg: &mut Paths) {
        polyg.clear();
        for i in 0..self.m_PolyOuts.len() {
            let outRec = self.m_PolyOuts[i];
            if self.outrecs[outRec].Pts == NIL {
                continue;
            }
            let mut p = self.outpts[self.outrecs[outRec].Pts].Prev;
            let cnt = self.PointCount(p);
            if cnt < 2 {
                continue;
            }
            let mut pg = Path::with_capacity(cnt as usize);
            for _ in 0..cnt {
                pg.push(self.outpts[p].Pt);
                p = self.outpts[p].Prev;
            }
            polyg.push(pg);
        }
    }

    fn BuildResult2(&mut self, polytree: &mut PolyTree) {
        polytree.Clear();
        polytree.nodes.truncate(1);

        //add each output polygon/contour to polytree ...
        for i in 0..self.m_PolyOuts.len() {
            let outRec = self.m_PolyOuts[i];
            let cnt = self.PointCount(self.outrecs[outRec].Pts);
            if (self.outrecs[outRec].IsOpen && cnt < 2) || (!self.outrecs[outRec].IsOpen && cnt < 3) {
                continue;
            }
            self.FixHoleLinkage(outRec);
            let pn = polytree.new_node();
            polytree.m_AllPolys.push(pn);
            self.outrecs[outRec].PolyNode = pn;
            let mut op = self.outpts[self.outrecs[outRec].Pts].Prev;
            for _ in 0..cnt {
                polytree.nodes[pn].m_polygon.push(self.outpts[op].Pt);
                op = self.outpts[op].Prev;
            }
        }

        //fixup PolyNode links etc ...
        for i in 0..self.m_PolyOuts.len() {
            let outRec = self.m_PolyOuts[i];
            let r = self.outrecs[outRec];
            if r.PolyNode == NIL {
                continue;
            } else if r.IsOpen {
                polytree.nodes[r.PolyNode].IsOpen = true;
                polytree.AddChild(PolyTree::ROOT, r.PolyNode);
            } else if r.FirstLeft != NIL && self.outrecs[r.FirstLeft].PolyNode != NIL {
                polytree.AddChild(self.outrecs[r.FirstLeft].PolyNode, r.PolyNode);
            } else {
                polytree.AddChild(PolyTree::ROOT, r.PolyNode);
            }
        }
    }

    fn FixupOutPolyline(&mut self, outrec: usize) {
        let mut pp = self.outrecs[outrec].Pts;
        let mut lastPP = self.outpts[pp].Prev;
        while pp != lastPP {
            pp = self.outpts[pp].Next;
            if self.outpts[pp].Pt == self.outpts[self.outpts[pp].Prev].Pt {
                if pp == lastPP {
                    lastPP = self.outpts[pp].Prev;
                }
                let tmpPP = self.outpts[pp].Prev;
                let ppn = self.outpts[pp].Next;
                self.outpts[tmpPP].Next = ppn;
                self.outpts[ppn].Prev = tmpPP;
                pp = tmpPP;
            }
        }
        if pp == self.outpts[pp].Prev {
            self.outrecs[outrec].Pts = NIL;
        }
    }

    /// FixupOutPolygon() - removes duplicate points and simplifies consecutive parallel edges
    /// by removing the middle vertex.
    fn FixupOutPolygon(&mut self, outRec: usize) {
        let mut lastOK = NIL;
        self.outrecs[outRec].BottomPt = NIL;
        let mut pp = self.outrecs[outRec].Pts;
        let preserveCol = self.PreserveCollinear || self.StrictlySimple;
        loop {
            let (prev, next) = (self.outpts[pp].Prev, self.outpts[pp].Next);
            if prev == pp || prev == next {
                self.outrecs[outRec].Pts = NIL;
                return;
            }
            let (pp_pt, prev_pt, next_pt) = (self.outpts[pp].Pt, self.outpts[prev].Pt, self.outpts[next].Pt);
            //test for duplicate points and collinear edges ...
            if (pp_pt == next_pt)
                || (pp_pt == prev_pt)
                || (Self::SlopesEqual3(prev_pt, pp_pt, next_pt, self.m_UseFullRange) && (!preserveCol || !Self::Pt2IsBetweenPt1AndPt3(prev_pt, pp_pt, next_pt)))
            {
                lastOK = NIL;
                self.outpts[prev].Next = next;
                self.outpts[next].Prev = prev;
                pp = prev;
            } else if pp == lastOK {
                break;
            } else {
                if lastOK == NIL {
                    lastOK = pp;
                }
                pp = next;
            }
        }
        self.outrecs[outRec].Pts = pp;
    }

    fn DupOutPt(&mut self, outPt: usize, InsertAfter: bool) -> usize {
        let o = self.outpts[outPt];
        let result = self.new_outpt(OutPt { Idx: o.Idx, Pt: o.Pt, Next: NIL, Prev: NIL });
        if InsertAfter {
            self.outpts[result].Next = o.Next;
            self.outpts[result].Prev = outPt;
            self.outpts[o.Next].Prev = result;
            self.outpts[outPt].Next = result;
        } else {
            self.outpts[result].Prev = o.Prev;
            self.outpts[result].Next = outPt;
            self.outpts[o.Prev].Next = result;
            self.outpts[outPt].Prev = result;
        }
        result
    }

    fn GetOverlap(a1: cInt, a2: cInt, b1: cInt, b2: cInt, Left: &mut cInt, Right: &mut cInt) -> bool {
        if a1 < a2 {
            if b1 < b2 {
                *Left = a1.max(b1);
                *Right = a2.min(b2);
            } else {
                *Left = a1.max(b2);
                *Right = a2.min(b1);
            }
        } else if b1 < b2 {
            *Left = a2.max(b1);
            *Right = a1.min(b2);
        } else {
            *Left = a2.max(b2);
            *Right = a1.min(b1);
        }
        *Left < *Right
    }

    #[allow(clippy::too_many_arguments)]
    fn JoinHorz(&mut self, mut op1: usize, mut op1b: usize, mut op2: usize, mut op2b: usize, Pt: IntPoint, DiscardLeft: bool) -> bool {
        let Dir1 = if self.outpts[op1].Pt.X > self.outpts[op1b].Pt.X { Direction::dRightToLeft } else { Direction::dLeftToRight };
        let Dir2 = if self.outpts[op2].Pt.X > self.outpts[op2b].Pt.X { Direction::dRightToLeft } else { Direction::dLeftToRight };
        if Dir1 == Dir2 {
            return false;
        }

        //When DiscardLeft, we want Op1b to be on the Left of Op1, otherwise we
        //want Op1b to be on the Right. (And likewise with Op2 and Op2b.)
        //So, to facilitate this while inserting Op1b and Op2b ...
        //when DiscardLeft, make sure we're AT or RIGHT of Pt before adding Op1b,
        //otherwise make sure we're AT or LEFT of Pt. (Likewise with Op2b.)
        let nx = |s: &Self, o: usize| s.outpts[s.outpts[o].Next].Pt;
        if Dir1 == Direction::dLeftToRight {
            while nx(self, op1).X <= Pt.X && nx(self, op1).X >= self.outpts[op1].Pt.X && nx(self, op1).Y == Pt.Y {
                op1 = self.outpts[op1].Next;
            }
            if DiscardLeft && (self.outpts[op1].Pt.X != Pt.X) {
                op1 = self.outpts[op1].Next;
            }
            op1b = self.DupOutPt(op1, !DiscardLeft);
            if self.outpts[op1b].Pt != Pt {
                op1 = op1b;
                self.outpts[op1].Pt = Pt;
                op1b = self.DupOutPt(op1, !DiscardLeft);
            }
        } else {
            while nx(self, op1).X >= Pt.X && nx(self, op1).X <= self.outpts[op1].Pt.X && nx(self, op1).Y == Pt.Y {
                op1 = self.outpts[op1].Next;
            }
            if !DiscardLeft && (self.outpts[op1].Pt.X != Pt.X) {
                op1 = self.outpts[op1].Next;
            }
            op1b = self.DupOutPt(op1, DiscardLeft);
            if self.outpts[op1b].Pt != Pt {
                op1 = op1b;
                self.outpts[op1].Pt = Pt;
                op1b = self.DupOutPt(op1, DiscardLeft);
            }
        }

        if Dir2 == Direction::dLeftToRight {
            while nx(self, op2).X <= Pt.X && nx(self, op2).X >= self.outpts[op2].Pt.X && nx(self, op2).Y == Pt.Y {
                op2 = self.outpts[op2].Next;
            }
            if DiscardLeft && (self.outpts[op2].Pt.X != Pt.X) {
                op2 = self.outpts[op2].Next;
            }
            op2b = self.DupOutPt(op2, !DiscardLeft);
            if self.outpts[op2b].Pt != Pt {
                op2 = op2b;
                self.outpts[op2].Pt = Pt;
                op2b = self.DupOutPt(op2, !DiscardLeft);
            }
        } else {
            while nx(self, op2).X >= Pt.X && nx(self, op2).X <= self.outpts[op2].Pt.X && nx(self, op2).Y == Pt.Y {
                op2 = self.outpts[op2].Next;
            }
            if !DiscardLeft && (self.outpts[op2].Pt.X != Pt.X) {
                op2 = self.outpts[op2].Next;
            }
            op2b = self.DupOutPt(op2, DiscardLeft);
            if self.outpts[op2b].Pt != Pt {
                op2 = op2b;
                self.outpts[op2].Pt = Pt;
                op2b = self.DupOutPt(op2, DiscardLeft);
            }
        }

        if (Dir1 == Direction::dLeftToRight) == DiscardLeft {
            self.outpts[op1].Prev = op2;
            self.outpts[op2].Next = op1;
            self.outpts[op1b].Next = op2b;
            self.outpts[op2b].Prev = op1b;
        } else {
            self.outpts[op1].Next = op2;
            self.outpts[op2].Prev = op1;
            self.outpts[op1b].Prev = op2b;
            self.outpts[op2b].Next = op1b;
        }
        true
    }

    /// `JoinPoints(Join j, OutRec outRec1, OutRec outRec2)`: `j` is the join's index, so the
    /// changes to its OutPt1/OutPt2 stay with it as with the C# reference.
    fn JoinPoints(&mut self, j: usize, outRec1: usize, outRec2: usize) -> bool {
        let jj = self.m_Joins[j];
        let mut op1 = jj.OutPt1;
        let mut op2 = jj.OutPt2;
        let mut op1b;
        let mut op2b;
        let OffPt = jj.OffPt;

        //There are 3 kinds of joins for output polygons ...
        //1. Horizontal joins where Join.OutPt1 & Join.OutPt2 are vertices anywhere
        //along (horizontal) collinear edges (& Join.OffPt is on the same horizontal).
        //2. Non-horizontal joins where Join.OutPt1 & Join.OutPt2 are at the same
        //location at the Bottom of the overlapping segment (& Join.OffPt is above).
        //3. StrictlySimple joins where edges touch but are not collinear and where
        //Join.OutPt1, Join.OutPt2 & Join.OffPt all share the same point.
        let isHorizontal = self.outpts[jj.OutPt1].Pt.Y == OffPt.Y;

        if isHorizontal && (OffPt == self.outpts[jj.OutPt1].Pt) && (OffPt == self.outpts[jj.OutPt2].Pt) {
            //Strictly Simple join ...
            if outRec1 != outRec2 {
                return false;
            }
            op1b = self.outpts[jj.OutPt1].Next;
            while op1b != op1 && (self.outpts[op1b].Pt == OffPt) {
                op1b = self.outpts[op1b].Next;
            }
            let reverse1 = self.outpts[op1b].Pt.Y > OffPt.Y;
            op2b = self.outpts[jj.OutPt2].Next;
            while op2b != op2 && (self.outpts[op2b].Pt == OffPt) {
                op2b = self.outpts[op2b].Next;
            }
            let reverse2 = self.outpts[op2b].Pt.Y > OffPt.Y;
            if reverse1 == reverse2 {
                return false;
            }
            if reverse1 {
                op1b = self.DupOutPt(op1, false);
                op2b = self.DupOutPt(op2, true);
                self.outpts[op1].Prev = op2;
                self.outpts[op2].Next = op1;
                self.outpts[op1b].Next = op2b;
                self.outpts[op2b].Prev = op1b;
            } else {
                op1b = self.DupOutPt(op1, true);
                op2b = self.DupOutPt(op2, false);
                self.outpts[op1].Next = op2;
                self.outpts[op2].Prev = op1;
                self.outpts[op1b].Prev = op2b;
                self.outpts[op2b].Next = op1b;
            }
            self.m_Joins[j].OutPt1 = op1;
            self.m_Joins[j].OutPt2 = op1b;
            true
        } else if isHorizontal {
            //treat horizontal joins differently to non-horizontal joins since with
            //them we're not yet sure where the overlapping is. OutPt1.Pt & OutPt2.Pt
            //may be anywhere along the horizontal edge.
            let o = |s: &Self, p: usize| s.outpts[p];
            op1b = op1;
            while o(self, o(self, op1).Prev).Pt.Y == o(self, op1).Pt.Y && o(self, op1).Prev != op1b && o(self, op1).Prev != op2 {
                op1 = o(self, op1).Prev;
            }
            while o(self, o(self, op1b).Next).Pt.Y == o(self, op1b).Pt.Y && o(self, op1b).Next != op1 && o(self, op1b).Next != op2 {
                op1b = o(self, op1b).Next;
            }
            if o(self, op1b).Next == op1 || o(self, op1b).Next == op2 {
                return false; //a flat 'polygon'
            }

            op2b = op2;
            while o(self, o(self, op2).Prev).Pt.Y == o(self, op2).Pt.Y && o(self, op2).Prev != op2b && o(self, op2).Prev != op1b {
                op2 = o(self, op2).Prev;
            }
            while o(self, o(self, op2b).Next).Pt.Y == o(self, op2b).Pt.Y && o(self, op2b).Next != op2 && o(self, op2b).Next != op1 {
                op2b = o(self, op2b).Next;
            }
            if o(self, op2b).Next == op2 || o(self, op2b).Next == op1 {
                return false; //a flat 'polygon'
            }

            let (mut Left, mut Right): (cInt, cInt) = (0, 0);
            //Op1 -. Op1b & Op2 -. Op2b are the extremites of the horizontal edges
            let (p1, p1b, p2, p2b) = (o(self, op1).Pt, o(self, op1b).Pt, o(self, op2).Pt, o(self, op2b).Pt);
            if !Self::GetOverlap(p1.X, p1b.X, p2.X, p2b.X, &mut Left, &mut Right) {
                return false;
            }

            //DiscardLeftSide: when overlapping edges are joined, a spike will created
            //which needs to be cleaned up. However, we don't want Op1 or Op2 caught up
            //on the discard Side as either may still be needed for other joins ...
            let (Pt, DiscardLeftSide) = if p1.X >= Left && p1.X <= Right {
                (p1, p1.X > p1b.X)
            } else if p2.X >= Left && p2.X <= Right {
                (p2, p2.X > p2b.X)
            } else if p1b.X >= Left && p1b.X <= Right {
                (p1b, p1b.X > p1.X)
            } else {
                (p2b, p2b.X > p2.X)
            };
            self.m_Joins[j].OutPt1 = op1;
            self.m_Joins[j].OutPt2 = op2;
            self.JoinHorz(op1, op1b, op2, op2b, Pt, DiscardLeftSide)
        } else {
            //nb: For non-horizontal joins ...
            //    1. Jr.OutPt1.Pt.Y == Jr.OutPt2.Pt.Y
            //    2. Jr.OutPt1.Pt > Jr.OffPt.Y

            //make sure the polygons are correctly oriented ...
            op1b = self.outpts[op1].Next;
            while (self.outpts[op1b].Pt == self.outpts[op1].Pt) && (op1b != op1) {
                op1b = self.outpts[op1b].Next;
            }
            let Reverse1 = (self.outpts[op1b].Pt.Y > self.outpts[op1].Pt.Y) || !Self::SlopesEqual3(self.outpts[op1].Pt, self.outpts[op1b].Pt, OffPt, self.m_UseFullRange);
            if Reverse1 {
                op1b = self.outpts[op1].Prev;
                while (self.outpts[op1b].Pt == self.outpts[op1].Pt) && (op1b != op1) {
                    op1b = self.outpts[op1b].Prev;
                }
                if (self.outpts[op1b].Pt.Y > self.outpts[op1].Pt.Y) || !Self::SlopesEqual3(self.outpts[op1].Pt, self.outpts[op1b].Pt, OffPt, self.m_UseFullRange) {
                    return false;
                }
            }
            op2b = self.outpts[op2].Next;
            while (self.outpts[op2b].Pt == self.outpts[op2].Pt) && (op2b != op2) {
                op2b = self.outpts[op2b].Next;
            }
            let Reverse2 = (self.outpts[op2b].Pt.Y > self.outpts[op2].Pt.Y) || !Self::SlopesEqual3(self.outpts[op2].Pt, self.outpts[op2b].Pt, OffPt, self.m_UseFullRange);
            if Reverse2 {
                op2b = self.outpts[op2].Prev;
                while (self.outpts[op2b].Pt == self.outpts[op2].Pt) && (op2b != op2) {
                    op2b = self.outpts[op2b].Prev;
                }
                if (self.outpts[op2b].Pt.Y > self.outpts[op2].Pt.Y) || !Self::SlopesEqual3(self.outpts[op2].Pt, self.outpts[op2b].Pt, OffPt, self.m_UseFullRange) {
                    return false;
                }
            }

            if (op1b == op1) || (op2b == op2) || (op1b == op2b) || ((outRec1 == outRec2) && (Reverse1 == Reverse2)) {
                return false;
            }

            if Reverse1 {
                op1b = self.DupOutPt(op1, false);
                op2b = self.DupOutPt(op2, true);
                self.outpts[op1].Prev = op2;
                self.outpts[op2].Next = op1;
                self.outpts[op1b].Next = op2b;
                self.outpts[op2b].Prev = op1b;
            } else {
                op1b = self.DupOutPt(op1, true);
                op2b = self.DupOutPt(op2, false);
                self.outpts[op1].Next = op2;
                self.outpts[op2].Prev = op1;
                self.outpts[op1b].Prev = op2b;
                self.outpts[op2b].Next = op1b;
            }
            self.m_Joins[j].OutPt1 = op1;
            self.m_Joins[j].OutPt2 = op1b;
            true
        }
    }

    /// `PointInPolygon(IntPoint pt, Path path)`: 0 if false, +1 if true, -1 if pt ON polygon
    /// boundary (Hormann & Agathos, "The Point in Polygon Problem for Arbitrary Polygons").
    pub fn PointInPolygon(pt: IntPoint, path: &Path) -> i32 {
        let mut result = 0;
        let cnt = path.len();
        if cnt < 3 {
            return 0;
        }
        let mut ip = path[0];
        for i in 1..=cnt {
            let ipNext = if i == cnt { path[0] } else { path[i] };
            if ipNext.Y == pt.Y && ((ipNext.X == pt.X) || (ip.Y == pt.Y && ((ipNext.X > pt.X) == (ip.X < pt.X)))) {
                return -1;
            }
            if (ip.Y < pt.Y) != (ipNext.Y < pt.Y) {
                let d = || ip.X.wrapping_sub(pt.X) as f64 * ipNext.Y.wrapping_sub(pt.Y) as f64 - ipNext.X.wrapping_sub(pt.X) as f64 * ip.Y.wrapping_sub(pt.Y) as f64;
                if ip.X >= pt.X {
                    if ipNext.X > pt.X {
                        result = 1 - result;
                    } else {
                        let d = d();
                        if d == 0.0 {
                            return -1;
                        } else if (d > 0.0) == (ipNext.Y > ip.Y) {
                            result = 1 - result;
                        }
                    }
                } else if ipNext.X > pt.X {
                    let d = d();
                    if d == 0.0 {
                        return -1;
                    } else if (d > 0.0) == (ipNext.Y > ip.Y) {
                        result = 1 - result;
                    }
                }
            }
            ip = ipNext;
        }
        result
    }

    /// `PointInPolygon(IntPoint pt, OutPt op)` (private overload)
    fn PointInPolygonOp(&self, pt: IntPoint, mut op: usize) -> i32 {
        let mut result = 0;
        let startOp = op;
        let (ptx, pty) = (pt.X, pt.Y);
        let (mut poly0x, mut poly0y) = (self.outpts[op].Pt.X, self.outpts[op].Pt.Y);
        loop {
            op = self.outpts[op].Next;
            let (poly1x, poly1y) = (self.outpts[op].Pt.X, self.outpts[op].Pt.Y);

            if poly1y == pty && ((poly1x == ptx) || (poly0y == pty && ((poly1x > ptx) == (poly0x < ptx)))) {
                return -1;
            }
            if (poly0y < pty) != (poly1y < pty) {
                let d = || poly0x.wrapping_sub(ptx) as f64 * poly1y.wrapping_sub(pty) as f64 - poly1x.wrapping_sub(ptx) as f64 * poly0y.wrapping_sub(pty) as f64;
                if poly0x >= ptx {
                    if poly1x > ptx {
                        result = 1 - result;
                    } else {
                        let d = d();
                        if d == 0.0 {
                            return -1;
                        }
                        if (d > 0.0) == (poly1y > poly0y) {
                            result = 1 - result;
                        }
                    }
                } else if poly1x > ptx {
                    let d = d();
                    if d == 0.0 {
                        return -1;
                    }
                    if (d > 0.0) == (poly1y > poly0y) {
                        result = 1 - result;
                    }
                }
            }
            poly0x = poly1x;
            poly0y = poly1y;
            if startOp == op {
                break;
            }
        }
        result
    }

    fn Poly2ContainsPoly1(&self, outPt1: usize, outPt2: usize) -> bool {
        let mut op = outPt1;
        loop {
            //nb: PointInPolygon returns 0 if false, +1 if true, -1 if pt on polygon
            let res = self.PointInPolygonOp(self.outpts[op].Pt, outPt2);
            if res >= 0 {
                return res > 0;
            }
            op = self.outpts[op].Next;
            if op == outPt1 {
                break;
            }
        }
        true
    }

    fn FixupFirstLefts1(&mut self, OldOutRec: usize, NewOutRec: usize) {
        for i in 0..self.m_PolyOuts.len() {
            let outRec = self.m_PolyOuts[i];
            let firstLeft = self.ParseFirstLeft(self.outrecs[outRec].FirstLeft);
            if self.outrecs[outRec].Pts != NIL && firstLeft == OldOutRec && self.Poly2ContainsPoly1(self.outrecs[outRec].Pts, self.outrecs[NewOutRec].Pts) {
                self.outrecs[outRec].FirstLeft = NewOutRec;
            }
        }
    }

    /// A polygon has split into two such that one is now the inner of the other. It's
    /// possible that these polygons now wrap around other polygons, so check every polygon
    /// that's also contained by OuterOutRec's FirstLeft container (including nil) to see if
    /// they've become inner to the new inner polygon ...
    fn FixupFirstLefts2(&mut self, innerOutRec: usize, outerOutRec: usize) {
        let orfl = self.outrecs[outerOutRec].FirstLeft;
        for i in 0..self.m_PolyOuts.len() {
            let outRec = self.m_PolyOuts[i];
            if self.outrecs[outRec].Pts == NIL || outRec == outerOutRec || outRec == innerOutRec {
                continue;
            }
            let firstLeft = self.ParseFirstLeft(self.outrecs[outRec].FirstLeft);
            if firstLeft != orfl && firstLeft != innerOutRec && firstLeft != outerOutRec {
                continue;
            }
            if self.Poly2ContainsPoly1(self.outrecs[outRec].Pts, self.outrecs[innerOutRec].Pts) {
                self.outrecs[outRec].FirstLeft = innerOutRec;
            } else if self.Poly2ContainsPoly1(self.outrecs[outRec].Pts, self.outrecs[outerOutRec].Pts) {
                self.outrecs[outRec].FirstLeft = outerOutRec;
            } else if self.outrecs[outRec].FirstLeft == innerOutRec || self.outrecs[outRec].FirstLeft == outerOutRec {
                self.outrecs[outRec].FirstLeft = orfl;
            }
        }
    }

    /// same as FixupFirstLefts1 but doesn't call Poly2ContainsPoly1()
    fn FixupFirstLefts3(&mut self, OldOutRec: usize, NewOutRec: usize) {
        for i in 0..self.m_PolyOuts.len() {
            let outRec = self.m_PolyOuts[i];
            let firstLeft = self.ParseFirstLeft(self.outrecs[outRec].FirstLeft);
            if self.outrecs[outRec].Pts != NIL && firstLeft == OldOutRec {
                self.outrecs[outRec].FirstLeft = NewOutRec;
            }
        }
    }

    fn ParseFirstLeft(&self, mut FirstLeft: usize) -> usize {
        while FirstLeft != NIL && self.outrecs[FirstLeft].Pts == NIL {
            FirstLeft = self.outrecs[FirstLeft].FirstLeft;
        }
        FirstLeft
    }

    fn JoinCommonEdges(&mut self) {
        for i in 0..self.m_Joins.len() {
            let join = self.m_Joins[i];

            let outRec1 = self.GetOutRec(self.outpts[join.OutPt1].Idx as usize);
            let mut outRec2 = self.GetOutRec(self.outpts[join.OutPt2].Idx as usize);

            if self.outrecs[outRec1].Pts == NIL || self.outrecs[outRec2].Pts == NIL {
                continue;
            }
            if self.outrecs[outRec1].IsOpen || self.outrecs[outRec2].IsOpen {
                continue;
            }

            //get the polygon fragment with the correct hole state (FirstLeft)
            //before calling JoinPoints() ...
            let holeStateRec = if outRec1 == outRec2 {
                outRec1
            } else if self.OutRec1RightOfOutRec2(outRec1, outRec2) {
                outRec2
            } else if self.OutRec1RightOfOutRec2(outRec2, outRec1) {
                outRec1
            } else {
                self.GetLowermostRec(outRec1, outRec2)
            };

            if !self.JoinPoints(i, outRec1, outRec2) {
                continue;
            }
            let join = self.m_Joins[i];

            if outRec1 == outRec2 {
                //instead of joining two polygons, we've just created a new one by
                //splitting one polygon into two.
                self.outrecs[outRec1].Pts = join.OutPt1;
                self.outrecs[outRec1].BottomPt = NIL;
                outRec2 = self.CreateOutRec();
                self.outrecs[outRec2].Pts = join.OutPt2;

                //update all OutRec2.Pts Idx's ...
                self.UpdateOutPtIdxs(outRec2);

                if self.Poly2ContainsPoly1(self.outrecs[outRec2].Pts, self.outrecs[outRec1].Pts) {
                    //outRec1 contains outRec2 ...
                    self.outrecs[outRec2].IsHole = !self.outrecs[outRec1].IsHole;
                    self.outrecs[outRec2].FirstLeft = outRec1;

                    if self.m_UsingPolyTree {
                        self.FixupFirstLefts2(outRec2, outRec1);
                    }

                    if (self.outrecs[outRec2].IsHole ^ self.ReverseSolution) == (self.AreaRec(outRec2) > 0.0) {
                        let p = self.outrecs[outRec2].Pts;
                        self.ReversePolyPtLinks(p);
                    }
                } else if self.Poly2ContainsPoly1(self.outrecs[outRec1].Pts, self.outrecs[outRec2].Pts) {
                    //outRec2 contains outRec1 ...
                    self.outrecs[outRec2].IsHole = self.outrecs[outRec1].IsHole;
                    self.outrecs[outRec1].IsHole = !self.outrecs[outRec2].IsHole;
                    self.outrecs[outRec2].FirstLeft = self.outrecs[outRec1].FirstLeft;
                    self.outrecs[outRec1].FirstLeft = outRec2;

                    if self.m_UsingPolyTree {
                        self.FixupFirstLefts2(outRec1, outRec2);
                    }

                    if (self.outrecs[outRec1].IsHole ^ self.ReverseSolution) == (self.AreaRec(outRec1) > 0.0) {
                        let p = self.outrecs[outRec1].Pts;
                        self.ReversePolyPtLinks(p);
                    }
                } else {
                    //the 2 polygons are completely separate ...
                    self.outrecs[outRec2].IsHole = self.outrecs[outRec1].IsHole;
                    self.outrecs[outRec2].FirstLeft = self.outrecs[outRec1].FirstLeft;

                    //fixup FirstLeft pointers that may need reassigning to OutRec2
                    if self.m_UsingPolyTree {
                        self.FixupFirstLefts1(outRec1, outRec2);
                    }
                }
            } else {
                //joined 2 polygons together ...

                self.outrecs[outRec2].Pts = NIL;
                self.outrecs[outRec2].BottomPt = NIL;
                self.outrecs[outRec2].Idx = self.outrecs[outRec1].Idx;

                self.outrecs[outRec1].IsHole = self.outrecs[holeStateRec].IsHole;
                if holeStateRec == outRec2 {
                    self.outrecs[outRec1].FirstLeft = self.outrecs[outRec2].FirstLeft;
                }
                self.outrecs[outRec2].FirstLeft = outRec1;

                //fixup FirstLeft pointers that may need reassigning to OutRec1
                if self.m_UsingPolyTree {
                    self.FixupFirstLefts3(outRec2, outRec1);
                }
            }
        }
    }

    fn UpdateOutPtIdxs(&mut self, outrec: usize) {
        let start = self.outrecs[outrec].Pts;
        let idx = self.outrecs[outrec].Idx;
        let mut op = start;
        loop {
            self.outpts[op].Idx = idx;
            op = self.outpts[op].Prev;
            if op == start {
                break;
            }
        }
    }

    fn DoSimplePolygons(&mut self) {
        let mut i = 0;
        while i < self.m_PolyOuts.len() {
            let outrec = self.m_PolyOuts[i];
            i += 1;
            let mut op = self.outrecs[outrec].Pts;
            if op == NIL || self.outrecs[outrec].IsOpen {
                continue;
            }
            loop {
                //for each Pt in Polygon until duplicate found do ...
                let mut op2 = self.outpts[op].Next;
                while op2 != self.outrecs[outrec].Pts {
                    if (self.outpts[op].Pt == self.outpts[op2].Pt) && self.outpts[op2].Next != op && self.outpts[op2].Prev != op {
                        //split the polygon into two ...
                        let op3 = self.outpts[op].Prev;
                        let op4 = self.outpts[op2].Prev;
                        self.outpts[op].Prev = op4;
                        self.outpts[op4].Next = op;
                        self.outpts[op2].Prev = op3;
                        self.outpts[op3].Next = op2;

                        self.outrecs[outrec].Pts = op;
                        let outrec2 = self.CreateOutRec();
                        self.outrecs[outrec2].Pts = op2;
                        self.UpdateOutPtIdxs(outrec2);
                        if self.Poly2ContainsPoly1(self.outrecs[outrec2].Pts, self.outrecs[outrec].Pts) {
                            //OutRec2 is contained by OutRec1 ...
                            self.outrecs[outrec2].IsHole = !self.outrecs[outrec].IsHole;
                            self.outrecs[outrec2].FirstLeft = outrec;
                            if self.m_UsingPolyTree {
                                self.FixupFirstLefts2(outrec2, outrec);
                            }
                        } else if self.Poly2ContainsPoly1(self.outrecs[outrec].Pts, self.outrecs[outrec2].Pts) {
                            //OutRec1 is contained by OutRec2 ...
                            self.outrecs[outrec2].IsHole = self.outrecs[outrec].IsHole;
                            self.outrecs[outrec].IsHole = !self.outrecs[outrec2].IsHole;
                            self.outrecs[outrec2].FirstLeft = self.outrecs[outrec].FirstLeft;
                            self.outrecs[outrec].FirstLeft = outrec2;
                            if self.m_UsingPolyTree {
                                self.FixupFirstLefts2(outrec, outrec2);
                            }
                        } else {
                            //the 2 polygons are separate ...
                            self.outrecs[outrec2].IsHole = self.outrecs[outrec].IsHole;
                            self.outrecs[outrec2].FirstLeft = self.outrecs[outrec].FirstLeft;
                            if self.m_UsingPolyTree {
                                self.FixupFirstLefts1(outrec, outrec2);
                            }
                        }
                        op2 = op; //ie get ready for the next iteration
                    }
                    op2 = self.outpts[op2].Next;
                }
                op = self.outpts[op].Next;
                if op == self.outrecs[outrec].Pts {
                    break;
                }
            }
        }
    }

    /// `Area(Path poly)`
    pub fn Area(poly: &Path) -> f64 {
        let cnt = poly.len();
        if cnt < 3 {
            return 0.0;
        }
        let mut a = 0.0;
        let mut j = cnt - 1;
        for i in 0..cnt {
            a += (poly[j].X as f64 + poly[i].X as f64) * (poly[j].Y as f64 - poly[i].Y as f64);
            j = i;
        }
        -a * 0.5
    }

    fn AreaRec(&self, outRec: usize) -> f64 {
        self.AreaOp(self.outrecs[outRec].Pts)
    }

    fn AreaOp(&self, mut op: usize) -> f64 {
        let opFirst = op;
        if op == NIL {
            return 0.0;
        }
        let mut a = 0.0;
        loop {
            let prev = self.outpts[self.outpts[op].Prev].Pt;
            let p = self.outpts[op].Pt;
            a += (prev.X.wrapping_add(p.X)) as f64 * (prev.Y.wrapping_sub(p.Y)) as f64;
            op = self.outpts[op].Next;
            if op == opFirst {
                break;
            }
        }
        a * 0.5
    }

    //------------------------------------------------------------------------------
    // SimplifyPolygon functions ...
    // Convert self-intersecting polygons into simple polygons
    //------------------------------------------------------------------------------

    /// `SimplifyPolygon(Path poly, PolyFillType fillType = pftEvenOdd)`
    pub fn SimplifyPolygon(poly: &Path, fillType: PolyFillType) -> Result<Paths, Exception> {
        let mut result = Paths::new();
        let mut c = Clipper::new(0);
        c.StrictlySimple = true;
        c.AddPath(poly, PolyType::ptSubject, true)?;
        c.Execute_paths(ClipType::ctUnion, &mut result, fillType, fillType)?;
        Ok(result)
    }

    /// `SimplifyPolygons(Paths polys, PolyFillType fillType = pftEvenOdd)`
    pub fn SimplifyPolygons(polys: &Paths, fillType: PolyFillType) -> Result<Paths, Exception> {
        let mut result = Paths::new();
        let mut c = Clipper::new(0);
        c.StrictlySimple = true;
        c.AddPaths(polys, PolyType::ptSubject, true)?;
        c.Execute_paths(ClipType::ctUnion, &mut result, fillType, fillType)?;
        Ok(result)
    }

    #[allow(dead_code)]
    fn DistanceSqrd(pt1: IntPoint, pt2: IntPoint) -> f64 {
        let dx = pt1.X as f64 - pt2.X as f64;
        let dy = pt1.Y as f64 - pt2.Y as f64;
        dx * dx + dy * dy
    }

    /// The perpendicular distance (squared) of pt from the line ln1-ln2, from its general
    /// form Ax + By + C = 0.
    fn DistanceFromLineSqrd(pt: IntPoint, ln1: IntPoint, ln2: IntPoint) -> f64 {
        let A = ln1.Y.wrapping_sub(ln2.Y) as f64;
        let B = ln2.X.wrapping_sub(ln1.X) as f64;
        let mut C = A * ln1.X as f64 + B * ln1.Y as f64;
        C = A * pt.X as f64 + B * pt.Y as f64 - C;
        (C * C) / (A * A + B * B)
    }

    /// this function is more accurate when the point that's GEOMETRICALLY between the other
    /// 2 points is the one that's tested for distance. nb: with 'spikes', either pt1 or pt3
    /// is geometrically between the other pts
    fn SlopesNearCollinear(pt1: IntPoint, pt2: IntPoint, pt3: IntPoint, distSqrd: f64) -> bool {
        if pt1.X.wrapping_sub(pt2.X).wrapping_abs() > pt1.Y.wrapping_sub(pt2.Y).wrapping_abs() {
            if (pt1.X > pt2.X) == (pt1.X < pt3.X) {
                Self::DistanceFromLineSqrd(pt1, pt2, pt3) < distSqrd
            } else if (pt2.X > pt1.X) == (pt2.X < pt3.X) {
                Self::DistanceFromLineSqrd(pt2, pt1, pt3) < distSqrd
            } else {
                Self::DistanceFromLineSqrd(pt3, pt1, pt2) < distSqrd
            }
        } else if (pt1.Y > pt2.Y) == (pt1.Y < pt3.Y) {
            Self::DistanceFromLineSqrd(pt1, pt2, pt3) < distSqrd
        } else if (pt2.Y > pt1.Y) == (pt2.Y < pt3.Y) {
            Self::DistanceFromLineSqrd(pt2, pt1, pt3) < distSqrd
        } else {
            Self::DistanceFromLineSqrd(pt3, pt1, pt2) < distSqrd
        }
    }

    fn PointsAreClose(pt1: IntPoint, pt2: IntPoint, distSqrd: f64) -> bool {
        let dx = pt1.X as f64 - pt2.X as f64;
        let dy = pt1.Y as f64 - pt2.Y as f64;
        (dx * dx) + (dy * dy) <= distSqrd
    }

    /// `CleanPolygon(Path path, double distance = 1.415)`: distance = proximity in
    /// units/pixels below which vertices will be stripped. Default ~= sqrt(2) so when
    /// adjacent vertices or semi-adjacent vertices have both x & y coords within 1 unit,
    /// then the second vertex will be stripped.
    pub fn CleanPolygon(path: &Path, distance: f64) -> Path {
        let mut cnt = path.len() as i32;

        if cnt == 0 {
            return Path::new();
        }

        // the OutPt ring, local to this call
        let n = cnt as usize;
        let mut pts: Vec<OutPt> = (0..n).map(|i| OutPt { Idx: 0, Pt: path[i], Next: (i + 1) % n, Prev: NIL }).collect();
        for i in 0..n {
            let next = pts[i].Next;
            pts[next].Prev = i;
        }
        // ExcludeOp(op)
        let exclude = |pts: &mut Vec<OutPt>, op: usize| -> usize {
            let result = pts[op].Prev;
            let next = pts[op].Next;
            pts[result].Next = next;
            pts[next].Prev = result;
            pts[result].Idx = 0;
            result
        };

        let distSqrd = distance * distance;
        let mut op = 0usize;
        while pts[op].Idx == 0 && pts[op].Next != pts[op].Prev {
            let (prev, next) = (pts[op].Prev, pts[op].Next);
            if Self::PointsAreClose(pts[op].Pt, pts[prev].Pt, distSqrd) {
                op = exclude(&mut pts, op);
                cnt -= 1;
            } else if Self::PointsAreClose(pts[prev].Pt, pts[next].Pt, distSqrd) {
                exclude(&mut pts, next);
                op = exclude(&mut pts, op);
                cnt -= 2;
            } else if Self::SlopesNearCollinear(pts[prev].Pt, pts[op].Pt, pts[next].Pt, distSqrd) {
                op = exclude(&mut pts, op);
                cnt -= 1;
            } else {
                pts[op].Idx = 1;
                op = pts[op].Next;
            }
        }

        if cnt < 3 {
            cnt = 0;
        }
        let mut result = Path::with_capacity(cnt as usize);
        for _ in 0..cnt {
            result.push(pts[op].Pt);
            op = pts[op].Next;
        }
        result
    }

    /// `CleanPolygons(Paths polys, double distance = 1.415)`
    pub fn CleanPolygons(polys: &Paths, distance: f64) -> Paths {
        polys.iter().map(|p| Self::CleanPolygon(p, distance)).collect()
    }

    fn Minkowski(pattern: &Path, path: &Path, IsSum: bool, IsClosed: bool) -> Paths {
        let delta = if IsClosed { 1 } else { 0 };
        let polyCnt = pattern.len();
        let pathCnt = path.len();
        let mut result = Paths::with_capacity(pathCnt);
        for i in 0..pathCnt {
            let mut p = Path::with_capacity(polyCnt);
            for ip in pattern {
                if IsSum {
                    p.push(IntPoint::new(path[i].X.wrapping_add(ip.X), path[i].Y.wrapping_add(ip.Y)));
                } else {
                    p.push(IntPoint::new(path[i].X.wrapping_sub(ip.X), path[i].Y.wrapping_sub(ip.Y)));
                }
            }
            result.push(p);
        }

        let mut quads = Paths::new();
        let upto = pathCnt as i64 - 1 + delta;
        let mut i: i64 = 0;
        while i < upto {
            let iu = i as usize;
            for j in 0..polyCnt {
                let mut quad = Path::with_capacity(4);
                quad.push(result[iu % pathCnt][j % polyCnt]);
                quad.push(result[(iu + 1) % pathCnt][j % polyCnt]);
                quad.push(result[(iu + 1) % pathCnt][(j + 1) % polyCnt]);
                quad.push(result[iu % pathCnt][(j + 1) % polyCnt]);
                if !Self::Orientation(&quad) {
                    quad.reverse();
                }
                quads.push(quad);
            }
            i += 1;
        }
        quads
    }

    /// `MinkowskiSum(Path pattern, Path path, bool pathIsClosed)`
    pub fn MinkowskiSum(pattern: &Path, path: &Path, pathIsClosed: bool) -> Result<Paths, Exception> {
        let mut paths = Self::Minkowski(pattern, path, true, pathIsClosed);
        let mut c = Clipper::new(0);
        c.AddPaths(&paths, PolyType::ptSubject, true)?;
        c.Execute_paths(ClipType::ctUnion, &mut paths, PolyFillType::pftNonZero, PolyFillType::pftNonZero)?;
        Ok(paths)
    }

    fn TranslatePath(path: &Path, delta: IntPoint) -> Path {
        path.iter().map(|p| IntPoint::new(p.X.wrapping_add(delta.X), p.Y.wrapping_add(delta.Y))).collect()
    }

    /// `MinkowskiSum(Path pattern, Paths paths, bool pathIsClosed)`
    pub fn MinkowskiSumPaths(pattern: &Path, paths: &Paths, pathIsClosed: bool) -> Result<Paths, Exception> {
        let mut solution = Paths::new();
        let mut c = Clipper::new(0);
        for p in paths {
            let tmp = Self::Minkowski(pattern, p, true, pathIsClosed);
            c.AddPaths(&tmp, PolyType::ptSubject, true)?;
            if pathIsClosed {
                let first = *pattern.first().ok_or_else(crate::dotnet::index_out_of_range)?;
                let path = Self::TranslatePath(p, first);
                c.AddPath(&path, PolyType::ptClip, true)?;
            }
        }
        c.Execute_paths(ClipType::ctUnion, &mut solution, PolyFillType::pftNonZero, PolyFillType::pftNonZero)?;
        Ok(solution)
    }

    /// `MinkowskiDiff(Path poly1, Path poly2)`
    pub fn MinkowskiDiff(poly1: &Path, poly2: &Path) -> Result<Paths, Exception> {
        let mut paths = Self::Minkowski(poly1, poly2, false, true);
        let mut c = Clipper::new(0);
        c.AddPaths(&paths, PolyType::ptSubject, true)?;
        c.Execute_paths(ClipType::ctUnion, &mut paths, PolyFillType::pftNonZero, PolyFillType::pftNonZero)?;
        Ok(paths)
    }

    /// `PolyTreeToPaths(PolyTree polytree)`
    pub fn PolyTreeToPaths(polytree: &PolyTree) -> Paths {
        let mut result = Paths::new();
        Self::AddPolyNodeToPaths(polytree, PolyTree::ROOT, NodeType::ntAny, &mut result);
        result
    }

    fn AddPolyNodeToPaths(tree: &PolyTree, polynode: usize, nt: NodeType, paths: &mut Paths) {
        let mut matched = true;
        match nt {
            NodeType::ntOpen => return,
            NodeType::ntClosed => matched = !tree.nodes[polynode].IsOpen,
            NodeType::ntAny => {}
        }

        if !tree.nodes[polynode].m_polygon.is_empty() && matched {
            paths.push(tree.nodes[polynode].m_polygon.clone());
        }
        for pn in tree.nodes[polynode].m_Childs.clone() {
            Self::AddPolyNodeToPaths(tree, pn, nt, paths);
        }
    }

    /// `ReversePaths(PolyTreeToPaths(polytree))`: the C# list holds the nodes' own contour
    /// lists, so reversing it reverses the tree's contours in place (Grid.CreateRotary relies
    /// on this). Same traversal as PolyTreeToPaths: every non-empty contour.
    pub fn ReversePolyTreePaths(polytree: &mut PolyTree) {
        fn walk(tree: &mut PolyTree, node: usize) {
            if !tree.nodes[node].m_polygon.is_empty() {
                tree.nodes[node].m_polygon.reverse();
            }
            for c in tree.nodes[node].m_Childs.clone() {
                walk(tree, c);
            }
        }
        walk(polytree, PolyTree::ROOT);
    }

    /// `OpenPathsFromPolyTree(PolyTree polytree)`
    pub fn OpenPathsFromPolyTree(polytree: &PolyTree) -> Paths {
        let mut result = Paths::new();
        for &c in &polytree.nodes[PolyTree::ROOT].m_Childs {
            if polytree.nodes[c].IsOpen {
                result.push(polytree.nodes[c].m_polygon.clone());
            }
        }
        result
    }

    /// `ClosedPathsFromPolyTree(PolyTree polytree)`
    pub fn ClosedPathsFromPolyTree(polytree: &PolyTree) -> Paths {
        let mut result = Paths::new();
        Self::AddPolyNodeToPaths(polytree, PolyTree::ROOT, NodeType::ntClosed, &mut result);
        result
    }
} //end Clipper

/// `internal enum NodeType`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum NodeType {
    ntAny,
    #[allow(dead_code)]
    ntOpen,
    ntClosed,
}

const two_pi: f64 = std::f64::consts::PI * 2.0;
const def_arc_tolerance: f64 = 0.25;

/// `public class ClipperOffset`
pub struct ClipperOffset {
    m_destPolys: Paths,
    m_srcPoly: Path,
    m_destPoly: Path,
    m_normals: Vec<DoublePoint>,
    m_delta: f64,
    m_sinA: f64,
    m_sin: f64,
    m_cos: f64,
    m_miterLim: f64,
    m_StepsPerRad: f64,
    m_lowest: IntPoint,
    /// `m_polyNodes`: a PolyNode whose children are the added paths
    m_polyNodes: PolyTree,
    /// `ArcTolerance`
    pub ArcTolerance: f64,
    /// `MiterLimit`
    pub MiterLimit: f64,
}

impl ClipperOffset {
    /// `new ClipperOffset(double miterLimit = 2.0, double arcTolerance = def_arc_tolerance)`
    pub fn new(miterLimit: f64, arcTolerance: f64) -> Self {
        ClipperOffset {
            m_destPolys: Paths::new(),
            m_srcPoly: Path::new(),
            m_destPoly: Path::new(),
            m_normals: Vec::new(),
            m_delta: 0.0,
            m_sinA: 0.0,
            m_sin: 0.0,
            m_cos: 0.0,
            m_miterLim: 0.0,
            m_StepsPerRad: 0.0,
            m_lowest: IntPoint::new(-1, 0),
            m_polyNodes: PolyTree::new(),
            ArcTolerance: arcTolerance,
            MiterLimit: miterLimit,
        }
    }

    /// `Clear()`
    pub fn Clear(&mut self) {
        self.m_polyNodes.nodes[PolyTree::ROOT].m_Childs.clear();
        self.m_polyNodes.nodes.truncate(1);
        self.m_lowest.X = -1;
    }

    /// `Round(double value)`
    pub(crate) fn Round(value: f64) -> cInt {
        Clipper::Round(value)
    }

    fn child(&self, i: usize) -> &PolyNode {
        &self.m_polyNodes.nodes[self.m_polyNodes.nodes[PolyTree::ROOT].m_Childs[i]]
    }

    /// `AddPath(Path path, JoinType joinType, EndType endType)`
    pub fn AddPath(&mut self, path: &Path, joinType: JoinType, endType: EndType) {
        let mut highI = path.len() as i32 - 1;
        if highI < 0 {
            return;
        }
        let mut poly = Path::new();

        //strip duplicate points from path and also get index to the lowest point ...
        if endType == EndType::etClosedLine || endType == EndType::etClosedPolygon {
            while highI > 0 && path[0] == path[highI as usize] {
                highI -= 1;
            }
        }
        poly.push(path[0]);
        let (mut j, mut k) = (0usize, 0usize);
        for i in 1..=(highI as usize) {
            if poly[j] != path[i] {
                j += 1;
                poly.push(path[i]);
                if path[i].Y > poly[k].Y || (path[i].Y == poly[k].Y && path[i].X < poly[k].X) {
                    k = j;
                }
            }
        }
        if endType == EndType::etClosedPolygon && j < 2 {
            return;
        }

        let newNode = self.m_polyNodes.new_node();
        self.m_polyNodes.nodes[newNode].m_jointype = joinType;
        self.m_polyNodes.nodes[newNode].m_endtype = endType;
        self.m_polyNodes.nodes[newNode].m_polygon = poly;
        self.m_polyNodes.AddChild(PolyTree::ROOT, newNode);

        //if this path's lowest pt is lower than all the others then update m_lowest
        if endType != EndType::etClosedPolygon {
            return;
        }
        let count = self.m_polyNodes.ChildCount(PolyTree::ROOT) as cInt;
        if self.m_lowest.X < 0 {
            self.m_lowest = IntPoint::new(count - 1, k as cInt);
        } else {
            let ip = self.child(self.m_lowest.X as usize).m_polygon[self.m_lowest.Y as usize];
            let pk = self.m_polyNodes.nodes[newNode].m_polygon[k];
            if pk.Y > ip.Y || (pk.Y == ip.Y && pk.X < ip.X) {
                self.m_lowest = IntPoint::new(count - 1, k as cInt);
            }
        }
    }

    /// `AddPaths(Paths paths, JoinType joinType, EndType endType)`
    pub fn AddPaths(&mut self, paths: &Paths, joinType: JoinType, endType: EndType) {
        for p in paths {
            self.AddPath(p, joinType, endType);
        }
    }

    /// fixup orientations of all closed paths if the orientation of the closed path with the
    /// lowermost vertex is wrong ...
    fn FixOrientations(&mut self) {
        let childs = self.m_polyNodes.nodes[PolyTree::ROOT].m_Childs.clone();
        if self.m_lowest.X >= 0 && !Clipper::Orientation(&self.child(self.m_lowest.X as usize).m_polygon) {
            for c in childs {
                let node = &mut self.m_polyNodes.nodes[c];
                if node.m_endtype == EndType::etClosedPolygon || (node.m_endtype == EndType::etClosedLine && Clipper::Orientation(&node.m_polygon)) {
                    node.m_polygon.reverse();
                }
            }
        } else {
            for c in childs {
                let node = &mut self.m_polyNodes.nodes[c];
                if node.m_endtype == EndType::etClosedLine && !Clipper::Orientation(&node.m_polygon) {
                    node.m_polygon.reverse();
                }
            }
        }
    }

    /// `GetUnitNormal(IntPoint pt1, IntPoint pt2)`
    pub(crate) fn GetUnitNormal(pt1: IntPoint, pt2: IntPoint) -> DoublePoint {
        let mut dx = pt2.X.wrapping_sub(pt1.X) as f64;
        let mut dy = pt2.Y.wrapping_sub(pt1.Y) as f64;
        if (dx == 0.0) && (dy == 0.0) {
            return DoublePoint::default();
        }

        let f = 1.0 * 1.0 / (dx * dx + dy * dy).sqrt();
        dx *= f;
        dy *= f;

        DoublePoint::new(dy, -dx)
    }

    fn DoOffset(&mut self, delta: f64) {
        self.m_destPolys = Paths::new();
        self.m_delta = delta;
        let childs = self.m_polyNodes.nodes[PolyTree::ROOT].m_Childs.clone();

        //if Zero offset, just copy any CLOSED polygons to m_p and return ...
        if near_zero(delta) {
            for c in &childs {
                let node = &self.m_polyNodes.nodes[*c];
                if node.m_endtype == EndType::etClosedPolygon {
                    self.m_destPolys.push(node.m_polygon.clone());
                }
            }
            return;
        }

        //see offset_triginometry3.svg in the documentation folder ...
        if self.MiterLimit > 2.0 {
            self.m_miterLim = 2.0 / (self.MiterLimit * self.MiterLimit);
        } else {
            self.m_miterLim = 0.5;
        }

        let y = if self.ArcTolerance <= 0.0 {
            def_arc_tolerance
        } else if self.ArcTolerance > delta.abs() * def_arc_tolerance {
            delta.abs() * def_arc_tolerance
        } else {
            self.ArcTolerance
        };
        //see offset_triginometry2.svg in the documentation folder ...
        let steps = std::f64::consts::PI / (1.0 - y / delta.abs()).acos();
        self.m_sin = (two_pi / steps).sin();
        self.m_cos = (two_pi / steps).cos();
        self.m_StepsPerRad = steps / two_pi;
        if delta < 0.0 {
            self.m_sin = -self.m_sin;
        }

        for c in childs {
            let node = self.m_polyNodes.nodes[c].clone();
            self.m_srcPoly = node.m_polygon.clone();

            let len = self.m_srcPoly.len();

            if len == 0 || (delta <= 0.0 && (len < 3 || node.m_endtype != EndType::etClosedPolygon)) {
                continue;
            }

            self.m_destPoly = Path::new();

            if len == 1 {
                let s0 = self.m_srcPoly[0];
                if node.m_jointype == JoinType::jtRound {
                    let (mut X, mut Y) = (1.0, 0.0);
                    let mut j = 1.0;
                    while j <= steps {
                        self.m_destPoly.push(IntPoint::new(Self::Round(s0.X as f64 + X * delta), Self::Round(s0.Y as f64 + Y * delta)));
                        let X2 = X;
                        X = X * self.m_cos - self.m_sin * Y;
                        Y = X2 * self.m_sin + Y * self.m_cos;
                        j += 1.0;
                    }
                } else {
                    let (mut X, mut Y) = (-1.0, -1.0);
                    for _ in 0..4 {
                        self.m_destPoly.push(IntPoint::new(Self::Round(s0.X as f64 + X * delta), Self::Round(s0.Y as f64 + Y * delta)));
                        if X < 0.0 {
                            X = 1.0;
                        } else if Y < 0.0 {
                            Y = 1.0;
                        } else {
                            X = -1.0;
                        }
                    }
                }
                self.m_destPolys.push(std::mem::take(&mut self.m_destPoly));
                continue;
            }

            //build m_normals ...
            self.m_normals.clear();
            for j in 0..len - 1 {
                let n = Self::GetUnitNormal(self.m_srcPoly[j], self.m_srcPoly[j + 1]);
                self.m_normals.push(n);
            }
            if node.m_endtype == EndType::etClosedLine || node.m_endtype == EndType::etClosedPolygon {
                let n = Self::GetUnitNormal(self.m_srcPoly[len - 1], self.m_srcPoly[0]);
                self.m_normals.push(n);
            } else {
                let n = self.m_normals[len - 2];
                self.m_normals.push(n);
            }

            if node.m_endtype == EndType::etClosedPolygon {
                let mut k = len - 1;
                for j in 0..len {
                    self.OffsetPoint(j, &mut k, node.m_jointype);
                }
                self.m_destPolys.push(std::mem::take(&mut self.m_destPoly));
            } else if node.m_endtype == EndType::etClosedLine {
                let mut k = len - 1;
                for j in 0..len {
                    self.OffsetPoint(j, &mut k, node.m_jointype);
                }
                self.m_destPolys.push(std::mem::take(&mut self.m_destPoly));
                self.m_destPoly = Path::new();
                //re-build m_normals ...
                let n = self.m_normals[len - 1];
                for j in (1..len).rev() {
                    self.m_normals[j] = DoublePoint::new(-self.m_normals[j - 1].X, -self.m_normals[j - 1].Y);
                }
                self.m_normals[0] = DoublePoint::new(-n.X, -n.Y);
                k = 0;
                for j in (0..len).rev() {
                    self.OffsetPoint(j, &mut k, node.m_jointype);
                }
                self.m_destPolys.push(std::mem::take(&mut self.m_destPoly));
            } else {
                let mut k = 0usize;
                for j in 1..len - 1 {
                    self.OffsetPoint(j, &mut k, node.m_jointype);
                }

                if node.m_endtype == EndType::etOpenButt {
                    let j = len - 1;
                    let (s, n) = (self.m_srcPoly[j], self.m_normals[j]);
                    self.m_destPoly.push(IntPoint::new(Self::Round(s.X as f64 + n.X * delta), Self::Round(s.Y as f64 + n.Y * delta)));
                    self.m_destPoly.push(IntPoint::new(Self::Round(s.X as f64 - n.X * delta), Self::Round(s.Y as f64 - n.Y * delta)));
                } else {
                    let j = len - 1;
                    k = len - 2;
                    self.m_sinA = 0.0;
                    self.m_normals[j] = DoublePoint::new(-self.m_normals[j].X, -self.m_normals[j].Y);
                    if node.m_endtype == EndType::etOpenSquare {
                        self.DoSquare(j, k);
                    } else {
                        self.DoRound(j, k);
                    }
                }

                //re-build m_normals ...
                for j in (1..len).rev() {
                    self.m_normals[j] = DoublePoint::new(-self.m_normals[j - 1].X, -self.m_normals[j - 1].Y);
                }

                self.m_normals[0] = DoublePoint::new(-self.m_normals[1].X, -self.m_normals[1].Y);

                k = len - 1;
                let mut j = k as i64 - 1;
                while j > 0 {
                    self.OffsetPoint(j as usize, &mut k, node.m_jointype);
                    j -= 1;
                }

                if node.m_endtype == EndType::etOpenButt {
                    let (s, n) = (self.m_srcPoly[0], self.m_normals[0]);
                    self.m_destPoly.push(IntPoint::new(Self::Round(s.X as f64 - n.X * delta), Self::Round(s.Y as f64 - n.Y * delta)));
                    self.m_destPoly.push(IntPoint::new(Self::Round(s.X as f64 + n.X * delta), Self::Round(s.Y as f64 + n.Y * delta)));
                } else {
                    self.m_sinA = 0.0;
                    if node.m_endtype == EndType::etOpenSquare {
                        self.DoSquare(0, 1);
                    } else {
                        self.DoRound(0, 1);
                    }
                }
                self.m_destPolys.push(std::mem::take(&mut self.m_destPoly));
            }
        }
    }

    /// The outer rectangle a negative offset is cleaned against.
    fn outer_rect(&self) -> Path {
        let r = Clipper::GetBounds(&self.m_destPolys);
        vec![
            IntPoint::new(r.left.wrapping_sub(10), r.bottom.wrapping_add(10)),
            IntPoint::new(r.right.wrapping_add(10), r.bottom.wrapping_add(10)),
            IntPoint::new(r.right.wrapping_add(10), r.top.wrapping_sub(10)),
            IntPoint::new(r.left.wrapping_sub(10), r.top.wrapping_sub(10)),
        ]
    }

    /// `Execute(ref Paths solution, double delta)`
    pub fn Execute(&mut self, solution: &mut Paths, delta: f64) -> Result<(), Exception> {
        solution.clear();
        self.FixOrientations();
        self.DoOffset(delta);
        //now clean up 'corners' ...
        let mut clpr = Clipper::new(0);
        clpr.AddPaths(&self.m_destPolys, PolyType::ptSubject, true)?;
        if delta > 0.0 {
            clpr.Execute_paths(ClipType::ctUnion, solution, PolyFillType::pftPositive, PolyFillType::pftPositive)?;
        } else {
            let outer = self.outer_rect();
            clpr.AddPath(&outer, PolyType::ptSubject, true)?;
            clpr.ReverseSolution = true;
            clpr.Execute_paths(ClipType::ctUnion, solution, PolyFillType::pftNegative, PolyFillType::pftNegative)?;
            if !solution.is_empty() {
                solution.remove(0);
            }
        }
        Ok(())
    }

    /// `Execute(ref PolyTree solution, double delta)`
    pub fn Execute_tree(&mut self, solution: &mut PolyTree, delta: f64) -> Result<(), Exception> {
        solution.Clear();
        self.FixOrientations();
        self.DoOffset(delta);

        //now clean up 'corners' ...
        let mut clpr = Clipper::new(0);
        clpr.AddPaths(&self.m_destPolys, PolyType::ptSubject, true)?;
        if delta > 0.0 {
            clpr.Execute_tree_fill(ClipType::ctUnion, solution, PolyFillType::pftPositive, PolyFillType::pftPositive)?;
        } else {
            let outer = self.outer_rect();
            clpr.AddPath(&outer, PolyType::ptSubject, true)?;
            clpr.ReverseSolution = true;
            clpr.Execute_tree_fill(ClipType::ctUnion, solution, PolyFillType::pftNegative, PolyFillType::pftNegative)?;
            //remove the outer PolyNode rectangle ...
            let root = PolyTree::ROOT;
            if solution.ChildCount(root) == 1 && solution.ChildCount(solution.nodes[root].m_Childs[0]) > 0 {
                let outerNode = solution.nodes[root].m_Childs[0];
                let outerChilds = solution.nodes[outerNode].m_Childs.clone();
                solution.nodes[root].m_Childs[0] = outerChilds[0];
                solution.nodes[outerChilds[0]].m_Parent = Some(root);
                for &c in &outerChilds[1..] {
                    solution.AddChild(root, c);
                }
            } else {
                solution.Clear();
            }
        }
        Ok(())
    }

    fn OffsetPoint(&mut self, j: usize, k: &mut usize, jointype: JoinType) {
        let (nk, nj) = (self.m_normals[*k], self.m_normals[j]);
        let s = self.m_srcPoly[j];
        //cross product ...
        self.m_sinA = nk.X * nj.Y - nj.X * nk.Y;

        if (self.m_sinA * self.m_delta).abs() < 1.0 {
            //dot product ...
            let cosA = nk.X * nj.X + nj.Y * nk.Y;
            if cosA > 0.0 {
                // angle ==> 0 degrees
                self.m_destPoly.push(IntPoint::new(Self::Round(s.X as f64 + nk.X * self.m_delta), Self::Round(s.Y as f64 + nk.Y * self.m_delta)));
                return;
            }
            //else angle ==> 180 degrees
        } else if self.m_sinA > 1.0 {
            self.m_sinA = 1.0;
        } else if self.m_sinA < -1.0 {
            self.m_sinA = -1.0;
        }

        if self.m_sinA * self.m_delta < 0.0 {
            self.m_destPoly.push(IntPoint::new(Self::Round(s.X as f64 + nk.X * self.m_delta), Self::Round(s.Y as f64 + nk.Y * self.m_delta)));
            self.m_destPoly.push(s);
            self.m_destPoly.push(IntPoint::new(Self::Round(s.X as f64 + nj.X * self.m_delta), Self::Round(s.Y as f64 + nj.Y * self.m_delta)));
        } else {
            match jointype {
                JoinType::jtMiter => {
                    let r = 1.0 + (nj.X * nk.X + nj.Y * nk.Y);
                    if r >= self.m_miterLim {
                        self.DoMiter(j, *k, r);
                    } else {
                        self.DoSquare(j, *k);
                    }
                }
                JoinType::jtSquare => self.DoSquare(j, *k),
                JoinType::jtRound => self.DoRound(j, *k),
            }
        }
        *k = j;
    }

    fn DoSquare(&mut self, j: usize, k: usize) {
        let (nk, nj) = (self.m_normals[k], self.m_normals[j]);
        let s = self.m_srcPoly[j];
        let dx = (self.m_sinA.atan2(nk.X * nj.X + nk.Y * nj.Y) / 4.0).tan();
        self.m_destPoly.push(IntPoint::new(
            Self::Round(s.X as f64 + self.m_delta * (nk.X - nk.Y * dx)),
            Self::Round(s.Y as f64 + self.m_delta * (nk.Y + nk.X * dx)),
        ));
        self.m_destPoly.push(IntPoint::new(
            Self::Round(s.X as f64 + self.m_delta * (nj.X + nj.Y * dx)),
            Self::Round(s.Y as f64 + self.m_delta * (nj.Y - nj.X * dx)),
        ));
    }

    fn DoMiter(&mut self, j: usize, k: usize, r: f64) {
        let (nk, nj) = (self.m_normals[k], self.m_normals[j]);
        let s = self.m_srcPoly[j];
        let q = self.m_delta / r;
        self.m_destPoly.push(IntPoint::new(Self::Round(s.X as f64 + (nk.X + nj.X) * q), Self::Round(s.Y as f64 + (nk.Y + nj.Y) * q)));
    }

    fn DoRound(&mut self, j: usize, k: usize) {
        let (nk, nj) = (self.m_normals[k], self.m_normals[j]);
        let s = self.m_srcPoly[j];
        let a = self.m_sinA.atan2(nk.X * nj.X + nk.Y * nj.Y);
        let steps = (Self::Round(self.m_StepsPerRad * a.abs()) as i32).max(1);

        let (mut X, mut Y) = (nk.X, nk.Y);
        for _ in 0..steps {
            self.m_destPoly.push(IntPoint::new(Self::Round(s.X as f64 + X * self.m_delta), Self::Round(s.Y as f64 + Y * self.m_delta)));
            let X2 = X;
            X = X * self.m_cos - self.m_sin * Y;
            Y = X2 * self.m_sin + Y * self.m_cos;
        }
        self.m_destPoly.push(IntPoint::new(Self::Round(s.X as f64 + nj.X * self.m_delta), Self::Round(s.Y as f64 + nj.Y * self.m_delta)));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn square(x: i64, y: i64, s: i64) -> Path {
        vec![IntPoint::new(x, y), IntPoint::new(x + s, y), IntPoint::new(x + s, y + s), IntPoint::new(x, y + s)]
    }

    #[test]
    fn union_intersection_and_difference_of_squares() {
        let (a, b) = (square(0, 0, 100), square(50, 50, 100));
        let run = |ct: ClipType| {
            let mut c = Clipper::new(0);
            c.AddPath(&a, PolyType::ptSubject, true).unwrap();
            c.AddPath(&b, PolyType::ptClip, true).unwrap();
            let mut sol = Paths::new();
            assert!(c.Execute(ct, &mut sol, PolyFillType::pftNonZero).unwrap());
            sol
        };
        let i = run(ClipType::ctIntersection);
        assert_eq!(i.len(), 1);
        assert_eq!(Clipper::Area(&i[0]).abs(), 2500.0);
        let u = run(ClipType::ctUnion);
        assert_eq!(u.len(), 1);
        assert_eq!(Clipper::Area(&u[0]).abs(), 17500.0);
        let d = run(ClipType::ctDifference);
        assert_eq!(d.len(), 1);
        assert_eq!(Clipper::Area(&d[0]).abs(), 7500.0);
        let x = run(ClipType::ctXor);
        assert_eq!(x.iter().map(|p| Clipper::Area(p).abs()).sum::<f64>(), 15000.0);
    }

    #[test]
    fn negative_offset_shrinks_into_a_tree() {
        let mut co = ClipperOffset::new(2.0, 0.25);
        co.AddPath(&square(0, 0, 1000), JoinType::jtMiter, EndType::etClosedPolygon);
        let mut tree = PolyTree::new();
        co.Execute_tree(&mut tree, -100.0).unwrap();
        assert_eq!(tree.ChildCount(PolyTree::ROOT), 1);
        let c = tree.Childs(PolyTree::ROOT)[0];
        assert_eq!(Clipper::Area(tree.Contour(c)).abs(), 800.0 * 800.0);
        assert!(!tree.IsHole(c));
        // shrunk away entirely
        let mut tree = PolyTree::new();
        co.Execute_tree(&mut tree, -600.0).unwrap();
        assert_eq!(tree.ChildCount(PolyTree::ROOT), 0);
        // a positive square offset grows it
        let mut sol = Paths::new();
        co.Execute(&mut sol, 100.0).unwrap();
        assert_eq!(sol.len(), 1);
        assert_eq!(Clipper::Area(&sol[0]).abs(), 1200.0 * 1200.0);
    }

    #[test]
    fn point_in_polygon_and_clean() {
        let sq = square(0, 0, 10);
        assert_eq!(Clipper::PointInPolygon(IntPoint::new(5, 5), &sq), 1);
        assert_eq!(Clipper::PointInPolygon(IntPoint::new(0, 5), &sq), -1);
        assert_eq!(Clipper::PointInPolygon(IntPoint::new(15, 5), &sq), 0);
        let noisy = vec![IntPoint::new(0, 0), IntPoint::new(5, 0), IntPoint::new(10, 0), IntPoint::new(10, 10), IntPoint::new(0, 10)];
        assert_eq!(Clipper::CleanPolygon(&noisy, 1.415).len(), 4);
        assert!(Clipper::Orientation(&sq));
        // open paths need a PolyTree
        let mut c = Clipper::new(0);
        c.AddPath(&vec![IntPoint::new(0, 0), IntPoint::new(10, 10)], PolyType::ptSubject, false).unwrap();
        assert!(c.Execute(ClipType::ctUnion, &mut Paths::new(), PolyFillType::pftEvenOdd).is_err());
        assert!(c.AddPath(&vec![IntPoint::new(0, 0), IntPoint::new(10, 10)], PolyType::ptClip, false).is_err());
    }
}
