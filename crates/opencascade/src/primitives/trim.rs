//! A face's boundary, sampled into **parameter space**.
//!
//! The other half of reading a face back out. [`super::nurbs`] answers what surface the
//! face lies on; this answers which part of that surface the face actually is. Sampling
//! the surface's UV rectangle without this renders holes filled in, extrusions running
//! past their ends, and — the case that is a correctness bug rather than a cosmetic one —
//! every cylinder in the model slit open along its seam.
//!
//! # Why this cannot be assembled from `Face::edges()`
//!
//! `Face::edges()` walks a `TopExp_Explorer`, which visits the edges a face contains in
//! whatever order its `TShape` stores them. A boundary loop needs them in **connection**
//! order, and the two coincide only by luck. A scrambled polygon is not detectably
//! scrambled — it is a closed polygon with the same vertices, so it validates, it has a
//! signed area, and the winding-number test that consumes it returns a confident wrong
//! answer. `BRepTools_WireExplorer` walks the chain vertex to vertex instead, and hands
//! back each edge oriented the way the wire traverses it.
//!
//! # Degenerate edges are sampled, not skipped
//!
//! This is the one place where the obvious reading of the requirement list is backwards,
//! so it is worth stating plainly. A sphere's pole is a **degenerate edge**: it has a
//! p-curve of finite length in parameter space that maps to a single point in ℝ³. In 3D
//! it bounds nothing, and requirement #5 is about exactly that — the normal there is not
//! defined by the cross product and must come from elsewhere.
//!
//! In parameter space it is the segment that **closes the loop**. Skip it and the sphere's
//! trim loop is an open chain whose two ends are half a period apart, which the winding
//! test reads as a region covering roughly half the surface. So `BRep_Tool::Degenerated`
//! is reported here for the caller's benefit and is deliberately not a filter.

use crate::primitives::Face;
use glam::{dvec2, DVec2};
use opencascade_sys::ffi;

/// One closed boundary loop of a face, in the face's own `(u, v)` parameters.
///
/// The loop is **implicitly closed**: the last point connects back to the first, and no
/// point is repeated at the join. Each edge contributes its start plus its interior
/// samples and drops its endpoint, because the next edge's start *is* that endpoint.
#[derive(Clone, Debug, PartialEq)]
pub struct TrimLoop {
    /// `BRepTools::OuterWire` said this is the one that bounds the face; the rest are
    /// holes through it.
    pub outer: bool,
    pub points: Vec<DVec2>,
    /// How many edges the explorer walked. Zero with a non-empty `points` is impossible;
    /// zero with empty `points` means the wire could not be walked at all.
    pub edges: usize,
    /// Of those, how many OCCT marks degenerate. **Sampled anyway** — see the module
    /// note. Reported because a sphere with none is a readback that lost its poles.
    pub degenerate_edges: usize,
    /// Edges carrying no p-curve on this face. **Any nonzero value means this loop has a
    /// gap in it** and its winding number is not trustworthy — the polyline jumps
    /// straight from one side of the missing edge to the other. Nothing here can repair
    /// that, so it is reported rather than papered over.
    pub missing_pcurves: usize,
    /// Individual samples OCCT declined on a p-curve it *did* hand over. A separate
    /// count from the one above because it is a different fault with the same symptom:
    /// there the edge was never there, here the curve was and could not be evaluated at
    /// a parameter inside its own stated range. Either way the loop has a gap.
    pub failed_samples: usize,
}

impl TrimLoop {
    /// Signed area × 2 by the shoelace formula; positive is counter-clockwise in
    /// parameter space. The sign is what distinguishes an outer wire's sense from a
    /// hole's under OCCT's convention, and the magnitude is what a consumer can sanity
    /// check a hole against — a hole larger than its outer wire is a mislabelling.
    pub fn signed_area_2(&self) -> f64 {
        let n = self.points.len();
        if n < 3 {
            return 0.0;
        }
        (0..n)
            .map(|i| {
                let a = self.points[i];
                let b = self.points[(i + 1) % n];
                a.x * b.y - b.x * a.y
            })
            .sum()
    }

    /// Whether this loop has everything it needs to be believed: at least a triangle's
    /// worth of points, and no edge that failed to produce a p-curve.
    pub fn is_usable(&self) -> bool {
        self.points.len() >= 3 && self.missing_pcurves == 0 && self.failed_samples == 0
    }
}

impl Face {
    /// Every boundary loop of this face, sampled into parameter space.
    ///
    /// The outer wire comes first when OCCT could identify one. Order among the holes is
    /// the explorer's and carries no meaning.
    ///
    /// # `samples_per_edge`
    ///
    /// Points contributed per edge, endpoint excluded. **1 means corners only**, and that
    /// is exact — not an approximation — for any edge whose p-curve is a straight line in
    /// parameter space, which covers every face of a box and all four sides of a
    /// cylinder's lateral face. Above 1 the edge is sampled uniformly in its own
    /// parameter.
    ///
    /// Uniform rather than adaptive on purpose: the consumer of this is a
    /// rasterised trim mask whose boundary is already jagged at cell resolution, so
    /// chord-error subdivision would buy a precision the mask immediately discards. A
    /// caller that needs better should raise this number, which is the knob that actually
    /// corresponds to what it gets.
    ///
    /// Clamped to at least 1: zero samples per edge is an empty loop, which is
    /// indistinguishable from an untrimmed face downstream and is the worst available
    /// silent failure.
    pub fn trim_loops(&self, samples_per_edge: usize) -> Vec<TrimLoop> {
        let n = samples_per_edge.max(1);

        // Null is a real answer — a face with no wires — and not a reason to stop. Such a
        // face has no loops at all, which the empty return below says correctly.
        let outer_wire = ffi::face_outer_wire(&self.inner);

        let mut loops = Vec::new();
        let mut explorer = ffi::TopExp_Explorer_ctor(
            ffi::cast_face_to_shape(&self.inner),
            ffi::TopAbs_ShapeEnum::TopAbs_WIRE,
        );

        while explorer.More() {
            let wire_shape = ffi::ExplorerCurrentShape(&explorer);
            let wire = ffi::TopoDS_cast_to_wire(&wire_shape);

            // `IsSame`, not `IsEqual`: the same wire reached two ways can carry different
            // accumulated orientations, and an outer wire that compares unequal to itself
            // is labelled a hole — which renders the face as its own negative.
            let outer = !outer_wire.is_null()
                && wire_shape.IsSame(ffi::cast_wire_to_shape(&outer_wire));

            loops.push(self.walk_wire(wire, outer, n));
            explorer.pin_mut().Next();
        }

        // Outer first. A stable position is worth more to a consumer than the explorer's
        // order, and there is at most one.
        loops.sort_by_key(|l| !l.outer);
        loops
    }

    /// One wire, in connection order, sampled onto this face.
    fn walk_wire(&self, wire: &ffi::TopoDS_Wire, outer: bool, samples_per_edge: usize) -> TrimLoop {
        let mut out = TrimLoop {
            outer,
            points: Vec::new(),
            edges: 0,
            degenerate_edges: 0,
            missing_pcurves: 0,
            failed_samples: 0,
        };

        let mut explorer = ffi::BRepTools_WireExplorer_ctor(wire, &self.inner);
        if explorer.is_null() {
            return out;
        }

        while explorer.More() {
            let edge = ffi::WireExplorerCurrentEdge(&explorer);
            out.edges += 1;

            if ffi::BRep_Tool_Degenerated(&edge) {
                out.degenerate_edges += 1;
            }

            // The edge/face pair, not the edge alone. A seam edge belongs to this face
            // twice with a different p-curve each time, and OCCT picks between them by the
            // edge's orientation — which is why the oriented edge from the wire explorer
            // is what has to be passed here. Pass the unoriented edge and both traversals
            // return the same curve, and the loop doubles back along one side of the seam
            // instead of going around.
            let curve = ffi::BRep_Tool_CurveOnSurface(&edge, &self.inner);
            let range = ffi::BRep_Tool_CurveOnSurface_range(&edge, &self.inner);
            if curve.is_null() || range.len() != 2 {
                out.missing_pcurves += 1;
                explorer.pin_mut().Next();
                continue;
            }

            let (first, last) = (range[0], range[1]);
            let reversed = ffi::cast_edge_to_shape(&edge).Orientation()
                == ffi::TopAbs_Orientation::TopAbs_REVERSED;

            for k in 0..samples_per_edge {
                // `k / samples_per_edge`, so the endpoint at 1.0 is never produced: the
                // next edge's first sample is that point. Using `k / (n - 1)` here would
                // duplicate every corner, putting a zero-length segment at each one.
                let s = k as f64 / samples_per_edge as f64;
                let t = if reversed {
                    last - (last - first) * s
                } else {
                    first + (last - first) * s
                };

                let p = ffi::Geom2d_Curve_value(&curve, t);
                if p.is_null() {
                    out.failed_samples += 1;
                } else {
                    out.points.push(dvec2(p.X(), p.Y()));
                }
            }

            explorer.pin_mut().Next();
        }

        out
    }
}
