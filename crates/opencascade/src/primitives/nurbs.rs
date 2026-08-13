//! Reading a face's B-spline definition back out of OCCT.
//!
//! Everything else in `primitives/` either builds geometry or meshes it. This is the
//! other direction, and it exists so that something outside OCCT — a compute shader,
//! say — can evaluate a surface OCCT owns.
//!
//! # The order matters and is enforced by types
//!
//! [`Face::to_nurbs`] converts the *face*, not the surface. The tempting shortcut is
//! `BRep_Tool::Surface` followed by `GeomConvert::SurfaceToBSplineSurface`; it is wrong
//! in a way that does not show up until trimming, because the conversion may
//! reparametrize while `UVBounds` and the p-curves still speak the original surface's
//! parameters. `BRepBuilderAPI_NurbsConvert` rebuilds surface and p-curves together, so
//! the surface, its domain and its trim loops are three answers to the same question.
//!
//! That is why [`NurbsFace`] holds the converted `Face` rather than only the numbers:
//! whatever reads the trim loops later must read them off *this* face.
//!
//! # What is not checked here
//!
//! Structural validation — pole counts against the knot identity, multiplicities
//! against the degree, weights being positive — belongs to the consumer, which has an
//! error type for it. This module reports what OCCT said and whether the call
//! succeeded, and nothing else. The one thing it does own is the **index arithmetic**:
//! OCCT's arrays are 1-based and its pole grid is addressed `(i, j)`, and getting that
//! wrong produces a transposed surface that is smooth, plausible and wrong.

use glam::DVec3;
use opencascade_sys::ffi;

use crate::primitives::{make_point, Face};

/// Why a face could not be read as a B-spline.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum NurbsError {
    /// `BRepBuilderAPI_NurbsConvert` declined the face, or returned something that was
    /// not a face. Not a bug: some geometry OCCT will mesh, it will not convert.
    ConversionFailed,
    /// The converted face's surface was not a `Geom_BSplineSurface` after all. Means
    /// the conversion silently did nothing, which is worth distinguishing from it
    /// failing outright.
    NotBSpline,
    /// An array came back empty from a shim that catches OCCT exceptions — so this is
    /// what an OCCT throw looks like from Rust, rather than the process aborting.
    ReadFailed(&'static str),
    /// The pole array's length disagreed with `NbUPoles * NbVPoles`. Cannot happen
    /// unless the shim and this file disagree about the packing, which is exactly the
    /// failure worth naming rather than indexing past.
    PoleCountMismatch { expected: usize, found: usize },
}

impl core::fmt::Display for NurbsError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::ConversionFailed => write!(f, "BRepBuilderAPI_NurbsConvert declined the face"),
            Self::NotBSpline => write!(f, "converted face is still not a B-spline surface"),
            Self::ReadFailed(what) => write!(f, "reading {what} failed inside OCCT"),
            Self::PoleCountMismatch { expected, found } => {
                write!(f, "expected {expected} poles, got {found}")
            }
        }
    }
}

impl std::error::Error for NurbsError {}

/// One parametric direction of a B-spline surface, as OCCT stores it.
///
/// Knots are **distinct** values paired with multiplicities, not the flat sequence.
/// Expanding the pair is the consumer's job, and it is the most dangerous step in this
/// whole readback: an off-by-one multiplicity yields a surface that is subtly wrong
/// near every interior knot and looks correct everywhere else.
#[derive(Clone, Debug, PartialEq)]
pub struct NurbsAxis {
    pub degree: usize,
    pub knots: Vec<f64>,
    pub mults: Vec<usize>,
    /// A periodic direction's knot vector is not clamped, and its pole count follows a
    /// different identity from the open one. Getting this wrong yields a surface
    /// rotated by exactly one pole span — closed, seamless, the right radius, and
    /// wrong.
    pub periodic: bool,
    pub pole_count: usize,
}

/// A face's surface, read back as a rational B-spline, together with the converted
/// face it came from.
pub struct NurbsFace {
    /// The face after `BRepBuilderAPI_NurbsConvert`. **Trim loops must be read off
    /// this**, not off the original — see the module note.
    pub face: Face,
    pub u: NurbsAxis,
    pub v: NurbsAxis,
    /// `u.pole_count * v.pole_count` poles, **u-major**: `pole(i, j)` is at
    /// `i * v.pole_count + j`.
    pub poles: Vec<DVec3>,
    /// `None` on a polynomial surface.
    ///
    /// Deliberately not a vector of ones. OCCT answers `Weight(i,j)` with 1.0 on a
    /// polynomial surface, so the value is always readable and only `IsURational` /
    /// `IsVRational` say whether it means anything. Storing `None` keeps a placeholder
    /// from being mistaken for a measurement downstream.
    pub weights: Option<Vec<f64>>,
    /// `(u_min, u_max, v_min, v_max)` — the **face's** domain, not the surface's. A
    /// plane's own bounds are infinite; this is the sampling rectangle.
    pub bounds: (f64, f64, f64, f64),
}

impl NurbsFace {
    pub fn pole(&self, i: usize, j: usize) -> DVec3 {
        self.poles[i * self.v.pole_count + j]
    }

    pub fn weight(&self, i: usize, j: usize) -> f64 {
        match &self.weights {
            Some(w) => w[i * self.v.pole_count + j],
            None => 1.0,
        }
    }

    pub fn is_rational(&self) -> bool {
        self.weights.is_some()
    }

    /// OCCT's own `S(u,v)` on the converted face.
    ///
    /// **An oracle, not a rendering path.** Nothing above is self-checking: nine
    /// separate quantities crossed the FFI, each with its own index convention, and a
    /// transposed pole grid or a misread multiplicity produces a plausible wrong
    /// surface. This lets a test compare an independent evaluator against the kernel
    /// that owns the definition, at `f64`.
    ///
    /// Deliberately the surface adaptor and not the triangulation: a mesh is
    /// deflection-limited, so disagreements against it are dominated by the mesher's
    /// tolerance rather than by anyone's arithmetic.
    ///
    /// `None` when OCCT threw — an out-of-domain parameter is a real way to reach that,
    /// so a test that samples outside [`NurbsFace::bounds`] gets an answer rather than
    /// a crash.
    pub fn occt_value(&self, u: f64, v: f64) -> Option<DVec3> {
        self.face.occt_value(u, v)
    }

    /// OCCT's `S`, `dS/du`, `dS/dv` in one evaluation. See [`Self::occt_value`].
    pub fn occt_d1(&self, u: f64, v: f64) -> Option<(DVec3, DVec3, DVec3)> {
        self.face.occt_d1(u, v)
    }

    /// How far `point` lies from the converted surface, by OCCT's own projection.
    ///
    /// **The conversion oracle, and deliberately not an equality.**
    /// [`Self::occt_value`] checks the *readback* — that our numbers describe the
    /// surface OCCT converted to. It says nothing about whether that surface is the one
    /// we started with, and the obvious way to check that is wrong: `NurbsConvert` may
    /// reparametrize, so `S_orig(u, v)` and `S_conv(u, v)` can both be correct and
    /// differ, and an equality at a shared parameter fails on a conversion that did its
    /// job.
    ///
    /// Sampling the *original* face and asking this how far each point is from the
    /// converted surface is parametrization-independent, because it never assumes the
    /// two agree about `(u, v)` — only that the point still lies on the surface. The
    /// tolerance is a conversion tolerance; state it rather than tune it.
    ///
    /// `None` when the projection found nothing.
    pub fn distance_to(&self, point: DVec3) -> Option<f64> {
        let surface = ffi::BRep_Tool_Surface(&self.face.inner);
        let p = make_point(point);
        let out = ffi::project_point_on_surface(&surface, &p);
        (out.len() == 3).then(|| out[2])
    }
}

impl Face {
    /// OCCT's own `S(u,v)` on **this** face's surface, whatever type it happens to be.
    ///
    /// On `Face` rather than only on [`NurbsFace`] because the conversion test needs to
    /// evaluate the *original* face — the one that is still a `Cylinder` or a `Plane` —
    /// and `Face::inner` is crate-private, so nothing outside this crate could reach it.
    /// [`NurbsFace::occt_value`] is this method on the converted face.
    ///
    /// `None` when OCCT threw. An out-of-domain parameter is a real way to reach that,
    /// so a caller that samples outside [`Self::uv_bounds`] gets an answer rather than a
    /// process abort — OCCT exceptions crossing the FFI uncaught are `SIGABRT`, not
    /// `Err`.
    pub fn occt_value(&self, u: f64, v: f64) -> Option<DVec3> {
        let adaptor = ffi::BRepAdaptor_Surface_ctor(&self.inner);
        let p = ffi::BRepAdaptor_Surface_value(&adaptor, u, v);
        if p.is_null() {
            return None;
        }
        Some(DVec3::new(p.X(), p.Y(), p.Z()))
    }

    /// OCCT's `S`, `dS/du`, `dS/dv` in one evaluation. See [`Self::occt_value`].
    pub fn occt_d1(&self, u: f64, v: f64) -> Option<(DVec3, DVec3, DVec3)> {
        let adaptor = ffi::BRepAdaptor_Surface_ctor(&self.inner);
        let d = ffi::BRepAdaptor_Surface_d1(&adaptor, u, v);
        if d.len() != 9 {
            return None;
        }
        Some((
            DVec3::new(d[0], d[1], d[2]),
            DVec3::new(d[3], d[4], d[5]),
            DVec3::new(d[6], d[7], d[8]),
        ))
    }

    /// `BRepTools::UVBounds` — `(u_min, u_max, v_min, v_max)`, the face's own parametric
    /// rectangle.
    ///
    /// Not the surface's range. A plane's surface is infinite in both directions; this
    /// is the part of it the face occupies, and it is the only sampling rectangle that
    /// means anything.
    pub fn uv_bounds(&self) -> Option<(f64, f64, f64, f64)> {
        let b = ffi::face_uv_bounds(&self.inner);
        if b.len() != 4 {
            return None;
        }
        Some((b[0], b[1], b[2], b[3]))
    }

    /// Convert this face to B-spline geometry and read the surface back.
    ///
    /// Returns the converted face alongside the numbers — see [`NurbsFace::face`] for
    /// why that pairing is not optional.
    pub fn to_nurbs(&self) -> Result<NurbsFace, NurbsError> {
        let converted = ffi::nurbs_convert_face(&self.inner);
        if converted.is_null() {
            return Err(NurbsError::ConversionFailed);
        }
        let face = Face { inner: converted };

        let surface = ffi::bspline_surface_of_face(&face.inner);
        if surface.is_null() || ffi::HandleGeomBSplineSurface_IsNull(&surface) {
            return Err(NurbsError::NotBSpline);
        }

        let n_u = ffi::bspline_nb_u_poles(&surface).max(0) as usize;
        let n_v = ffi::bspline_nb_v_poles(&surface).max(0) as usize;

        let u = read_axis(
            ffi::bspline_u_degree(&surface),
            ffi::bspline_u_knots(&surface),
            ffi::bspline_u_mults(&surface),
            ffi::bspline_is_u_periodic(&surface),
            n_u,
            "u knots",
        )?;
        let v = read_axis(
            ffi::bspline_v_degree(&surface),
            ffi::bspline_v_knots(&surface),
            ffi::bspline_v_mults(&surface),
            ffi::bspline_is_v_periodic(&surface),
            n_v,
            "v knots",
        )?;

        // A surface is rational if *either* direction is. There is one weight per pole,
        // not one per direction, so the two flags are a disjunction rather than a pair.
        let rational =
            ffi::bspline_is_u_rational(&surface) || ffi::bspline_is_v_rational(&surface);

        let flat = ffi::bspline_poles(&surface);
        if flat.is_empty() && n_u * n_v != 0 {
            return Err(NurbsError::ReadFailed("poles"));
        }
        if flat.len() != n_u * n_v * 4 {
            return Err(NurbsError::PoleCountMismatch {
                expected: n_u * n_v,
                found: flat.len() / 4,
            });
        }

        let poles: Vec<DVec3> =
            flat.chunks_exact(4).map(|p| DVec3::new(p[0], p[1], p[2])).collect();
        let weights =
            rational.then(|| flat.chunks_exact(4).map(|p| p[3]).collect::<Vec<f64>>());

        let b = ffi::face_uv_bounds(&face.inner);
        if b.len() != 4 {
            return Err(NurbsError::ReadFailed("UV bounds"));
        }

        Ok(NurbsFace { face, u, v, poles, weights, bounds: (b[0], b[1], b[2], b[3]) })
    }
}

fn read_axis(
    degree: i32,
    knots: Vec<f64>,
    mults: Vec<i32>,
    periodic: bool,
    pole_count: usize,
    what: &'static str,
) -> Result<NurbsAxis, NurbsError> {
    if knots.is_empty() || mults.len() != knots.len() {
        return Err(NurbsError::ReadFailed(what));
    }
    Ok(NurbsAxis {
        degree: degree.max(0) as usize,
        knots,
        mults: mults.into_iter().map(|m| m.max(0) as usize).collect(),
        periodic,
        pole_count,
    })
}
