use crate::{
    primitives::{FaceOrientation, Shape},
    Error,
};
use cxx::UniquePtr;
use glam::{dvec2, dvec3, DVec2, DVec3};
use opencascade_sys::ffi;

#[derive(Debug)]
pub struct Mesh {
    pub vertices: Vec<DVec3>,
    pub uvs: Vec<DVec2>,
    pub normals: Vec<DVec3>,
    pub indices: Vec<usize>,
}

pub struct Mesher {
    pub(crate) inner: UniquePtr<ffi::BRepMesh_IncrementalMesh>,
}

/// Which 2D Delaunay triangulation the mesher builds each face with.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum MeshAlgo {
    /// Whatever OCCT's global default is (currently Watson).
    #[default]
    Default,
    Watson,
    Delabella,
}

impl MeshAlgo {
    const fn as_i32(self) -> i32 {
        match self {
            Self::Default => -1,
            Self::Watson => 0,
            Self::Delabella => 1,
        }
    }
}

/// The mesher parameters beyond linear and angular deflection.
///
/// [`Mesher::try_new_with_angle`] reaches four of `IMeshTools_Parameters`' fourteen
/// fields. The ones here are the rest of what a caller can usefully steer; the
/// remainder are either fixed by this crate's threading model (`InParallel`,
/// `Relative` — see [`Mesher::try_new_with_angle`]) or overwritten by the kernel
/// (`CleanModel`).
///
/// `None` means "leave it to OCCT", which is not the same as zero — each of those
/// fields has a negative sentinel that `initParameters()` resolves, and the resolved
/// value is documented per field below.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MeshParams {
    /// Floor on triangle edge length, in world units. `None` resolves to
    /// `0.1 * min(deflection, deflection_interior)`.
    ///
    /// This is the kernel's guard against a distorted curve or surface amplifying
    /// into an unbounded triangle count, and it is the only bound on mesh density
    /// that is per-face and geometric rather than scaled from the whole shape.
    pub min_size: Option<f64>,

    /// Adjust `min_size` locally by each edge's own length. Off in OCCT.
    pub adjust_min_size: bool,

    /// Angular deflection across the face **interior**, in radians. `None` resolves
    /// to **twice** the boundary angle passed to the constructor.
    ///
    /// That doubling is OCCT's default and it is easy to miss: a caller asking for
    /// 0.5 rad gets 0.5 along the boundary edges and 1.0 across the interior, which
    /// is a fine silhouette over a coarse middle. Set this equal to the boundary
    /// angle to mesh both the same.
    pub angle_interior: Option<f64>,

    /// Linear deflection across the face interior. `None` resolves to the boundary
    /// deflection, so unlike `angle_interior` the default here is already uniform.
    pub deflection_interior: Option<f64>,

    /// Check the triangulation's deviation from the face interior and refine where it
    /// exceeds the deflection. On in OCCT; turning it off trades interior accuracy
    /// for the cost of that pass.
    pub control_surface_deflection: bool,

    /// Whether a request for a *coarser* mesh than the one already stored is honoured.
    ///
    /// Off in OCCT, which means it is not: the reuse test
    /// (`BRepMesh_Deflection::IsConsistent`) treats any stored triangulation finer
    /// than the request as satisfying it. Turning this on makes the test a band in
    /// both directions. It does **not** make an angular change observable — that
    /// comparison reads linear deflection alone and `Poly_Triangulation` stores no
    /// angle — so dropping a stale triangulation is still
    /// [`Shape::clean_triangulation`](crate::primitives::Shape::clean_triangulation)'s
    /// job.
    pub allow_quality_decrease: bool,

    pub mesh_algo: MeshAlgo,
}

impl Default for MeshParams {
    /// Every field as OCCT itself defaults it, so a `Mesher` built with these meshes
    /// exactly as [`Mesher::try_new_with_angle`] does.
    fn default() -> Self {
        Self {
            min_size: None,
            adjust_min_size: false,
            angle_interior: None,
            deflection_interior: None,
            control_surface_deflection: true,
            allow_quality_decrease: false,
            mesh_algo: MeshAlgo::Default,
        }
    }
}

/// The `IMeshData_Status` bits OCCT accumulated across every face and wire of a run.
///
/// Obtained from [`Mesher::status_flags`]. Zero is success; the interesting bits are
/// [`MeshStatus::failure`], which says the mesher gave up on some faces, and
/// [`MeshStatus::reused`], which says it kept an existing triangulation instead of
/// building one.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct MeshStatus(pub i32);

impl MeshStatus {
    const BITS: [(i32, &'static str); 9] = [
        (0x1, "OpenWire"),
        (0x2, "SelfIntersectingWire"),
        (0x4, "Failure"),
        (0x8, "ReMesh"),
        (0x10, "UnorientedWire"),
        (0x20, "TooFewPoints"),
        (0x40, "Outdated"),
        (0x80, "Reused"),
        (0x100, "UserBreak"),
    ];

    /// No bit set: every face meshed, nothing was reused, no wire was suspect.
    #[must_use]
    pub const fn is_clean(self) -> bool {
        self.0 == 0
    }

    /// The mesher failed to generate a mesh for some faces.
    ///
    /// [`Mesher::mesh`] discards the **whole shape** when it meets an untriangulated
    /// face, so this is the flag that says why a shape came back empty — and, unlike
    /// that error, it is also set when extraction happens to succeed anyway.
    #[must_use]
    pub const fn failure(self) -> bool {
        self.0 & 0x4 != 0
    }

    /// An existing triangulation on some faces was reused rather than rebuilt.
    ///
    /// After a `clean_triangulation` there is nothing left to reuse, so this
    /// appearing on a supposedly cold mesh means the clean did not reach the shape
    /// being meshed.
    #[must_use]
    pub const fn reused(self) -> bool {
        self.0 & 0x80 != 0
    }

    /// Some faces carried a triangulation coarser than requested.
    #[must_use]
    pub const fn outdated(self) -> bool {
        self.0 & 0x40 != 0
    }
}

impl std::fmt::Display for MeshStatus {
    /// `NoError`, or the set bits joined by `|` — e.g. `Reused|Outdated`. Unknown bits
    /// are reported in hex rather than dropped.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if self.0 == 0 {
            return f.write_str("NoError");
        }

        let mut first = true;
        let mut seen = 0;
        for (bit, name) in Self::BITS {
            if self.0 & bit != 0 {
                if !first {
                    f.write_str("|")?;
                }
                f.write_str(name)?;
                first = false;
                seen |= bit;
            }
        }

        let unknown = self.0 & !seen;
        if unknown != 0 {
            if !first {
                f.write_str("|")?;
            }
            write!(f, "0x{unknown:x}")?;
        }
        Ok(())
    }
}

impl Mesher {
    pub fn try_new(shape: &Shape, triangulation_tolerance: f64) -> Result<Self, Error> {
        let inner = ffi::BRepMesh_IncrementalMesh_ctor(&shape.inner, triangulation_tolerance);

        if inner.IsDone() {
            Ok(Self { inner })
        } else {
            Err(Error::TriangulationFailed)
        }
    }

    /// Mesh with an explicit angular deflection as well as a linear one.
    ///
    /// `try_new` leaves the angle at OCCT's default 0.5 rad (~28.6 degrees), which
    /// on a curved surface is usually the binding constraint — so tightening only
    /// `triangulation_tolerance` there buys less than it appears to. Both limits
    /// apply at once and the mesher honours whichever is stricter.
    ///
    /// `angular_deflection` is in **radians**, and it is the maximum angle between
    /// the normals at adjacent nodes. Values are clamped into OCCT's own accepted
    /// band: it rejects a non-positive angle outright, and anything above ~1.57 rad
    /// stops constraining anything at all.
    ///
    /// The two flags are fixed rather than exposed, because only one setting of
    /// each is correct for this crate's callers:
    ///
    /// - `isRelative = false` — `triangulation_tolerance` is an absolute world-unit
    ///   chord height. Turning this on would reinterpret it as a fraction of each
    ///   edge's own length, silently changing what every existing caller's number
    ///   means.
    /// - `isInParallel = false` — this flag splits *one shape's faces* across
    ///   threads. Callers meshing many shapes should parallelise at the shape level
    ///   with this off; nesting the two is the configuration OCCT's own forum
    ///   reports as sporadically hanging, and a swept tube has too few faces for it
    ///   to pay anyway.
    pub fn try_new_with_angle(
        shape: &Shape,
        triangulation_tolerance: f64,
        angular_deflection: f64,
    ) -> Result<Self, Error> {
        let angle = angular_deflection.clamp(Self::MIN_ANGLE, std::f64::consts::FRAC_PI_2);

        let inner = ffi::BRepMesh_IncrementalMesh_ctor_full(
            &shape.inner,
            triangulation_tolerance,
            false,
            angle,
            false,
        );

        if inner.IsDone() {
            Ok(Self { inner })
        } else {
            Err(Error::TriangulationFailed)
        }
    }

    /// Smallest angular deflection OCCT will accept. Below `Precision::Angular()` the
    /// kernel treats the value as unset rather than as a very tight request.
    const MIN_ANGLE: f64 = 1.0e-3;

    /// Smallest linear deflection OCCT will accept: `Precision::Confusion()`.
    ///
    /// `initParameters()` **throws** `Standard_NumericError` below this, and that
    /// exception would unwind out of a constructor called across the FFI boundary.
    /// Clamping here is what keeps it unreachable.
    const MIN_DEFLECTION: f64 = 1.0e-7;

    /// Mesh with the mesher's full parameter set.
    ///
    /// [`try_new_with_angle`](Self::try_new_with_angle) reaches four of
    /// `IMeshTools_Parameters`' fourteen fields and leaves the rest at defaults that
    /// are not all inert — see [`MeshParams`], particularly
    /// [`angle_interior`](MeshParams::angle_interior), which OCCT resolves to *twice*
    /// the angle passed here.
    ///
    /// `triangulation_tolerance` and `angular_deflection` mean exactly what they mean
    /// on `try_new_with_angle`, and `MeshParams::default()` reproduces its behaviour
    /// field for field, so this is a superset rather than a second regime.
    pub fn try_new_with_params(
        shape: &Shape,
        triangulation_tolerance: f64,
        angular_deflection: f64,
        params: &MeshParams,
    ) -> Result<Self, Error> {
        let angle = angular_deflection.clamp(Self::MIN_ANGLE, std::f64::consts::FRAC_PI_2);
        let deflection = triangulation_tolerance.max(Self::MIN_DEFLECTION);

        let mut p = ffi::IMeshTools_Parameters_ctor();
        ffi::IMeshTools_Parameters_set_deflection(p.pin_mut(), deflection);
        ffi::IMeshTools_Parameters_set_angle(p.pin_mut(), angle);

        // `None` is passed through as OCCT's negative sentinel rather than as a
        // number, so the kernel applies its own rule and this crate does not have to
        // restate it. A `Some` is clamped into the same band as the boundary value
        // above: too small does not mean "very fine", it means "unset", and would
        // silently resolve to the default the caller was trying to override.
        ffi::IMeshTools_Parameters_set_deflection_interior(
            p.pin_mut(),
            params.deflection_interior.map_or(-1.0, |d| d.max(Self::MIN_DEFLECTION)),
        );
        ffi::IMeshTools_Parameters_set_angle_interior(
            p.pin_mut(),
            params
                .angle_interior
                .map_or(-1.0, |a| a.clamp(Self::MIN_ANGLE, std::f64::consts::FRAC_PI_2)),
        );
        ffi::IMeshTools_Parameters_set_min_size(
            p.pin_mut(),
            params.min_size.map_or(-1.0, |m| m.max(Self::MIN_DEFLECTION)),
        );

        ffi::IMeshTools_Parameters_set_adjust_min_size(p.pin_mut(), params.adjust_min_size);
        ffi::IMeshTools_Parameters_set_control_surface_deflection(
            p.pin_mut(),
            params.control_surface_deflection,
        );
        ffi::IMeshTools_Parameters_set_allow_quality_decrease(
            p.pin_mut(),
            params.allow_quality_decrease,
        );
        ffi::IMeshTools_Parameters_set_mesh_algo(p.pin_mut(), params.mesh_algo.as_i32());

        // Meshing happens here: this constructor calls `Perform()` itself, so nothing
        // set on `p` after this point would reach the result.
        let inner = ffi::BRepMesh_IncrementalMesh_ctor_params(&shape.inner, &p);

        if inner.IsDone() {
            Ok(Self { inner })
        } else {
            Err(Error::TriangulationFailed)
        }
    }

    /// What OCCT recorded about the run that just happened.
    ///
    /// Cheap, and worth reading even on success: [`MeshStatus::failure`] names the
    /// condition that makes [`mesh`](Self::mesh) discard a whole shape, and
    /// [`MeshStatus::reused`] reports triangulation reuse directly instead of leaving
    /// it to be inferred from how long the mesher took.
    #[must_use]
    pub fn status_flags(&self) -> MeshStatus {
        MeshStatus(self.inner.GetStatusFlags())
    }

    /// Whether this run changed any triangulation at all.
    #[must_use]
    pub fn is_modified(&self) -> bool {
        self.inner.IsModified()
    }

    pub fn mesh(mut self) -> Result<Mesh, Error> {
        let mut vertices = vec![];
        let mut uvs = vec![];
        let mut normals = vec![];
        let mut indices = vec![];

        let triangulated_shape = Shape::from_shape(self.inner.pin_mut().Shape());

        for face in triangulated_shape.faces() {
            let mut location = ffi::TopLoc_Location_ctor();

            let triangulation_handle =
                ffi::BRep_Tool_Triangulation(&face.inner, location.pin_mut());

            let triangulation = ffi::HandlePoly_Triangulation_Get(&triangulation_handle)
                .map_err(|_| Error::UntriangulatedFace)?;

            let index_offset = vertices.len();
            let face_point_count = triangulation.NbNodes();

            for i in 1..=face_point_count {
                let mut point = ffi::Poly_Triangulation_Node(triangulation, i);
                point.pin_mut().Transform(&ffi::TopLoc_Location_Transformation(&location));
                vertices.push(dvec3(point.X(), point.Y(), point.Z()));
            }

            let mut u_min = f64::INFINITY;
            let mut v_min = f64::INFINITY;

            let mut u_max = f64::NEG_INFINITY;
            let mut v_max = f64::NEG_INFINITY;

            for i in 1..=(face_point_count) {
                let uv = ffi::Poly_Triangulation_UV(triangulation, i);
                let (u, v) = (uv.X(), uv.Y());

                u_min = u_min.min(u);
                v_min = v_min.min(v);

                u_max = u_max.max(u);
                v_max = v_max.max(v);

                uvs.push(dvec2(u, v));
            }

            // Normalize the newly added UV coordinates.
            for uv in &mut uvs[index_offset..(index_offset + face_point_count as usize)] {
                uv.x = (uv.x - u_min) / (u_max - u_min);
                uv.y = (uv.y - v_min) / (v_max - v_min);

                if face.orientation() != FaceOrientation::Forward {
                    uv.x = 1.0 - uv.x;
                }
            }

            // Add in the normals.
            // TODO(bschwind) - Use `location` to transform the normals.
            let normal_array = ffi::TColgp_Array1OfDir_ctor(0, face_point_count);

            ffi::compute_normals(&face.inner, &triangulation_handle);

            // TODO(bschwind) - Why do we start at 1 here?
            for i in 1..(normal_array.Length() as usize) {
                let normal = ffi::Poly_Triangulation_Normal(triangulation, i as i32);
                normals.push(dvec3(normal.X(), normal.Y(), normal.Z()));
            }

            for i in 1..=triangulation.NbTriangles() {
                let triangle = triangulation.Triangle(i);

                if face.orientation() == FaceOrientation::Forward {
                    indices.push(index_offset + triangle.Value(1) as usize - 1);
                    indices.push(index_offset + triangle.Value(2) as usize - 1);
                    indices.push(index_offset + triangle.Value(3) as usize - 1);
                } else {
                    indices.push(index_offset + triangle.Value(3) as usize - 1);
                    indices.push(index_offset + triangle.Value(2) as usize - 1);
                    indices.push(index_offset + triangle.Value(1) as usize - 1);
                }
            }
        }

        Ok(Mesh { vertices, uvs, normals, indices })
    }
}

#[cfg(test)]
mod tests {
    use super::{MeshParams, MeshStatus, Mesher};
    use crate::primitives::Shape;

    #[test]
    fn mesh_status_names_its_bits() {
        assert_eq!(MeshStatus(0).to_string(), "NoError");
        assert_eq!(MeshStatus(0x80).to_string(), "Reused");
        assert_eq!(MeshStatus(0x4 | 0x40).to_string(), "Failure|Outdated");
        // A bit OCCT might add later is reported, not dropped.
        assert_eq!(MeshStatus(0x200).to_string(), "0x200");
        assert_eq!(MeshStatus(0x4 | 0x200).to_string(), "Failure|0x200");

        assert!(MeshStatus(0).is_clean());
        assert!(MeshStatus(0x4).failure());
        assert!(MeshStatus(0x80).reused());
        assert!(!MeshStatus(0x80).failure());
    }

    /// A sphere rather than a cylinder or a box: `angle_interior` steers the face
    /// interior, and only a doubly-curved face has an interior that its boundary
    /// discretisation does not already determine. A cylinder's lateral face is
    /// developable and would leave the interior test unable to observe anything.
    fn counts(params: Option<&MeshParams>) -> (usize, usize) {
        let shape = Shape::sphere(10.0).build();
        let mesher = match params {
            None => Mesher::try_new_with_angle(&shape, 0.1, 0.5),
            Some(p) => Mesher::try_new_with_params(&shape, 0.1, 0.5, p),
        }
        .expect("sphere should mesh");
        let mesh = mesher.mesh().expect("sphere should extract");
        (mesh.vertices.len(), mesh.indices.len())
    }

    /// The null result: the full parameter set at its defaults must reproduce the
    /// five-argument constructor exactly. This is what proves the nine setters are
    /// wired to the fields they are named for — a transposition would show up here as
    /// a different mesh rather than as a compile error.
    #[test]
    fn default_params_reproduce_the_five_argument_ctor() {
        assert_eq!(counts(None), counts(Some(&MeshParams::default())));
    }

    /// `angle_interior` defaults to twice the boundary angle, so pinning it to the
    /// boundary angle must refine the interior. If these come back equal the field is
    /// not reaching the kernel.
    #[test]
    fn angle_interior_is_live_and_defaults_to_double() {
        let pinned = MeshParams { angle_interior: Some(0.5), ..MeshParams::default() };
        let (default_verts, _) = counts(None);
        let (pinned_verts, _) = counts(Some(&pinned));
        assert!(
            pinned_verts > default_verts,
            "interior at 0.5 rad should be finer than the default 1.0 rad: \
             {pinned_verts} vs {default_verts}"
        );
    }
}
