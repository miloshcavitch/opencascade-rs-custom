use thiserror::Error;

/// **The `glam` this crate's signatures are written in**, re-exported so a consumer can
/// name it.
///
/// Not a convenience. A downstream workspace on a different `glam` major sees two
/// unrelated types that share a name, spelling, field set and layout — `DVec3` and
/// `DVec3` — and the compiler's diagnostic for it is the famously unhelpful
/// `expected DVec3, found DVec3`. Without this, a caller's only recourses are to pin its
/// own `glam` to ours, or to `transmute`. The first couples an unrelated workspace's
/// version choice to this crate's; the second compiles today and becomes a silent
/// reinterpretation the moment either side changes layout.
///
/// So the boundary is made **nameable** instead: `opencascade::glam::DVec3` is
/// unambiguously the one these functions take, and a conversion written against it is a
/// conversion someone can see.
pub use glam;

pub mod angle;
pub mod bounding_box;
pub mod kicad;
pub mod mesh;
pub mod primitives;
pub mod section;
pub mod workplane;

mod law_function;
mod make_pipe_shell;

#[derive(Error, Debug)]
pub enum Error {
    #[error("failed to write STL file")]
    StlWriteFailed,
    #[error("failed to read STEP file")]
    StepReadFailed,
    #[error("failed to read IGES file")]
    IgesReadFailed,
    #[error("failed to read KiCAD PCB file: {0}")]
    KicadReadFailed(#[from] kicad_parser::Error),
    #[error("failed to write STEP file")]
    StepWriteFailed,
    #[error("failed to write IGES file")]
    IgesWriteFailed,
    #[error("failed to triangulate Shape")]
    TriangulationFailed,
    #[error("encountered a face with no triangulation")]
    UntriangulatedFace,
    #[error("at least 2 points are required for creating a wire")]
    NotEnoughPoints,
}
