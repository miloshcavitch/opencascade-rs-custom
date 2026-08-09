use cxx::UniquePtr;
use glam::DVec3;
use opencascade_sys::ffi;

use crate::primitives::Shape;

/// A wrapper around the `Bnd_Box` API of OCC. Note that a `Bnd_Box` has a `Gap`
/// property, which is a small tolerance value added to all dimensions. This
/// means that the point values of a `BoundingBox` will often be slightly larger
/// or smaller than expected of the geometry of known shapes.
pub struct BoundingBox {
    pub(crate) inner: UniquePtr<ffi::Bnd_Box>,
}
impl BoundingBox {
    /// Create a new void box. A void box in OCC is defined as a box that contains no points.
    pub fn void() -> BoundingBox {
        Self { inner: ffi::Bnd_Box_ctor() }
    }

    pub fn is_void(&self) -> bool {
        self.inner.IsVoid()
    }

    pub fn get_gap(&self) -> f64 {
        self.inner.GetGap()
    }

    pub fn min(&self) -> DVec3 {
        let p = ffi::Bnd_Box_CornerMin(self.inner.as_ref().unwrap());
        glam::dvec3(p.X(), p.Y(), p.Z())
    }

    pub fn max(&self) -> DVec3 {
        let p = ffi::Bnd_Box_CornerMax(self.inner.as_ref().unwrap());
        glam::dvec3(p.X(), p.Y(), p.Z())
    }

    /// Get a vector corresponding to the `gap` of this box in all dimensions.
    pub fn gap_vec(&self) -> DVec3 {
        glam::DVec3::ONE * self.get_gap()
    }
}

/// Compute the axis-aligned bounding box of `shape` using the `BRepBndLib`
/// package.
///
/// Note that this uses the shape's triangulation when one is present, and falls
/// back to a cheap geometric estimate when it is not — for a B-spline face that
/// estimate is the control-polygon hull, which can be far larger than the
/// surface. A shape therefore reports *different* boxes before and after it has
/// been meshed. Use [`aabb_optimal`] when the answer has to be stable.
pub fn aabb(shape: &Shape) -> BoundingBox {
    let mut bb = BoundingBox::void();
    ffi::BRepBndLib_Add(
        shape.inner.as_ref().expect("Input shape ref was null"),
        bb.inner.pin_mut(),
        true,
    );
    bb
}

/// Compute a tight axis-aligned bounding box by sampling the actual curves and
/// surfaces (`BRepBndLib::AddOptimal`).
///
/// Triangulation is deliberately ignored, so the result depends only on the
/// geometry and does not change once the shape has been meshed. Slower than
/// [`aabb`] — it samples rather than reading poles — but it is the variant to
/// reach for when the box feeds a cache key or a tolerance.
pub fn aabb_optimal(shape: &Shape) -> BoundingBox {
    let mut bb = BoundingBox::void();
    ffi::BRepBndLib_AddOptimal(
        shape.inner.as_ref().expect("Input shape ref was null"),
        bb.inner.pin_mut(),
        false,
        false,
    );
    bb
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn create_bounding_box() {
        let bb = BoundingBox::void();
        assert!(bb.is_void());
    }

    #[test]
    fn get_bounding_box_of_sphere() {
        let s = Shape::sphere(1.0).build();

        let bb = aabb(&s);

        assert_eq!(bb.min(), glam::dvec3(-1.0, -1.0, -1.0) - bb.gap_vec());
        assert_eq!(bb.max(), glam::dvec3(1.0, 1.0, 1.0) + bb.gap_vec());
    }

    #[test]
    fn get_bounding_box_of_sphere_transformed() {
        let s = Shape::sphere(1.0).at(glam::dvec3(1.0, 2.0, 3.0)).build();

        let bb = aabb(&s);
        let gap = bb.gap_vec();
        assert_eq!(bb.min(), glam::dvec3(0.0, 1.0, 2.0) - gap);
        assert_eq!(bb.max(), glam::dvec3(2.0, 3.0, 4.0) + gap);
    }
}
