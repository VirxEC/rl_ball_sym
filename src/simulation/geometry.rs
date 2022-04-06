use crate::linear_algebra::math::{dot, local};
use glam::{Mat3A, Vec3A};

/// Find the distance between a ray and a point.
pub fn distance_between(start: Vec3A, dir: Vec3A, p: Vec3A) -> f32 {
    let u = ((p - start).dot(dir) / dir.length_squared()).clamp(0., 1.);
    (start + dir * u - p).length()
}

/// A triangle made from 3 points.
#[derive(Clone, Copy, Debug, Default)]
pub struct Tri {
    pub p: [Vec3A; 3],
}

impl Tri {
    /// Get the center of the triangle.
    pub fn center(&self) -> Vec3A {
        self.p.iter().sum::<Vec3A>() / 3.
    }

    /// Get the normal of the triangle.
    pub fn unit_normal(&self) -> Vec3A {
        (self.p[1] - self.p[0]).cross(self.p[2] - self.p[0]).normalize()
    }

    /// Check if a sphere intersects the triangle.
    #[allow(clippy::many_single_char_names)]
    pub fn intersect_sphere(&self, b: &Sphere) -> bool {
        let e1 = self.p[1] - self.p[0];
        let e2 = self.p[2] - self.p[1];
        let e3 = self.p[0] - self.p[2];
        let n = e3.cross(e1).normalize();

        let a = Mat3A::from_cols_array_2d(&[[e1.x, -e3.x, n.x], [e1.y, -e3.y, n.y], [e1.z, -e3.z, n.z]]);
        let x = dot(a.inverse(), b.center - self.p[0]);

        let u = x.x;
        let v = x.y;
        let w = 1. - u - v;
        let z = x.z;

        // if the projection of sphere's center
        // along the triangle normal puts it inside
        // the triangle, then we can just check
        // the out-of-plane distance
        // otherwise, check the distances to
        // the closest edge of the triangle
        let dist = if (0. ..=1.).contains(&u) && (0. ..=1.).contains(&v) && (0. ..=1.).contains(&w) {
            z.abs()
        } else {
            (b.radius + 1.).min(distance_between(self.p[0], e1, b.center)).min(distance_between(self.p[1], e2, b.center)).min(distance_between(self.p[2], e3, b.center))
        };

        dist <= b.radius
    }
}

// AABB stands for "Axis-Aligned Bounding Boxes"
// Learn more here: https://developer.nvidia.com/blog/thinking-parallel-part-i-collision-detection-gpu/
/// An axis-aligned bounding box.
#[derive(Clone, Copy, Debug, Default)]
pub struct Aabb {
    min: Vec3A,
    max: Vec3A,
}

impl Aabb {
    /// Create a new AABB.
    pub const fn from(min: Vec3A, max: Vec3A) -> Aabb {
        Aabb {
            min,
            max,
        }
    }

    /// The minimum point contained in the AABB.
    pub const fn min(&self) -> Vec3A {
        self.min
    }

    /// The maximum point contained in the AABB.
    pub const fn max(&self) -> Vec3A {
        self.max
    }

    /// Combine two AABBs.
    pub fn add(&self, b: &Aabb) -> Self {
        Self {
            min: self.min.min(b.min),
            max: self.max.max(b.max),
        }
    }

    /// Create an AABB from a triangle.
    pub fn from_tri(t: &Tri) -> Self {
        let min = t.p.into_iter().reduce(Vec3A::min).unwrap();
        let max = t.p.into_iter().reduce(Vec3A::max).unwrap();

        Self {
            min,
            max,
        }
    }

    /// Create an AABB from a sphere
    pub fn from_sphere(s: &Sphere) -> Self {
        Self {
            min: s.center - s.radius,
            max: s.center + s.radius,
        }
    }

    /// Check if another AABB intersects this one.
    pub fn intersect_self(&self, b: &Aabb) -> bool {
        self.min.cmple(b.max).all() && self.max.cmpge(b.min).all()
    }

    /// Check if a sphere intersects this AABB.
    pub fn intersect_sphere(&self, b: &Sphere) -> bool {
        let nearest = b.center.clamp(self.min, self.max);

        (b.center - nearest).length() <= b.radius
    }
}

impl From<&'_ Tri> for Aabb {
    fn from(value: &'_ Tri) -> Self {
        Aabb::from_tri(value)
    }
}

impl From<&'_ Sphere> for Aabb {
    fn from(value: &'_ Sphere) -> Self {
        Aabb::from_sphere(value)
    }
}

/// A ray starting at `start` and going in `direction`.
#[derive(Clone, Copy, Debug, Default)]
pub struct Ray {
    /// Starting location of the ray.
    pub start: Vec3A,
    /// Direction the ray is pointing.
    pub direction: Vec3A,
}

/// A Sphere-like object.
#[derive(Clone, Copy, Debug, Default)]
pub struct Sphere {
    /// Location of the center of the sphere.
    pub center: Vec3A,
    /// Radius of the sphere.
    pub radius: f32,
}

/// A generic object bounding box
#[derive(Clone, Copy, Debug, Default)]
pub struct Obb {
    /// Location of the center of the OBB.
    pub center: Vec3A,
    /// Distance of the center to the front, side, and top of the OBB.
    pub half_width: Vec3A,
    /// Rotation of the OBB.
    pub orientation: Mat3A,
}

impl Obb {
    /// Create a new OBB.
    pub fn new(location: Vec3A, orientation: Mat3A, dimensions: Vec3A, offset: Vec3A) -> Self {
        Self {
            orientation,
            half_width: dimensions / 2.,
            center: dot(orientation, offset) + location,
        }
    }

    /// Get the closest point on the OBB to a given point.
    pub fn closest_point_on_obb(&self, v: Vec3A) -> Vec3A {
        let mut v_local = local(v - self.center, self.orientation);

        v_local[0] = v_local[0].clamp(-self.half_width[0], self.half_width[0]);
        v_local[1] = v_local[1].clamp(-self.half_width[1], self.half_width[1]);
        v_local[2] = v_local[2].clamp(-self.half_width[2], self.half_width[2]);

        dot(self.orientation, v_local) + self.center
    }
}

#[cfg(test)]
mod test {
    use glam::{const_vec3a, vec3a};

    use super::*;

    const TRI: Tri = Tri {
        p: [const_vec3a!([-1.0, 5.0, 0.0]), const_vec3a!([2.0, 2.0, -3.0]), const_vec3a!([5.0, 5.0, 0.0])],
    };

    const SPHERE: Sphere = Sphere {
        center: const_vec3a!([1.0, 0.0, 1.0]),
        radius: 2.0,
    };

    const BOUNDING_BOXES: &[Aabb] = &[
        Aabb {
            min: const_vec3a!([-0.5, -2.0, -0.5]),
            max: const_vec3a!([0.5, 2.0, 0.5]),
        },
        Aabb {
            min: const_vec3a!([-1.0, -1.0, -1.0]),
            max: const_vec3a!([1.0, 1.0, 1.0]),
        },
        Aabb {
            min: const_vec3a!([1.0, 1.0, 1.0]),
            max: const_vec3a!([3.0, 3.0, 3.0]),
        },
        Aabb {
            min: const_vec3a!([-4.0, -4.0, -4.0]),
            max: const_vec3a!([-3.0, -3.0, -3.0]),
        },
    ];

    #[test]
    fn tri_sphere_intersect() {
        {
            let sphere = Sphere {
                center: vec3a(2.0, 4.0, -1.0),
                radius: 0.5,
            };

            assert!(TRI.intersect_sphere(&sphere));
        }
        {
            let sphere = Sphere {
                center: vec3a(-1.0, 5.0, 0.0),
                radius: 0.5,
            };

            assert!(TRI.intersect_sphere(&sphere));
        }
    }

    #[test]
    fn tri_sphere_not_intersect() {
        {
            let sphere = Sphere {
                center: Vec3A::splat(2.0),
                radius: 1.0,
            };

            assert!(!TRI.intersect_sphere(&sphere));
        }
        {
            let sphere = Sphere {
                center: Vec3A::splat(-2.0),
                radius: 1.0,
            };

            assert!(!TRI.intersect_sphere(&sphere));
        }
    }

    #[test]
    fn aabb_sphere_intersect() {
        {
            let aabb = Aabb {
                min: vec3a(2.0, 1.0, 2.0),
                max: vec3a(4.0, 3.0, 4.0),
            };

            assert!(aabb.intersect_sphere(&SPHERE));
        }
        {
            let aabb = Aabb {
                min: vec3a(0.0, -1.0, 0.0),
                max: vec3a(1.0, 0.0, 1.0),
            };

            assert!(aabb.intersect_sphere(&SPHERE));
        }
        {
            let aabb = Aabb {
                min: Vec3A::splat(-5.0),
                max: Vec3A::splat(5.0),
            };

            assert!(aabb.intersect_sphere(&SPHERE));
        }
    }

    #[test]
    fn aabb_sphere_not_intersect() {
        let aabb = Aabb {
            min: Vec3A::splat(-2.0),
            max: Vec3A::splat(-1.0),
        };

        assert!(!aabb.intersect_sphere(&SPHERE));
    }

    #[test]
    fn aabb_aabb_intersect() {
        // Test for intersection with itself
        assert!(BOUNDING_BOXES[0].intersect_self(&BOUNDING_BOXES[0]));
        assert!(BOUNDING_BOXES[1].intersect_self(&BOUNDING_BOXES[1]));
        assert!(BOUNDING_BOXES[2].intersect_self(&BOUNDING_BOXES[2]));
        assert!(BOUNDING_BOXES[3].intersect_self(&BOUNDING_BOXES[3]));

        // Test for intersection with other bounding boxes
        assert!(BOUNDING_BOXES[0].intersect_self(&BOUNDING_BOXES[1]));
        assert!(BOUNDING_BOXES[1].intersect_self(&BOUNDING_BOXES[0]));
        assert!(BOUNDING_BOXES[1].intersect_self(&BOUNDING_BOXES[2]));
        assert!(BOUNDING_BOXES[2].intersect_self(&BOUNDING_BOXES[1]));
    }

    #[test]
    fn aabb_aabb_not_intersect() {
        assert!(!BOUNDING_BOXES[0].intersect_self(&BOUNDING_BOXES[2]));
        assert!(!BOUNDING_BOXES[0].intersect_self(&BOUNDING_BOXES[3]));
        assert!(!BOUNDING_BOXES[1].intersect_self(&BOUNDING_BOXES[3]));
        assert!(!BOUNDING_BOXES[2].intersect_self(&BOUNDING_BOXES[0]));
        assert!(!BOUNDING_BOXES[2].intersect_self(&BOUNDING_BOXES[3]));
        assert!(!BOUNDING_BOXES[3].intersect_self(&BOUNDING_BOXES[0]));
        assert!(!BOUNDING_BOXES[3].intersect_self(&BOUNDING_BOXES[1]));
        assert!(!BOUNDING_BOXES[3].intersect_self(&BOUNDING_BOXES[2]));
    }
}
