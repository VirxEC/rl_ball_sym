//! Various geometrical objects and tools.

use glam::{Mat3A, Vec3A};
use std::{
    array::IntoIter,
    ops::{Add, AddAssign},
};

#[must_use]
/// Find the distance between a ray and a point.
pub fn distance_between(start: Vec3A, dir: Vec3A, p: Vec3A) -> f32 {
    let u = ((p - start).dot(dir) / dir.length_squared()).clamp(0., 1.);
    (start + dir * u - p).length()
}

/// A triangle made from 3 points.
#[derive(Clone, Copy, Debug, Default)]
pub struct Tri([Vec3A; 3]);

impl IntoIterator for Tri {
    type Item = Vec3A;
    type IntoIter = IntoIter<Vec3A, 3>;

    #[inline]
    fn into_iter(self) -> Self::IntoIter {
        self.0.into_iter()
    }
}

impl Tri {
    #[must_use]
    #[inline]
    #[allow(dead_code)]
    /// Get one of the points of the triangle. Panics if the index is out of bounds.
    ///
    /// Used in tests.
    pub const fn get_point(&self, i: usize) -> Vec3A {
        self.0[i]
    }

    #[must_use]
    #[inline]
    #[allow(dead_code)]
    /// Create a new triangle from 3 points
    pub const fn from_points(p0: Vec3A, p1: Vec3A, p2: Vec3A) -> Self {
        Self([p0, p1, p2])
    }

    #[must_use]
    #[inline]
    /// Create a new triangle from an iterator that must be of 3 points
    pub fn from_points_iter(mut iter: impl Iterator<Item = Vec3A>) -> Self {
        Self([iter.next().unwrap(), iter.next().unwrap(), iter.next().unwrap()])
    }

    #[must_use]
    #[inline]
    /// Get the center of the triangle.
    pub fn center(self) -> Vec3A {
        (self.0[0] + self.0[1] + self.0[2]) / 3.
    }

    #[must_use]
    /// Get the normal of the triangle.
    pub fn unit_normal(self) -> Vec3A {
        (self.0[1] - self.0[0]).cross(self.0[2] - self.0[0]).normalize()
    }

    #[allow(clippy::many_single_char_names)]
    #[must_use]
    /// Check if a sphere intersects the triangle.
    pub fn intersect_sphere(&self, obj: Sphere) -> bool {
        let e1 = self.0[1] - self.0[0];
        let e3 = self.0[0] - self.0[2];
        let n = e3.cross(e1).normalize();

        let a: Mat3A = Mat3A::from_cols(e1, -e3, n);
        let x = a.inverse() * (obj.center - self.0[0]);

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
            (obj.radius + 1.)
                .min(distance_between(self.0[0], e1, obj.center))
                .min(distance_between(self.0[1], self.0[2] - self.0[1], obj.center))
                .min(distance_between(self.0[2], e3, obj.center))
        };

        dist <= obj.radius
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
    #[must_use]
    #[inline]
    #[allow(dead_code)]
    /// Create a new AABB.
    ///
    /// Used in tests.
    pub const fn new(min: Vec3A, max: Vec3A) -> Self {
        Self { min, max }
    }

    #[must_use]
    #[inline]
    /// The minimum point contained in the AABB.
    pub const fn min(self) -> Vec3A {
        self.min
    }

    #[must_use]
    #[inline]
    /// The maximum point contained in the AABB.
    pub const fn max(self) -> Vec3A {
        self.max
    }

    #[must_use]
    #[inline]
    /// Create an AABB from a triangle.
    pub fn from_tri(t: Tri) -> Self {
        Self {
            min: t.into_iter().reduce(Vec3A::min).unwrap(),
            max: t.into_iter().reduce(Vec3A::max).unwrap(),
        }
    }

    #[must_use]
    #[inline]
    /// Create an AABB from a sphere
    pub fn from_sphere(s: Sphere) -> Self {
        Self {
            min: s.center - s.radius,
            max: s.center + s.radius,
        }
    }

    #[must_use]
    #[inline]
    /// Check if another AABB intersects this one.
    pub fn intersect_self(self, b: &Self) -> bool {
        self.min.cmple(b.max).all() && self.max.cmpge(b.min).all()
    }

    #[must_use]
    #[allow(dead_code)]
    /// Check if a sphere intersects this AABB.
    ///
    /// Used in tests.
    pub fn intersect_sphere(&self, b: &Sphere) -> bool {
        let nearest = b.center.clamp(self.min, self.max);

        (b.center - nearest).length() <= b.radius
    }
}

impl Add for Aabb {
    type Output = Self;

    #[inline]
    fn add(self, rhs: Self) -> Self {
        Self {
            min: self.min.min(rhs.min),
            max: self.max.max(rhs.max),
        }
    }
}

impl From<Tri> for Aabb {
    #[inline]
    fn from(value: Tri) -> Self {
        Self::from_tri(value)
    }
}

impl From<Sphere> for Aabb {
    #[inline]
    fn from(value: Sphere) -> Self {
        Self::from_sphere(value)
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

impl AddAssign<Self> for Ray {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        self.start += rhs.start;
        self.direction += rhs.direction;
    }
}

impl Ray {
    #[inline]
    pub const fn new(start: Vec3A, direction: Vec3A) -> Self {
        Self { start, direction }
    }
}

/// A Sphere-like object.
#[derive(Clone, Copy, Debug, Default)]
pub struct Sphere {
    /// Location of the center of the sphere.
    pub center: Vec3A,
    /// Radius of the sphere.
    pub radius: f32,
}

#[cfg(test)]
mod test {
    use super::*;
    use glam::Vec3A;

    const TRI: Tri = Tri::from_points(
        Vec3A::new(-1.0, 5.0, 0.0),
        Vec3A::new(2.0, 2.0, -3.0),
        Vec3A::new(5.0, 5.0, 0.0),
    );

    const SPHERE: Sphere = Sphere {
        center: Vec3A::new(1.0, 0.0, 1.0),
        radius: 2.0,
    };

    const BOUNDING_BOXES: &[Aabb] = &[
        Aabb {
            min: Vec3A::new(-0.5, -2.0, -0.5),
            max: Vec3A::new(0.5, 2.0, 0.5),
        },
        Aabb {
            min: Vec3A::new(-1.0, -1.0, -1.0),
            max: Vec3A::new(1.0, 1.0, 1.0),
        },
        Aabb {
            min: Vec3A::new(1.0, 1.0, 1.0),
            max: Vec3A::new(3.0, 3.0, 3.0),
        },
        Aabb {
            min: Vec3A::new(-4.0, -4.0, -4.0),
            max: Vec3A::new(-3.0, -3.0, -3.0),
        },
    ];

    #[test]
    fn tri_sphere_intersect() {
        {
            let sphere = Sphere {
                center: Vec3A::new(2.0, 4.0, -1.0),
                radius: 0.5,
            };

            assert!(TRI.intersect_sphere(sphere));
        }
        {
            let sphere = Sphere {
                center: Vec3A::new(-1.0, 5.0, 0.0),
                radius: 0.5,
            };

            assert!(TRI.intersect_sphere(sphere));
        }
    }

    #[test]
    fn tri_sphere_not_intersect() {
        {
            let sphere = Sphere {
                center: Vec3A::splat(2.0),
                radius: 1.0,
            };

            assert!(!TRI.intersect_sphere(sphere));
        }
        {
            let sphere = Sphere {
                center: Vec3A::splat(-2.0),
                radius: 1.0,
            };

            assert!(!TRI.intersect_sphere(sphere));
        }
    }

    #[test]
    fn aabb_sphere_intersect() {
        {
            let aabb = Aabb {
                min: Vec3A::new(2.0, 1.0, 2.0),
                max: Vec3A::new(4.0, 3.0, 4.0),
            };

            assert!(aabb.intersect_sphere(&SPHERE));
        }
        {
            let aabb = Aabb {
                min: Vec3A::new(0.0, -1.0, 0.0),
                max: Vec3A::new(1.0, 0.0, 1.0),
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
