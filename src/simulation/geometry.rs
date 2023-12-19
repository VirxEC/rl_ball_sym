//! Various geometrical objects and tools.

use glam::Vec3A;
use std::ops::{Add, AddAssign};

#[repr(transparent)]
/// A triangle made from 3 points.
#[derive(Clone, Copy, Debug, Default)]
pub struct Tri {
    points: [Vec3A; 3],
}

impl Tri {
    #[inline]
    /// Create a new triangle from 3 points.
    pub const fn new(points: [Vec3A; 3]) -> Self {
        Self { points }
    }

    #[inline]
    /// Create a new triangle from an iterator that must be of 3 points
    pub fn from_points_iter(mut iter: impl Iterator<Item = Vec3A>) -> Self {
        Self::new([iter.next().unwrap(), iter.next().unwrap(), iter.next().unwrap()])
    }

    #[must_use]
    /// Get the normal of the triangle.
    pub fn unit_normal(self) -> Vec3A {
        (self.points[1] - self.points[0])
            .cross(self.points[2] - self.points[0])
            .normalize()
    }

    /// Check if a point projected onto the same place as the triangle
    /// is within the bounds of it.
    /// This is used instead of bullet's method because it's much faster:
    /// <https://gamedev.stackexchange.com/a/152476>
    fn face_contains(&self, point: Vec3A) -> bool {
        let u = self.points[1] - self.points[0];
        let v = self.points[2] - self.points[0];
        let n = u.cross(v);
        let w = point - self.points[0];
        let gamma = u.cross(w).dot(n) / n.dot(n);
        let beta = w.cross(v).dot(n) / n.dot(n);
        let alpha = 1. - gamma - beta;

        let gba = Vec3A::new(gamma, beta, alpha);
        gba.cmple(Vec3A::ONE).all() && gba.cmpge(Vec3A::ZERO).all()
    }

    /// Instead of using bullet's method,
    /// we use the method described here which is much faster:
    /// <https://stackoverflow.com/a/74395029/10930209>
    fn closest_point(&self, point: Vec3A) -> Vec3A {
        let ab = self.points[1] - self.points[0];
        let ac = self.points[2] - self.points[0];
        let ap = point - self.points[0];

        let d1 = ab.dot(ap);
        let d2 = ac.dot(ap);
        if d1 <= 0. && d2 <= 0. {
            return self.points[0];
        }

        let bp = point - self.points[1];
        let d3 = ab.dot(bp);
        let d4 = ac.dot(bp);
        if d3 >= 0. && d4 <= d3 {
            return self.points[1];
        }

        let cp = point - self.points[2];
        let d5 = ab.dot(cp);
        let d6 = ac.dot(cp);
        if d6 >= 0. && d5 <= d6 {
            return self.points[2];
        }

        let vc = d1 * d4 - d3 * d2;
        if vc <= 0. && d1 >= 0. && d3 <= 0. {
            let v = d1 / (d1 - d3);
            return self.points[0] + v * ab;
        }

        let vb = d5 * d2 - d1 * d6;
        if vb <= 0. && d2 >= 0. && d6 <= 0. {
            let v = d2 / (d2 - d6);
            return self.points[0] + v * ac;
        }

        let va = d3 * d6 - d5 * d4;
        if va <= 0. && (d4 - d3) >= 0. && (d5 - d6) >= 0. {
            let v = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            return self.points[1] + v * (self.points[2] - self.points[1]);
        }

        let denom = 1. / (va + vb + vc);
        let v = vb * denom;
        let w = vc * denom;
        self.points[0] + v * ab + w * ac
    }

    #[must_use]
    /// Check if a sphere intersects the triangle.
    pub fn intersect_sphere(&self, obj: Sphere) -> Option<Ray> {
        let mut normal = self.unit_normal();

        let p1_to_center = obj.center - self.points[0];
        let mut distance_from_plane = p1_to_center.dot(normal);

        if distance_from_plane < 0. {
            distance_from_plane *= -1.;
            normal *= -1.;
        }

        if distance_from_plane >= obj.radius_with_threshold {
            return None;
        }

        let contact_point = if self.face_contains(obj.center) {
            Some(obj.center - normal * distance_from_plane)
        } else {
            let min_dist_sqr = obj.radius_with_threshold.powi(2);

            let closest_point = self.closest_point(obj.center);
            let distance_sqr = (closest_point - obj.center).length_squared();

            if distance_sqr < min_dist_sqr {
                Some(closest_point)
            } else {
                None
            }
        }?;

        let contact_to_center = obj.center - contact_point;
        let distance_sqr = contact_to_center.length_squared();

        if distance_sqr >= obj.radius_with_threshold.powi(2) {
            return None;
        }

        let result_normal = if distance_sqr > f32::EPSILON {
            contact_to_center / distance_sqr.sqrt()
        } else {
            normal
        };

        Some(Ray::new(contact_point, result_normal))
    }
}

/// An axis-aligned bounding box.
/// Learn more here: <https://developer.nvidia.com/blog/thinking-parallel-part-i-collision-detection-gpu/>
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
            min: t.points[0].min(t.points[1]).min(t.points[2]),
            max: t.points[0].max(t.points[1]).max(t.points[2]),
        }
    }

    #[must_use]
    #[inline]
    /// Create an AABB from a sphere
    pub fn from_sphere(s: Sphere) -> Self {
        Self {
            min: s.center - s.radius_with_threshold,
            max: s.center + s.radius_with_threshold,
        }
    }

    #[inline]
    #[must_use]
    /// Check if another AABB intersects this one.
    pub fn intersect_self(self, b: &Self) -> bool {
        self.min.cmple(b.max).all() && self.max.cmpge(b.min).all()
    }

    #[must_use]
    #[cfg(test)]
    /// Check if a sphere intersects this AABB.
    ///
    /// Used in tests.
    pub fn intersect_sphere(&self, b: Sphere) -> bool {
        let nearest = b.center.clamp(self.min, self.max);

        (b.center - nearest).length() < b.radius
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
    pub center: Vec3A,
    pub radius: f32,
    pub radius_with_threshold: f32,
}

impl Sphere {
    pub const CONTACT_BREAKING_THRESHOLD: f32 = 1.905;

    #[inline]
    #[must_use]
    pub fn new(center: Vec3A, radius: f32) -> Self {
        Self {
            center,
            radius,
            radius_with_threshold: radius + Self::CONTACT_BREAKING_THRESHOLD,
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use glam::Vec3A;

    const TRI: Tri = Tri::new([
        Vec3A::new(-1.0, 5.0, 0.0),
        Vec3A::new(2.0, 2.0, -3.0),
        Vec3A::new(5.0, 5.0, 0.0),
    ]);

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
            let sphere = Sphere::new(Vec3A::new(2.0, 4.0, -1.0), 0.5);

            assert!(TRI.intersect_sphere(sphere).is_some());
        }
        {
            let sphere = Sphere::new(Vec3A::new(-1.0, 5.0, 0.0), 0.5);

            assert!(TRI.intersect_sphere(sphere).is_some());
        }
    }

    #[test]
    fn tri_sphere_not_intersect() {
        {
            let sphere = Sphere::new(Vec3A::splat(2.0), 1.0);

            assert!(TRI.intersect_sphere(sphere).is_none());
        }
        {
            let sphere = Sphere::new(Vec3A::splat(-2.0), 1.0);

            assert!(TRI.intersect_sphere(sphere).is_none());
        }
    }

    #[test]
    fn aabb_sphere_intersect() {
        {
            let aabb = Aabb {
                min: Vec3A::new(2.0, 1.0, 2.0),
                max: Vec3A::new(4.0, 3.0, 4.0),
            };

            assert!(aabb.intersect_sphere(Sphere::new(Vec3A::new(1.0, 0.0, 1.0), 2.0)));
        }
        {
            let aabb = Aabb {
                min: Vec3A::new(0.0, -1.0, 0.0),
                max: Vec3A::new(1.0, 0.0, 1.0),
            };

            assert!(aabb.intersect_sphere(Sphere::new(Vec3A::new(1.0, 0.0, 1.0), 2.0)));
        }
        {
            let aabb = Aabb {
                min: Vec3A::splat(-5.0),
                max: Vec3A::splat(5.0),
            };

            assert!(aabb.intersect_sphere(Sphere::new(Vec3A::new(1.0, 0.0, 1.0), 2.0)));
        }
    }

    #[test]
    fn aabb_sphere_not_intersect() {
        let aabb = Aabb {
            min: Vec3A::splat(-2.0),
            max: Vec3A::splat(-1.0),
        };

        assert!(!aabb.intersect_sphere(Sphere::new(Vec3A::new(1.0, 0.0, 1.0), 2.0)));
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
