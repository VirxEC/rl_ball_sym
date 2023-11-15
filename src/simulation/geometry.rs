//! Various geometrical objects and tools.

use glam::Vec3A;
use std::{
    array::IntoIter,
    ops::{Add, AddAssign},
};

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

    // called by SphereTriangleDetector::facecontains
    // bool SphereTriangleDetector::pointInTriangle(const btVector3 vertices[], const btVector3& normal, btVector3* p)
    pub fn face_contains(self, normal: Vec3A, point: Vec3A) -> bool {
        // const btVector3* p1 = &vertices[0];
        // const btVector3* p2 = &vertices[1];
        // const btVector3* p3 = &vertices[2];

        // btVector3 edge1(*p2 - *p1);
        let edge1 = self.0[1] - self.0[0];
        // btVector3 edge2(*p3 - *p2);
        let edge2 = self.0[2] - self.0[1];
        // btVector3 edge3(*p1 - *p3);
        let edge3 = self.0[0] - self.0[2];

        // btVector3 p1_to_p(*p - *p1);
        let p1_to_p = point - self.0[0];
        // btVector3 p2_to_p(*p - *p2);
        let p2_to_p = point - self.0[1];
        // btVector3 p3_to_p(*p - *p3);
        let p3_to_p = point - self.0[2];

        // btVector3 edge1_normal(edge1.cross(normal));
        let edge1_normal = edge1.cross(normal);
        // btVector3 edge2_normal(edge2.cross(normal));
        let edge2_normal = edge2.cross(normal);
        // btVector3 edge3_normal(edge3.cross(normal));
        let edge3_normal = edge3.cross(normal);

        // btScalar r1, r2, r3;
        // r1 = edge1_normal.dot(p1_to_p);
        let r1 = edge1_normal.dot(p1_to_p);
        // r2 = edge2_normal.dot(p2_to_p);
        let r2 = edge2_normal.dot(p2_to_p);
        // r3 = edge3_normal.dot(p3_to_p);
        let r3 = edge3_normal.dot(p3_to_p);

        // if ((r1 > 0 && r2 > 0 && r3 > 0) ||
        //     (r1 <= 0 && r2 <= 0 && r3 <= 0))
        //     return true;
        // return false;
        (r1 > 0. && r2 > 0. && r3 > 0.) || (r1 <= 0. && r2 <= 0. && r3 <= 0.)
    }

    // btScalar SegmentSqrDistance(const btVector3& from, const btVector3& to, const btVector3& p, btVector3& nearest)
    fn segment_sqr_distance(from: Vec3A, to: Vec3A, p: Vec3A, nearest: &mut Vec3A) -> f32 {
        // btVector3 diff = p - from;
        let mut diff = p - from;
        // btVector3 v = to - from;
        let v = to - from;
        // btScalar t = v.dot(diff);
        let mut t = v.dot(diff);

        // if (t > 0)
        if t > 0. {
            // btScalar dotVV = v.dot(v);
            let dot_vv = v.dot(v);
            // if (t < dotVV)
            if t < dot_vv {
                // t /= dotVV;
                t /= dot_vv;
                // diff -= t * v;
                diff -= t * v;
            } else {
                // else
                // t = 1;
                t = 1.;
                // diff -= v;
                diff -= v;
            }
        } else {
            // else
            // t = 0;
            t = 0.;
        }

        // nearest = from + t * v;
        *nearest = from + t * v;
        // return diff.dot(diff);
        diff.dot(diff)
    }

    #[must_use]
    /// Check if a sphere intersects the triangle.
    pub fn intersect_sphere(&self, obj: Sphere) -> Option<(Ray, Vec3A)> {
        // btScalar radius = m_sphere->getRadius();
        // btScalar radiusWithThreshold = radius + contactBreakingThreshold;
        let radius = obj.radius - Sphere::get_contact_breaking_threshold();
        let radius_with_threshold = obj.radius;

        // btVector3 normal = (vertices[1] - vertices[0]).cross(vertices[2] - vertices[0]);
        let mut normal = (self.0[1] - self.0[0]).cross(self.0[2] - self.0[0]);

        // btScalar l2 = normal.length2();
        let l2 = normal.length_squared();
        // bool hasContact = false;
        let mut has_contact = false;
        // btVector3 contactPoint;
        let mut contact_point = Vec3A::ZERO;

        // if (l2 >= SIMD_EPSILON * SIMD_EPSILON)
        if l2 >= f32::EPSILON.powi(2) {
            // normal /= btSqrt(l2);
            normal /= l2.sqrt();

            // btVector3 p1ToCentre = sphereCenter - vertices[0];
            let p1_to_center = obj.center - self.0[0];
            // btScalar distanceFromPlane = p1ToCentre.dot(normal);
            let mut distance_from_plane = p1_to_center.dot(normal);

            // if (distanceFromPlane < btScalar(0.))
            if distance_from_plane < 0. {
                // triangle facing the other way
                // distanceFromPlane *= btScalar(-1.);
                distance_from_plane *= -1.;
                // normal *= btScalar(-1.);
                normal *= -1.;
            }

            // bool isInsideContactPlane = distanceFromPlane < radiusWithThreshold;
            let is_inside_contact_plane = distance_from_plane < radius_with_threshold;

            // Check for contact / intersection

            // if (isInsideContactPlane)
            if is_inside_contact_plane {
                // if (facecontains(sphereCenter, vertices, normal))
                if self.face_contains(normal, obj.center) {
                    // Inside the contact wedge - touches a point on the shell plane
                    // hasContact = true;
                    has_contact = true;
                    // contactPoint = sphereCenter - normal * distanceFromPlane;
                    contact_point = obj.center - normal * distance_from_plane;
                } else {
                    // Could be inside one of the contact capsules

                    // btScalar contactCapsuleRadiusSqr = radiusWithThreshold * radiusWithThreshold;
                    let contact_capsule_radius_sqr = radius_with_threshold.powi(2);
                    // btScalar minDistSqr = contactCapsuleRadiusSqr;
                    let mut min_dist_sqr = contact_capsule_radius_sqr;

                    // btVector3 nearestOnEdge;
                    let mut nearest_on_edge = Vec3A::ZERO;
                    // for (int i = 0; i < m_triangle->getNumEdges(); i++)
                    for i in 0..self.0.len() {
                        // btVector3 pa;
                        // btVector3 pb;

                        // m_triangle->getEdge(i, pa, pb);
                        // getVertex(i, pa);
                        let pa = self.0[i];
                        // getVertex((i + 1) % 3, pb);
                        let pb = self.0[(i + 1) % 3];

                        // btScalar distanceSqr = SegmentSqrDistance(pa, pb, sphereCenter, nearestOnEdge);
                        let distance_sqr = Self::segment_sqr_distance(pa, pb, obj.center, &mut nearest_on_edge);
                        // if (distanceSqr < minDistSqr)
                        if distance_sqr < min_dist_sqr {
                            // Yep, we're inside a capsule, and record the capsule with smallest distance
                            // minDistSqr = distanceSqr;
                            min_dist_sqr = distance_sqr;
                            // hasContact = true;
                            has_contact = true;
                            // contactPoint = nearestOnEdge;
                            contact_point = nearest_on_edge;
                        }
                    }
                }
            }
        }

        // if (hasContact)
        if has_contact {
            // btVector3 contactToCentre = sphereCenter - contactPoint;
            let contact_to_center = obj.center - contact_point;
            // btScalar distanceSqr = contactToCentre.length2();
            let distance_sqr = contact_to_center.length_squared();

            // if (distanceSqr < radiusWithThreshold * radiusWithThreshold)
            if distance_sqr < radius_with_threshold.powi(2) {
                // if (distanceSqr > SIMD_EPSILON)
                let (point, result_normal, depth) = if distance_sqr > f32::EPSILON {
                    // btScalar distance = btSqrt(distanceSqr);
                    let distance = distance_sqr.sqrt();
                    // resultNormal = contactToCentre;
                    // resultNormal.normalize();
                    let result_normal = contact_to_center.normalize();
                    // point = contactPoint;
                    let point = contact_point;
                    // depth = -(radius - distance);
                    let depth = -(radius - distance);
                    (point, result_normal, depth)
                } else {
                    // resultNormal = normal;
                    let result_normal = normal;
                    // point = contactPoint;
                    let point = contact_point;
                    // depth = -radius;
                    let depth = -radius;
                    (point, result_normal, depth)
                };
                // dbg!(result_normal, point / 50., depth / 50., normal);
                return Some((Ray::new(point, result_normal, depth), normal));
                // return true;
            }
        }

        None
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
    /// Length of the ray.
    pub depth: f32,
}

impl AddAssign<Self> for Ray {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        self.start += rhs.start;
        self.direction += rhs.direction;
        self.depth += rhs.depth;
    }
}

impl Ray {
    #[inline]
    pub const fn new(start: Vec3A, direction: Vec3A, depth: f32) -> Self {
        Self { start, direction, depth }
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

impl Sphere {
    #[inline]
    pub const fn get_contact_breaking_threshold() -> f32 {
        1.905
    }
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

            assert!(TRI.intersect_sphere(sphere).is_some());
        }
        {
            let sphere = Sphere {
                center: Vec3A::new(-1.0, 5.0, 0.0),
                radius: 0.5,
            };

            assert!(TRI.intersect_sphere(sphere).is_some());
        }
    }

    #[test]
    fn tri_sphere_not_intersect() {
        {
            let sphere = Sphere {
                center: Vec3A::splat(2.0),
                radius: 1.0,
            };

            assert!(TRI.intersect_sphere(sphere).is_none());
        }
        {
            let sphere = Sphere {
                center: Vec3A::splat(-2.0),
                radius: 1.0,
            };

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
