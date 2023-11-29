//! Various geometrical objects and tools.

use crate::simulation::game::Constraints;
use combo_vec::ReArr;
use glam::Vec3A;
use std::ops::{Add, AddAssign};

#[derive(Debug)]
pub struct Contact {
    pub ray: Ray,
    pub triangle_normal: Vec3A,
    pub position: Vec3A,
    pub local_position: Vec3A,
}

#[derive(Debug)]
#[repr(transparent)]
pub struct Hits(ReArr<Contact, { Constraints::MAX_CONTACTS }>);

impl Hits {
    #[inline]
    pub const fn new() -> Self {
        Self(ReArr::new())
    }

    // int btPersistentManifold::getCacheEntry(const btManifoldPoint& newPoint) const
    fn nearest_point(&self, new_contact: &Contact) -> usize {
        // btScalar shortestDist = getContactBreakingThreshold() * getContactBreakingThreshold();
        let mut shortest_dist = Sphere::CONTACT_BREAKING_THRESHOLD.powi(2);
        // int size = getNumContacts();
        // int nearestPoint = -1;
        let mut nearest_point = self.0.len();
        // for (int i = 0; i < size; i++)
        for (i, contact) in self.0.iter().enumerate() {
            // const btManifoldPoint& mp = m_pointCache[i];

            // btVector3 diffA = mp.m_localPointA - newPoint.m_localPointA;
            let diff_a = contact.local_position - new_contact.local_position;
            // const btScalar distToManiPoint = diffA.dot(diffA);
            let dist_to_mani_point = diff_a.dot(diff_a);
            // if (distToManiPoint < shortestDist)
            if dist_to_mani_point < shortest_dist {
                // shortestDist = distToManiPoint;
                shortest_dist = dist_to_mani_point;
                // nearestPoint = i;
                nearest_point = i;
            }
        }
        // return nearestPoint;
        nearest_point
    }

    // int btPersistentManifold::sortCachedPoints(const btManifoldPoint& pt)
    fn replacement_index(&self, new_contact: &Contact) -> usize {
        //calculate 4 possible cases areas, and take biggest area
        //also need to keep 'deepest'

        // int maxPenetrationIndex = -1;
        let mut max_penetration_index = self.0.len();
        // btScalar maxPenetration = pt.getDistance();
        let mut max_penetration = new_contact.ray.depth;
        // for (int i = 0; i < 4; i++)
        for (i, contact) in self.0.iter().enumerate() {
            // if (m_pointCache[i].getDistance() < maxPenetration)
            if contact.ray.depth < max_penetration {
                // maxPenetrationIndex = i;
                max_penetration_index = i;
                // maxPenetration = m_pointCache[i].getDistance();
                max_penetration = contact.ray.depth;
            }
        }

        // btScalar res0(btScalar(0.)), res1(btScalar(0.)), res2(btScalar(0.)), res3(btScalar(0.));
        let mut res = [0., 0., 0., 0.];

        // if (maxPenetrationIndex != 0)
        if max_penetration_index != 0 {
            // btVector3 a0 = pt.m_localPointA - m_pointCache[1].m_localPointA;
            let a0 = new_contact.local_position - self.0[1].local_position;
            // btVector3 b0 = m_pointCache[3].m_localPointA - m_pointCache[2].m_localPointA;
            let b0 = self.0[3].local_position - self.0[2].local_position;
            // btVector3 cross = a0.cross(b0);
            let cross = a0.cross(b0);
            // res0 = cross.length2();
            res[0] = cross.length_squared();
        }

        // if (maxPenetrationIndex != 1)
        if max_penetration_index != 1 {
            // btVector3 a1 = pt.m_localPointA - m_pointCache[0].m_localPointA;
            let a1 = new_contact.local_position - self.0[0].local_position;
            // btVector3 b1 = m_pointCache[3].m_localPointA - m_pointCache[2].m_localPointA;
            let b1 = self.0[3].local_position - self.0[2].local_position;
            // btVector3 cross = a1.cross(b1);
            let cross = a1.cross(b1);
            // res1 = cross.length2();
            res[1] = cross.length_squared();
        }

        // if (maxPenetrationIndex != 2)
        if max_penetration_index != 2 {
            // btVector3 a2 = pt.m_localPointA - m_pointCache[0].m_localPointA;
            let a2 = new_contact.local_position - self.0[0].local_position;
            // btVector3 b2 = m_pointCache[3].m_localPointA - m_pointCache[1].m_localPointA;
            let b2 = self.0[3].local_position - self.0[1].local_position;
            // btVector3 cross = a2.cross(b2);
            let cross = a2.cross(b2);
            // res2 = cross.length2();
            res[2] = cross.length_squared();
        }

        // if (maxPenetrationIndex != 3)
        if max_penetration_index != 3 {
            // btVector3 a3 = pt.m_localPointA - m_pointCache[0].m_localPointA;
            let a3 = new_contact.local_position - self.0[0].local_position;
            // btVector3 b3 = m_pointCache[2].m_localPointA - m_pointCache[1].m_localPointA;
            let b3 = self.0[2].local_position - self.0[1].local_position;
            // btVector3 cross = a3.cross(b3);
            let cross = a3.cross(b3);
            // res3 = cross.length2();
            res[3] = cross.length_squared();
        }

        // btVector4 maxvec(res0, res1, res2, res3);
        // int biggestarea = maxvec.closestAxis4();
        // return biggestarea;

        // get the index of the biggest element
        let mut biggest_area = res[0];
        let mut biggest_area_index = 0;
        
        if res[1] > biggest_area {
            biggest_area = res[1];
            biggest_area_index = 1;
        }

        if res[2] > biggest_area {
            biggest_area = res[2];
            biggest_area_index = 2;
        }

        if res[3] > biggest_area {
            biggest_area_index = 3;
        }

        biggest_area_index
    }

    pub fn push(&mut self, contact: Contact) {
        let mut index = self.nearest_point(&contact);

        if index < self.0.len() {
            // replaceContactPoint
            self.0[index] = contact;
        } else {
            // btPersistentManifold::addManifoldPoint
            if index == Constraints::MAX_CONTACTS {
                index = self.replacement_index(&contact);
            }

            if index == self.0.len() {
                self.0.push(contact);
            } else {
                self.0[index] = contact;
            }
        }
    }

    #[inline]
    pub fn inner(self) -> ReArr<Contact, { Constraints::MAX_CONTACTS }> {
        self.0
    }
}

#[repr(transparent)]
/// A triangle made from 3 points.
#[derive(Clone, Copy, Debug, Default)]
pub struct Tri {
    points: [Vec3A; 3],
}

impl Tri {
    #[inline]
    const fn new(points: [Vec3A; 3]) -> Self {
        Self { points }
    }

    #[cfg(test)]
    /// Create a new triangle from 3 points
    pub const fn from_points(p0: Vec3A, p1: Vec3A, p2: Vec3A) -> Self {
        Self::new([p0, p1, p2])
    }

    #[inline]
    /// Create a new triangle from an iterator that must be of 3 points
    pub fn from_points_iter(mut iter: impl Iterator<Item = Vec3A>) -> Self {
        Self::new([iter.next().unwrap(), iter.next().unwrap(), iter.next().unwrap()])
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
    pub fn intersect_sphere(&self, obj: Sphere) -> Option<Contact> {
        let mut normal = (self.points[1] - self.points[0])
            .cross(self.points[2] - self.points[0])
            .normalize();

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
            let contact_capsule_radius_sqr = obj.radius_with_threshold.powi(2);
            let min_dist_sqr = contact_capsule_radius_sqr;

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

        let (point, result_normal, depth) = if distance_sqr > f32::EPSILON {
            let distance = distance_sqr.sqrt();
            let result_normal = contact_to_center / distance;
            let point = contact_point;
            let depth = -(obj.radius - distance);
            (point, result_normal, depth)
        } else {
            let result_normal = normal;
            let point = contact_point;
            let depth = -obj.radius;
            (point, result_normal, depth)
        };

        let point_in_world = point + result_normal * depth;

        Some(Contact {
            ray: Ray::new(point, result_normal, depth),
            triangle_normal: normal,
            position: point_in_world,
            local_position: point - obj.center,
        })
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
    center: Vec3A,
    radius: f32,
    radius_with_threshold: f32,
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

    const TRI: Tri = Tri::from_points(
        Vec3A::new(-1.0, 5.0, 0.0),
        Vec3A::new(2.0, 2.0, -3.0),
        Vec3A::new(5.0, 5.0, 0.0),
    );

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
