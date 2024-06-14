//! Various geometrical objects and tools.

use crate::simulation::game::Constraints;
use combo_vec::ReArr;
use glam::Vec3A;
use std::ops::Add;

#[cfg(feature = "heatseeker")]
use glam::Vec3Swizzles;
#[cfg(feature = "heatseeker")]
use std::f32::consts::{FRAC_PI_2, PI};

#[cfg(feature = "heatseeker")]
fn wrap_normalize_float(val: f32, minmax: f32) -> f32 {
    let r = val % (minmax * 2.);

    if r > minmax {
        r - minmax * 2.
    } else if r < -minmax {
        r + minmax * 2.
    } else {
        r
    }
}

#[derive(Clone, Copy, Debug, Default)]
#[cfg(feature = "heatseeker")]
pub struct Angle {
    pub yaw: f32,
    pub pitch: f32,
}

#[cfg(feature = "heatseeker")]
impl Angle {
    #[must_use]
    pub fn from_vec(forward: Vec3A) -> Self {
        Self {
            yaw: forward.y.atan2(forward.x),
            pitch: forward.z.atan2(forward.xy().length()),
        }
    }

    #[must_use]
    pub fn get_forward_vec(self) -> Vec3A {
        let (sp, cp) = self.pitch.sin_cos();
        let (sy, cy) = self.yaw.sin_cos();

        Vec3A::new(cp * cy, cp * sy, sp)
    }

    pub fn normalize_fix(&mut self) {
        self.yaw = wrap_normalize_float(self.yaw, PI);
        self.pitch = wrap_normalize_float(self.pitch, FRAC_PI_2);
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Contact {
    pub local_position: Vec3A,
    pub triangle_normal: Vec3A,
    pub depth: f32,
}

#[derive(Debug)]
#[repr(transparent)]
pub struct Hits(ReArr<Contact, { Constraints::MAX_CONTACTS }>);

impl Hits {
    #[inline]
    pub const fn new() -> Self {
        Self(ReArr::new())
    }

    #[inline]
    fn get_res(new_contact_local: Vec3A, point1: Vec3A, point2: Vec3A, point3: Vec3A) -> f32 {
        (new_contact_local - point1).cross(point2 - point3).length_squared()
    }

    #[inline]
    fn get_res_0(&self, new_contact_local: Vec3A) -> f32 {
        Self::get_res(
            new_contact_local,
            self.0[1].local_position,
            self.0[3].local_position,
            self.0[2].local_position,
        )
    }

    #[inline]
    fn get_res_1(&self, new_contact_local: Vec3A) -> f32 {
        Self::get_res(
            new_contact_local,
            self.0[0].local_position,
            self.0[3].local_position,
            self.0[2].local_position,
        )
    }

    #[inline]
    fn get_res_2(&self, new_contact_local: Vec3A) -> f32 {
        Self::get_res(
            new_contact_local,
            self.0[0].local_position,
            self.0[3].local_position,
            self.0[1].local_position,
        )
    }

    #[inline]
    fn get_res_3(&self, new_contact_local: Vec3A) -> f32 {
        Self::get_res(
            new_contact_local,
            self.0[0].local_position,
            self.0[2].local_position,
            self.0[1].local_position,
        )
    }

    // sortCachedPoints
    fn replacement_index(&self, new_contact: &Contact) -> usize {
        let mut max_penetration_index = self.0.len();
        let mut max_penetration = new_contact.depth;
        for (i, contact) in self.0.iter().enumerate() {
            if contact.depth < max_penetration {
                max_penetration_index = i;
                max_penetration = contact.depth;
            }
        }

        let res = match max_penetration_index {
            0 => [
                0.,
                self.get_res_1(new_contact.local_position),
                self.get_res_2(new_contact.local_position),
                self.get_res_3(new_contact.local_position),
            ],
            1 => [
                self.get_res_0(new_contact.local_position),
                0.,
                self.get_res_2(new_contact.local_position),
                self.get_res_3(new_contact.local_position),
            ],
            2 => [
                self.get_res_0(new_contact.local_position),
                self.get_res_1(new_contact.local_position),
                0.,
                self.get_res_3(new_contact.local_position),
            ],
            3 => [
                self.get_res_0(new_contact.local_position),
                self.get_res_1(new_contact.local_position),
                self.get_res_2(new_contact.local_position),
                0.,
            ],
            _ => [
                self.get_res_0(new_contact.local_position),
                self.get_res_1(new_contact.local_position),
                self.get_res_2(new_contact.local_position),
                self.get_res_3(new_contact.local_position),
            ],
        };

        let (mut biggest_area, mut biggest_area_index) = if res[1] > res[0] { (res[1], 1) } else { (res[0], 0) };

        if res[2] > biggest_area {
            biggest_area = res[2];
            biggest_area_index = 2;
        }

        if res[3] > biggest_area {
            biggest_area_index = 3;
        }

        biggest_area_index
    }

    // addManifoldPoint
    pub fn push(&mut self, contact: Contact) {
        if self.0.len() == Constraints::MAX_CONTACTS {
            let index = self.replacement_index(&contact);
            self.0[index] = contact;
        } else {
            self.0.push(contact);
        }
    }

    #[inline]
    pub const fn inner(self) -> ReArr<Contact, { Constraints::MAX_CONTACTS }> {
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
    /// Create a new triangle from 3 points.
    pub const fn new(points: [Vec3A; 3]) -> Self {
        Self { points }
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
    fn face_contains(u: Vec3A, v: Vec3A, n: Vec3A, w: Vec3A) -> bool {
        let dot_n = n.dot(n);

        let gamma = u.cross(w).dot(n) / dot_n;
        let beta = w.cross(v).dot(n) / dot_n;
        let alpha = 1. - gamma - beta;

        (gamma >= 0. && beta >= 0. && alpha >= 0.) && (gamma <= 1. && beta <= 1. && alpha <= 1.)
    }

    /// Instead of using bullet's method,
    /// we use the method described here which is much faster:
    /// <https://stackoverflow.com/a/74395029/10930209>
    fn closest_point(&self, point: Vec3A, ab: Vec3A, ac: Vec3A, ap: Vec3A) -> Vec3A {
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
        let u = self.points[1] - self.points[0];
        let v = self.points[2] - self.points[0];
        let n = u.cross(v);
        let mut triangle_normal = n.normalize();

        let p1_to_center = obj.center - self.points[0];
        let mut distance_from_plane = p1_to_center.dot(triangle_normal);

        if distance_from_plane < 0. {
            distance_from_plane *= -1.;
            triangle_normal *= -1.;
        }

        if distance_from_plane >= obj.radius_with_threshold {
            return None;
        }

        let w = obj.center - self.points[0];

        let contact_point = if Self::face_contains(u, v, n, w) {
            obj.center - triangle_normal * distance_from_plane
        } else {
            let min_dist_sqr = obj.radius_with_threshold.powi(2);
            let closest_point = self.closest_point(obj.center, u, v, w);
            let distance_sqr = (closest_point - obj.center).length_squared();

            if distance_sqr < min_dist_sqr {
                closest_point
            } else {
                return None;
            }
        };

        let contact_to_center = obj.center - contact_point;
        let distance_sqr = contact_to_center.length_squared();

        if distance_sqr >= obj.radius_with_threshold.powi(2) {
            return None;
        }

        let (result_normal, depth) = if distance_sqr > f32::EPSILON {
            let distance = distance_sqr.sqrt();
            (contact_to_center / distance, -(obj.radius - distance))
        } else {
            (triangle_normal, -obj.radius)
        };

        // println!("CONTACT");

        let point_in_world = contact_point + result_normal * depth;

        Some(Contact {
            local_position: point_in_world - obj.center,
            triangle_normal,
            depth,
        })
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
    #[inline]
    #[must_use]
    #[cfg(test)]
    /// Create a new AABB.
    ///
    /// Used in tests.
    pub const fn new(min: Vec3A, max: Vec3A) -> Self {
        Self { min, max }
    }

    #[inline]
    #[must_use]
    /// The minimum point contained in the AABB.
    pub const fn min(self) -> Vec3A {
        self.min
    }

    #[inline]
    #[must_use]
    /// The maximum point contained in the AABB.
    pub const fn max(self) -> Vec3A {
        self.max
    }

    #[inline]
    #[must_use]
    /// Create an AABB from a triangle.
    pub fn from_tri(t: Tri) -> Self {
        Self {
            min: t.points[0].min(t.points[1]).min(t.points[2]),
            max: t.points[0].max(t.points[1]).max(t.points[2]),
        }
    }

    #[inline]
    #[must_use]
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
