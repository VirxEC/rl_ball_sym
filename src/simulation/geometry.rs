use crate::linear_algebra::mat::{inv, Mat3};
use crate::linear_algebra::math::dot;
use crate::linear_algebra::vector::Vec3;

pub fn distance_between(start: &Vec3, dir: &Vec3, p: &Vec3) -> f64 {
    let u = ((*p - *start).dot(dir) / dir.dot(dir)).clamp(0., 1.);
    (*start + *dir * u - *p).magnitude()
}

#[derive(Clone, Copy, Debug)]
pub struct Tri {
    pub p: [Vec3; 3],
}

impl Tri {
    pub fn center(&self) -> Vec3 {
        (self.p[0] + self.p[1] + self.p[2]) / 3.
    }

    pub fn unit_normal(&self) -> Vec3 {
        (self.p[1] - self.p[0]).cross(&(self.p[2] - self.p[0])).normalize()
    }

    pub fn intersect_sphere(&self, b: &Sphere) -> bool {
        let mut _dist = 0.;

        let e1 = self.p[1] - self.p[0];
        let e2 = self.p[2] - self.p[1];
        let e3 = self.p[0] - self.p[2];
        let n = (&e3).cross(&e1).normalize();

        let a: Mat3 = [[e1.x, -e3.x, n.x], [e1.y, -e3.y, n.y], [e1.z, -e3.z, n.z]];

        let x = dot(&inv(&a), &(b.center - self.p[0]));

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
        if 0. <= u && u <= 1. && 0. <= v && v <= 1. && 0. <= w && w <= 1. {
            _dist = z.abs();
        } else {
            _dist = b.radius + 1.;
            _dist = _dist.min(distance_between(&self.p[0], &e1, &b.center));
            _dist = _dist.min(distance_between(&self.p[1], &e2, &b.center));
            _dist = _dist.min(distance_between(&self.p[2], &e3, &b.center));
        }

        _dist <= b.radius
    }
}

impl Default for Tri {
    fn default() -> Self {
        Self {
            p: [Vec3::default(), Vec3::default(), Vec3::default()],
        }
    }
}


// AABB stands for "Axis-Aligned Bounding Boxes"
// Learn more here: https://developer.nvidia.com/blog/thinking-parallel-part-i-collision-detection-gpu/
#[derive(Clone, Copy, Debug)]
pub struct Aabb {
    pub min: Vec3,
    pub max: Vec3,
}

impl Default for Aabb {
    fn default() -> Self {
        Self {
            min: Vec3::default(),
            max: Vec3::default(),
        }
    }
}

impl Aabb {
    pub fn add(&self, b: &Aabb) -> Self {
        let min = Vec3 {
            x: self.min.x.min(b.min.x),
            y: self.min.y.min(b.min.y),
            z: self.min.z.min(b.min.z),
        };

        let max = Vec3 {
            x: self.max.x.max(b.max.x),
            y: self.max.y.max(b.max.y),
            z: self.max.z.max(b.max.z),
        };

        Self {
            min,
            max,
        }
    }

    pub fn from_tri(t: &Tri) -> Self {
        let min = Vec3 {
            x: t.p[0].x.min(t.p[1].x.min(t.p[2].x)),
            y: t.p[0].y.min(t.p[1].y.min(t.p[2].y)),
            z: t.p[0].z.min(t.p[1].z.min(t.p[2].z)),
        };

        let max = Vec3 {
            x: t.p[0].x.max(t.p[1].x.max(t.p[2].x)),
            y: t.p[0].y.max(t.p[1].y.max(t.p[2].y)),
            z: t.p[0].z.max(t.p[1].z.max(t.p[2].z)),
        };

        Self {
            min,
            max,
        }
    }

    pub fn from_sphere(s: &Sphere) -> Self {
        let min = Vec3 {
            x: s.center.x - s.radius,
            y: s.center.y - s.radius,
            z: s.center.z - s.radius,
        };

        let max = Vec3 {
            x: s.center.x + s.radius,
            y: s.center.y + s.radius,
            z: s.center.z + s.radius,
        };

        Self {
            min,
            max,
        }
    }

    pub fn intersect_self(&self, b: &Aabb) -> bool {
        (self.min.x <= b.max.x) & (self.max.x >= b.min.x) & (self.min.y <= b.max.y) & (self.max.y >= b.min.y) & (self.min.z <= b.max.z) & (self.max.z >= b.min.z)
    }

    pub fn intersect_sphere(&self, b: &Sphere) -> bool {
        let nearest = Vec3 {
            x: b.center.x.clamp(self.min.x, self.max.x),
            y: b.center.y.clamp(self.min.y, self.max.y),
            z: b.center.z.clamp(self.min.z, self.max.z),
        };

        (b.center - nearest).magnitude() <= b.radius
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Int2 {
    pub x: i32,
    pub y: i32,
}

impl Default for Int2 {
    fn default() -> Self {
        Self {
            x: 0,
            y: 0,
        }
    }
}

// endpoint is start + direction
#[derive(Clone, Copy, Debug)]
pub struct Ray {
    pub start: Vec3,
    pub direction: Vec3,
}

impl Default for Ray {
    fn default() -> Self {
        Self {
            start: Vec3::default(),
            direction: Vec3::default(),
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Sphere {
    pub center: Vec3,
    pub radius: f64,
}
