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

#[derive(Clone, Copy, Debug)]
pub struct Aabb {
    pub min_x: f64,
    pub min_y: f64,
    pub min_z: f64,
    pub max_x: f64,
    pub max_y: f64,
    pub max_z: f64,
}

impl Default for Aabb {
    fn default() -> Self {
        Self {
            min_x: 0.,
            min_y: 0.,
            min_z: 0.,
            max_x: 0.,
            max_y: 0.,
            max_z: 0.,
        }
    }
}

impl Aabb {
    pub fn add(&self, b: Aabb) -> Self {
        let min_x = self.min_x.min(b.min_x);
        let min_y = self.min_y.min(b.min_y);
        let min_z = self.min_z.min(b.min_z);
        let max_x = self.max_x.max(b.max_x);
        let max_y = self.max_y.max(b.max_y);
        let max_z = self.max_z.max(b.max_z);

        Self {
            min_x,
            min_y,
            min_z,
            max_x,
            max_y,
            max_z,
        }
    }

    pub fn from_tri(t: Tri) -> Self {
        let min_x = t.p[0].x.min(t.p[1].x.min(t.p[2].x));
        let min_y = t.p[0].y.min(t.p[1].y.min(t.p[2].y));
        let min_z = t.p[0].z.min(t.p[1].z.min(t.p[2].z));

        let max_x = t.p[0].x.max(t.p[1].x.max(t.p[2].x));
        let max_y = t.p[0].y.max(t.p[1].y.max(t.p[2].y));
        let max_z = t.p[0].z.max(t.p[1].z.max(t.p[2].z));

        Self {
            min_x,
            min_y,
            min_z,
            max_x,
            max_y,
            max_z,
        }
    }

    pub fn from_sphere(s: &Sphere) -> Self {
        let min_x = s.center.x - s.radius;
        let min_y = s.center.y - s.radius;
        let min_z = s.center.z - s.radius;

        let max_x = s.center.x + s.radius;
        let max_y = s.center.y + s.radius;
        let max_z = s.center.z + s.radius;

        Self {
            min_x,
            min_y,
            min_z,
            max_x,
            max_y,
            max_z,
        }
    }

    pub fn intersect_self(&self, b: &Aabb) -> bool {
        (self.min_x <= b.max_x) & (self.max_x >= b.min_x) & (self.min_y <= b.max_y) & (self.max_y >= b.min_y) & (self.min_z <= b.max_z) & (self.max_z >= b.min_z)
    }

    pub fn intersect_sphere(&self, b: &Sphere) -> bool {
        let nearest = Vec3 {
            x: b.center.x.clamp(self.min_x, self.max_x),
            y: b.center.y.clamp(self.min_y, self.max_y),
            z: b.center.z.clamp(self.min_z, self.max_z),
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
