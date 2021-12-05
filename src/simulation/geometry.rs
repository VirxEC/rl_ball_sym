use crate::linear_algebra::mat::Mat3;
use crate::linear_algebra::math::dot;
use vvec3::Vec3;

pub fn distance_between(start: &Vec3, dir: &Vec3, p: &Vec3) -> f32 {
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

    #[allow(clippy::many_single_char_names)]
    pub fn intersect_sphere(&self, b: &Sphere) -> bool {
        let e1 = self.p[1] - self.p[0];
        let e2 = self.p[2] - self.p[1];
        let e3 = self.p[0] - self.p[2];
        let n = (&e3).cross(&e1).normalize();

        let a = Mat3 {
            m: [[e1.x, -e3.x, n.x], [e1.y, -e3.y, n.y], [e1.z, -e3.z, n.z]],
        };

        let x = dot(a.inv(), b.center - self.p[0]);

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
            (b.radius + 1.).min(distance_between(&self.p[0], &e1, &b.center)).min(distance_between(&self.p[1], &e2, &b.center)).min(distance_between(&self.p[2], &e3, &b.center))
        };

        dist <= b.radius
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
        Self {
            min: self.min.min(&b.min),
            max: self.max.max(&b.max),
        }
    }

    pub fn from_tri(t: &Tri) -> Self {
        let min = Vec3::new(t.p[0].x.min(t.p[1].x.min(t.p[2].x)), t.p[0].y.min(t.p[1].y.min(t.p[2].y)), t.p[0].z.min(t.p[1].z.min(t.p[2].z)));
        let max = Vec3::new(t.p[0].x.max(t.p[1].x.max(t.p[2].x)), t.p[0].y.max(t.p[1].y.max(t.p[2].y)), t.p[0].z.max(t.p[1].z.max(t.p[2].z)));

        Self {
            min,
            max,
        }
    }

    pub fn from_sphere(s: &Sphere) -> Self {
        let radius = Vec3::new(s.radius, s.radius, s.radius);

        Self {
            min: s.center - radius,
            max: s.center + radius,
        }
    }

    pub fn intersect_self(&self, b: &Aabb) -> bool {
        (self.min.x <= b.max.x) & (self.max.x >= b.min.x) & (self.min.y <= b.max.y) & (self.max.y >= b.min.y) & (self.min.z <= b.max.z) & (self.max.z >= b.min.z)
    }

    pub fn intersect_sphere(&self, b: &Sphere) -> bool {
        let nearest = Vec3::new(b.center.x.clamp(self.min.x, self.max.x), b.center.y.clamp(self.min.y, self.max.y), b.center.z.clamp(self.min.z, self.max.z));

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
    pub radius: f32,
}

#[cfg(test)]
mod test {
    use super::*;

    fn generate_tri() -> Tri {
        Tri {
            p: [Vec3::new(-1.0, 5.0, 0.0), Vec3::new(2.0, 2.0, -3.0), Vec3::new(5.0, 5.0, 0.0)],
        }
    }

    fn generate_sphere() -> Sphere {
        Sphere {
            center: Vec3::new(1.0, 0.0, 1.0),
            radius: 2.0,
        }
    }

    fn generate_bounding_boxes() -> Vec<Aabb> {
        vec![
            Aabb {
                min: Vec3::new(-0.5, -2.0, -0.5),
                max: Vec3::new(0.5, 2.0, 0.5),
            },
            Aabb {
                min: Vec3::new(-1.0, -1.0, -1.0),
                max: Vec3::new(1.0, 1.0, 1.0),
            },
            Aabb {
                min: Vec3::new(1.0, 1.0, 1.0),
                max: Vec3::new(3.0, 3.0, 3.0),
            },
            Aabb {
                min: Vec3::new(-4.0, -4.0, -4.0),
                max: Vec3::new(-3.0, -3.0, -3.0),
            },
        ]
    }

    #[test]
    fn tri_sphere_intersect() {
        let tri = generate_tri();
        {
            let sphere = Sphere {
                center: Vec3::new(2.0, 4.0, -1.0),
                radius: 0.5,
            };

            assert!(tri.intersect_sphere(&sphere));
        }
        {
            let sphere = Sphere {
                center: Vec3::new(-1.0, 5.0, 0.0),
                radius: 0.5,
            };

            assert!(tri.intersect_sphere(&sphere));
        }
    }

    #[test]
    fn tri_sphere_not_intersect() {
        let tri = generate_tri();
        {
            let sphere = Sphere {
                center: Vec3::new(2.0, 2.0, 2.0),
                radius: 1.0,
            };

            assert!(!tri.intersect_sphere(&sphere));
        }
        {
            let sphere = Sphere {
                center: Vec3::new(-2.0, -2.0, -2.0),
                radius: 1.0,
            };

            assert!(!tri.intersect_sphere(&sphere));
        }
    }

    #[test]
    fn aabb_sphere_intersect() {
        let sphere = generate_sphere();

        {
            let aabb = Aabb {
                min: Vec3::new(2.0, 1.0, 2.0),
                max: Vec3::new(4.0, 3.0, 4.0),
            };

            assert!(aabb.intersect_sphere(&sphere));
        }
        {
            let aabb = Aabb {
                min: Vec3::new(0.0, -1.0, 0.0),
                max: Vec3::new(1.0, 0.0, 1.0),
            };

            assert!(aabb.intersect_sphere(&sphere));
        }
        {
            let aabb = Aabb {
                min: Vec3::new(-5.0, -5.0, -5.0),
                max: Vec3::new(5.0, 5.0, 5.0),
            };

            assert!(aabb.intersect_sphere(&sphere));
        }
    }

    #[test]
    fn aabb_sphere_not_intersect() {
        let sphere = generate_sphere();

        let aabb = Aabb {
            min: Vec3::new(-2.0, -2.0, -2.0),
            max: Vec3::new(-1.0, -1.0, -1.0),
        };

        assert!(!aabb.intersect_sphere(&sphere));
    }

    #[test]
    fn aabb_aabb_intersect() {
        let bounding_boxes = generate_bounding_boxes();

        // Test for intersection with itself
        assert!(bounding_boxes[0].intersect_self(&bounding_boxes[0]));
        assert!(bounding_boxes[1].intersect_self(&bounding_boxes[1]));
        assert!(bounding_boxes[2].intersect_self(&bounding_boxes[2]));
        assert!(bounding_boxes[3].intersect_self(&bounding_boxes[3]));

        // Test for intersection with other bounding boxes
        assert!(bounding_boxes[0].intersect_self(&bounding_boxes[1]));
        assert!(bounding_boxes[1].intersect_self(&bounding_boxes[0]));
        assert!(bounding_boxes[1].intersect_self(&bounding_boxes[2]));
        assert!(bounding_boxes[2].intersect_self(&bounding_boxes[1]));
    }

    #[test]
    fn aabb_aabb_not_intersect() {
        let bounding_boxes = generate_bounding_boxes();

        assert!(!bounding_boxes[0].intersect_self(&bounding_boxes[2]));
        assert!(!bounding_boxes[0].intersect_self(&bounding_boxes[3]));
        assert!(!bounding_boxes[1].intersect_self(&bounding_boxes[3]));
        assert!(!bounding_boxes[2].intersect_self(&bounding_boxes[0]));
        assert!(!bounding_boxes[2].intersect_self(&bounding_boxes[3]));
        assert!(!bounding_boxes[3].intersect_self(&bounding_boxes[0]));
        assert!(!bounding_boxes[3].intersect_self(&bounding_boxes[1]));
        assert!(!bounding_boxes[3].intersect_self(&bounding_boxes[2]));
    }
}
