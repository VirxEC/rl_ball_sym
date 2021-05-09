use crate::linear_algebra::vector::Vec3;

#[derive(Clone, Copy, Debug)]
pub struct Tri {
    pub p: [Vec3; 3],
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
    pub min_x: f32,
    pub min_y: f32,
    pub min_z: f32,
    pub max_x: f32,
    pub max_y: f32,
    pub max_z: f32,
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

    pub fn from(t: Tri) -> Self {
        let min_x = t.p[0].x.min(t.p[1].x.min(t.p[2].x)) as f32;
        let min_y = t.p[0].y.min(t.p[1].y.min(t.p[2].y)) as f32;
        let min_z = t.p[0].z.min(t.p[1].z.min(t.p[2].z)) as f32;

        let max_x = t.p[0].x.max(t.p[1].x.max(t.p[2].x)) as f32;
        let max_y = t.p[0].y.max(t.p[1].y.max(t.p[2].y)) as f32;
        let max_z = t.p[0].z.max(t.p[1].z.max(t.p[2].z)) as f32;

        Self {
            min_x,
            min_y,
            min_z,
            max_x,
            max_y,
            max_z,
        }
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
