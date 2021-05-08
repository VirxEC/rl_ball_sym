use crate::vector::Vec3;

#[derive(Clone, Copy, Debug)]
pub struct Tri {
    pub p: [Vec3; 3],
}

impl Default for Tri {
    fn default() -> Tri {
        Tri {
            p: [Vec3::default(), Vec3::default(), Vec3::default()],
        }
    }
}

pub struct Aabb {
    pub min_x: f32,
    pub min_y: f32,
    pub min_z: f32,
    pub max_x: f32,
    pub max_y: f32,
    pub max_z: f32
}
