use crate::vector::{empty_vec3, Vec3};

pub struct Tri {
    pub p: [Vec3; 3],
}

pub fn empty_tri() -> Tri {
    Tri {
        p: [empty_vec3(), empty_vec3(), empty_vec3()],
    }
}
