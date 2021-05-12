use crate::linear_algebra::vector::Vec3;
use crate::simulation::geometry::Aabb;

pub struct Morton {
    pub offset: Vec3,
    pub scale: Vec3,
}

impl Morton {
    // 2 ^ 21 - 1 = 2097151

    pub fn from(global_box: &Aabb) -> Morton {
        let offset = global_box.min;
        let scale = 1. / (global_box.max - offset);

        Morton {
            offset,
            scale,
        }
    }

    fn expand3(a: u32) -> u64 {
        let mut x = (a as u64) & 0x1fffff; // we only look at the first 21 bits

        x = (x | x << 16) & 0x0000ffff0000ffff;
        x = (x | x << 8) & 0x00ff00ff00ff00ff;
        x = (x | x << 4) & 0x0f0f0f0f0f0f0f0f;
        x = (x | x << 2) & 0x3333333333333333;
        x = (x | x << 1) & 0x5555555555555555;

        x
    }

    fn encode(u: Vec3) -> u64 {
        // These should actually be 21 bits, but there's no u21 type
        let x = u.x as u32;
        let y = u.y as u32;
        let z = u.z as u32;

        Morton::expand3(x) | Morton::expand3(y) << 1 | Morton::expand3(z) << 2
    }

    pub fn get_code(&self, box_: &Aabb) -> u64 {
        // get the centroid of the ith bounding box
        let c = (box_.min + box_.max) / 2.;

        let u = (c - self.offset) * self.scale;

        Morton::encode(u)
    }
}
