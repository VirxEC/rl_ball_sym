use glam::Vec3A;

use super::geometry::Aabb;

pub struct Morton {
    pub offset: Vec3A,
    pub scale: Vec3A,
}

impl Morton {
    // 2 ^ 21 - 1 = 2097151
    // 2 ^ 20 - 1 = 1048575

    pub fn from(global_box: &Aabb) -> Morton {
        let offset = global_box.min;
        let scale = 1048575. / (global_box.max - offset);

        Morton {
            offset,
            scale,
        }
    }

    pub fn expand3(a: u32) -> u64 {
        let mut x = (a as u64) & 0x1fffff; // we only look at the first 21 bits

        x = (x | x << 32) & 0x1f00000000ffff;
        x = (x | x << 16) & 0x1f0000ff0000ff;
        x = (x | x << 8) & 0x100f00f00f00f00f;
        x = (x | x << 4) & 0x10c30c30c30c30c3;
        x = (x | x << 2) & 0x1249249249249249;

        x
    }

    pub fn encode(u: Vec3A) -> u64 {
        // These should actually be 21 bits, but there's no u21 type and the final type is u64 (21 bits * 3 = 63 bits)
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
