use glam::Vec3A;

use super::geometry::Aabb;

/// Basic data for generating morton codes.
#[derive(Clone, Copy, Debug, Default)]
pub struct Morton {
    offset: Vec3A,
    scale: Vec3A,
}

impl Morton {
    /// Calculate basic information required to generate a morton code.
    pub fn from(global_box: &Aabb) -> Morton {
        let offset = global_box.min();
        // 2 ^ 20 - 1 = 1048575
        let scale = 1048575. / (global_box.max() - offset);

        Morton {
            offset,
            scale,
        }
    }

    /// Get the vector offset.
    pub const fn offset(&self) -> Vec3A {
        self.offset
    }

    /// Get the vector scale.
    pub const fn scale(&self) -> Vec3A {
        self.scale
    }

    /// Prepare a 21-bit unsigned int for inverweaving.
    pub fn expand3(a: u32) -> u64 {
        let mut x = (a as u64) & 0x1fffff; // we only look at the first 21 bits

        x = (x | x << 32) & 0x1f00000000ffff;
        x = (x | x << 16) & 0x1f0000ff0000ff;
        x = (x | x << 8) & 0x100f00f00f00f00f;
        x = (x | x << 4) & 0x10c30c30c30c30c3;
        x = (x | x << 2) & 0x1249249249249249;

        x
    }

    fn encode(u: Vec3A) -> u64 {
        // These should actually be 21 bits, but there's no u21 type and the final type is u64 (21 bits * 3 = 63 bits)
        let x = u.x as u32;
        let y = u.y as u32;
        let z = u.z as u32;

        Morton::expand3(x) | Morton::expand3(y) << 1 | Morton::expand3(z) << 2
    }

    /// Get an AABB's morton code.
    pub fn get_code(&self, box_: &Aabb) -> u64 {
        // get the centroid of the ith bounding box
        let c = (box_.min() + box_.max()) / 2.;

        let u = (c - self.offset) * self.scale;

        Morton::encode(u)
    }
}
