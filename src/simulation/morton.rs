use super::geometry::Aabb;
use glam::Vec3A;

/// Basic data for generating morton codes.
#[derive(Clone, Copy, Debug, Default)]
pub struct Morton {
    offset: Vec3A,
    scale: Vec3A,
}

impl Morton {
    /// Calculate basic information required to generate a morton code.
    #[must_use]
    pub fn from(global_box: &Aabb) -> Self {
        let offset = global_box.min();
        // 2 ^ 20 - 1 = 1048575
        let scale = 1_048_575. / (global_box.max() - offset);

        Self { offset, scale }
    }

    /// Get the vector offset.
    #[must_use]
    pub const fn offset(&self) -> Vec3A {
        self.offset
    }

    /// Get the vector scale.
    #[must_use]
    pub const fn scale(&self) -> Vec3A {
        self.scale
    }

    /// Prepare a 21-bit unsigned int for inverweaving.
    #[must_use]
    pub fn expand3(a: u32) -> u64 {
        let mut x = u64::from(a) & 0x001f_ffff; // we only look at the first 21 bits

        x = (x | x << 32) & 0x001f_0000_0000_ffff;
        x = (x | x << 16) & 0x001f_0000_ff00_00ff;
        x = (x | x << 8) & 0x100f_00f0_0f00_f00f;
        x = (x | x << 4) & 0x10c3_0c30_c30c_30c3;
        x = (x | x << 2) & 0x1249_2492_4924_9249;

        x
    }

    fn encode(u: Vec3A) -> u64 {
        // These should actually be 21 bits, but there's no u21 type and the final type is u64 (21 bits * 3 = 63 bits)
        let x = u.x as u32;
        let y = u.y as u32;
        let z = u.z as u32;

        Self::expand3(x) | Self::expand3(y) << 1 | Self::expand3(z) << 2
    }

    /// Get an AABB's morton code.
    #[must_use]
    pub fn get_code(&self, box_: &Aabb) -> u64 {
        // get the centroid of the ith bounding box
        let c = (box_.min() + box_.max()) / 2.;

        let u = (c - self.offset) * self.scale;

        Self::encode(u)
    }
}
