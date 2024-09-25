//! Tools for calculating morton codes and Morton codes for 3D points.

use super::geometry::Aabb;
use glam::Vec3A;

/// Basic data for generating morton codes.
#[derive(Debug)]
pub struct Morton {
    offset: Vec3A,
    scale: Vec3A,
}

impl Morton {
    /// Calculate basic information required to generate a morton code.
    #[must_use]
    pub fn new(global_aabb: Aabb) -> Self {
        let offset = global_aabb.min();
        // 2 ^ 20 - 1 = 1048575
        let scale = 1_048_575. / (global_aabb.max() - offset);

        Self { offset, scale }
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

    /// Get an AABB's morton code.
    #[must_use]
    pub fn get_code(&self, aabb: &Aabb) -> u64 {
        // get the centroid of the ith bounding box
        let c = (aabb.min() + aabb.max()) / 2.;

        let u = (c - self.offset) * self.scale;

        debug_assert!(u.x >= 0.);
        debug_assert!(u.y >= 0.);
        debug_assert!(u.z >= 0.);

        // These should actually be 21 bits, but there's no u21 type and the final type is u64 (21 bits * 3 = 63 bits)
        // Allowing these warnings is ok because:
        // We have offset the values so they're all greater than 0
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        (Self::expand3(u.x as u32)
            | Self::expand3(u.y as u32) << 1
            | Self::expand3(u.z as u32) << 2)
    }
}

#[cfg(test)]
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
mod test {
    use super::Morton;
    use crate::simulation::geometry::Aabb;
    use glam::Vec3A;

    #[test]
    fn morton() {
        let global_box = Aabb::new(
            Vec3A::new(-4096., -5120., 0.),
            Vec3A::new(4096., 5120., 2044.),
        );

        let morton = Morton::new(global_box);

        let aabb = Aabb::new(
            Vec3A::new(-4095., -5119., 1.),
            Vec3A::new(-4094., -5118., 2.),
        );

        let c = (aabb.min() + aabb.max()) / 2.;

        let u = (c - morton.offset) * morton.scale;
        dbg!(&u);

        let code_x = Morton::expand3(u.x as u32);
        dbg!(code_x);
        dbg!(format!("{:b}", u.x as u32));
        dbg!(format!("{code_x:b}"));
        assert_eq!(format!("{code_x:b}"), "1000001001001001001001"); // 001000001001001001001001

        let code_y = Morton::expand3(u.y as u32) << 1;
        dbg!(code_y);
        dbg!(format!("{:b}", u.y as u32));
        dbg!(format!("{code_y:b}"));
        assert_eq!(format!("{code_y:b}"), "10000000010010000000010"); // 010000000010010000000010

        let code_z = Morton::expand3(u.z as u32) << 2;
        dbg!(code_z);
        dbg!(format!("{:b}", u.z as u32));
        dbg!(format!("{code_z:b}"));
        assert_eq!(format!("{code_z:b}"), "100100000000000000000000000100");

        let code = code_z | code_y | code_x;
        dbg!(&code); // 610317903
        assert_eq!(
            format!("{:b}", code as u32),
            "100100011000001011011001001111"
        );
    }
}
