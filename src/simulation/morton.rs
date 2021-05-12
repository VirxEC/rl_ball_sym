use crate::linear_algebra::vector::Vec3;
use crate::simulation::bit_packing::bits_needed;
use crate::simulation::geometry::Aabb;

pub struct Morton {
    pub offset: Vec3,
    pub scale: Vec3,
}

impl Morton {
    const DIM: u32 = 3;

    pub fn from(global_box: &Aabb, num_boxes: u32) -> Morton {
        let offset = global_box.min;

        // let b = bits_needed(num_boxes) as i32;
        // let bits_per_dimension = (64 - b) / (Morton::DIM as i32);
        // let divisions_per_dimension = 1 << bits_per_dimension;

        // let scale = ((divisions_per_dimension - 1) as f32) / (global_box.max - global_box.min);
        let scale = 1. / (global_box.max - global_box.min);

        Morton {
            offset,
            scale,
        }
    }

    fn expand3(a: u32) -> u64 {
        let mut x = (a as u64) & 0x1fffff; // we only look at the first 21 bits

        x = (x | x << 32) & 0x1f00000000ffff; // shift left 32 bits, OR with self, and 00011111000000000000000000000000000000001111111111111111
        x = (x | x << 16) & 0x1f0000ff0000ff; // shift left 32 bits, OR with self, and 00011111000000000000000011111111000000000000000011111111
        x = (x | x << 8) & 0x100f00f00f00f00f; // shift left 32 bits, OR with self, and 0001000000001111000000001111000000001111000000001111000000000000
        x = (x | x << 4) & 0x10c30c30c30c30c3; // shift left 32 bits, OR with self, and 0001000011000011000011000011000011000011000011000011000100000000
        x = (x | x << 2) & 0x1249249249249249;

        x
    }

    fn encode(x: u32, y: u32, z: u32) -> u64 {
        0 | Morton::expand3(x) | (Morton::expand3(y) << 1) | (Morton::expand3(z) << 2)
    }

    pub fn get_code(&self, box_: &Aabb) -> u64 {
        // get the centroid of the ith bounding box
        let c = (box_.min + box_.max) * 0.5;

        let ux = ((c.x - self.offset.x) * self.scale.x) as u32;
        let uy = ((c.y - self.offset.y) * self.scale.y) as u32;
        let uz = ((c.z - self.offset.z) * self.scale.z) as u32;

        Morton::encode(ux, uy, uz)
    }
}
