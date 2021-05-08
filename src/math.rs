use crate::mat::Mat3;
use crate::vector::Vec3;

pub fn dot(a: &Mat3, v: &Vec3) -> Vec3 {
    Vec3 {
        x: a[0][0] * v.x + a[0][1] * v.y + a[0][2] * v.z,
        y: a[1][0] * v.x + a[1][1] * v.y + a[1][2] * v.z,
        z: a[2][0] * v.x + a[2][1] * v.y + a[2][2] * v.z,
    }
}
