use crate::linear_algebra::mat::Mat3;
use crate::linear_algebra::vector::Vec3;

pub fn dot(a: &Mat3, v: &Vec3) -> Vec3 {
    Vec3 {
        x: a.m[0][0] * v.x + a.m[0][1] * v.y + a.m[0][2] * v.z,
        y: a.m[1][0] * v.x + a.m[1][1] * v.y + a.m[1][2] * v.z,
        z: a.m[2][0] * v.x + a.m[2][1] * v.y + a.m[2][2] * v.z,
    }
}
