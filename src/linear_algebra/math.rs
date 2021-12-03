use super::mat::Mat3;
use vvec3::Vec3;

pub fn axis_to_rotation(vec: Vec3) -> Mat3 {
    let norm_omega = vec.magnitude();

    if norm_omega.abs() < 0.000001 {
        return Mat3::eye();
    }
    let u = vec / norm_omega;

    let c = norm_omega.cos();
    let s = norm_omega.sin();

    Mat3 {
        m: [[u.x * u.x * (1. - c) + c, u.x * u.y * (1. - c) - u.z * s, u.x * u.z * (1. - c) + u.y * s], [u.y * u.x * (1. - c) + u.z * s, u.y * u.y * (1. - c) + c, u.y * u.z * (1. - c) - u.x * s], [u.y * u.x * (1. - c) - u.y * s, u.y * u.y * (1. - c) + u.x * s, u.y * u.z * (1. - c) + c]],
    }
}

pub fn dot(a: Mat3, v: Vec3) -> Vec3 {
    Vec3::new(a.m[0][0] * v.x + a.m[0][1] * v.y + a.m[0][2] * v.z, a.m[1][0] * v.x + a.m[1][1] * v.y + a.m[1][2] * v.z, a.m[2][0] * v.x + a.m[2][1] * v.y + a.m[2][2] * v.z)
}
