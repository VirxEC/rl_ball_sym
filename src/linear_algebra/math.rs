use glam::{Mat3A, Vec3A};
use std::ops::{Add, Mul, Sub};

#[must_use]
pub fn axis_to_rotation(axis: Vec3A) -> Mat3A {
    let angle = axis.length();

    Mat3A::from_axis_angle(axis.into(), angle)
}

/// Returns the dot product of the matrix and the vector.
#[must_use]
pub fn dot(matrix: Mat3A, vector: Vec3A) -> Vec3A {
    matrix.transpose().mul_vec3a(vector)
}

#[must_use]
pub fn antisym(w: Vec3A) -> Mat3A {
    Mat3A::from_cols(Vec3A::new(0., -w[2], w[1]), Vec3A::new(w[2], 0., -w[0]), Vec3A::new(-w[1], w[0], 0.))
}

/// Returns the dot product of the vector and the matrix
#[must_use]
pub fn local(vector: Vec3A, matrix: Mat3A) -> Vec3A {
    Vec3A::new(vector.dot(matrix.x_axis), vector.dot(matrix.y_axis), vector.dot(matrix.z_axis))
}

/// Returns the value t% between a and b;
/// 0 = a, 1 = b
#[must_use]
pub fn lerp<T: Add<Output = T> + Sub<f32> + Mul<f32, Output = T>>(a: T, b: T, t: f32) -> T {
    a * (1. - t) + b * t
}

#[cfg(test)]
mod test {
    use glam::{Mat3A, Vec3A};

    use crate::linear_algebra::math::dot;

    #[allow(clippy::approx_constant)]
    const MAT: Mat3A = Mat3A::from_cols_array_2d(&[
        [-0.0, -0.166_666_67, -0.166_666_67],
        [0.166_666_67, 0.083_333_336, 0.083_333_336],
        [0.0, -0.707_106_8, 0.707_106_8],
    ]);
    const VEC: Vec3A = Vec3A::new(3., -1., -1.);
    const RES: Vec3A = Vec3A::new(0.333_333_34, 0.333_333_3, 0.0);

    #[test]
    fn test_dot() {
        assert_eq!(dot(MAT, VEC), RES);
    }
}
