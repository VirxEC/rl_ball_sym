use glam::{Mat3A, Vec3A};

/// Round a vector to a given precision
pub fn round_vec_bullet(vec: &mut Vec3A, scale: f32, precision: f32) {
    *vec = (*vec / scale / precision).round() * precision * scale;
}

#[must_use]
#[inline]
/// Convert an axis-angle vector to a rotation matrix.
pub fn axis_to_rotation(axis: Vec3A) -> Mat3A {
    Mat3A::from_axis_angle(axis.into(), axis.length())
}

#[must_use]
#[inline]
/// Returns the dot product of the matrix and the vector.
pub fn dot(matrix: Mat3A, vector: Vec3A) -> Vec3A {
    matrix.transpose() * vector
}

#[cfg(test)]
mod test {
    use super::dot;
    use glam::{Mat3A, Vec3A};

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
        assert_eq!(dot(MAT, VEC), Vec3A::new(VEC.dot(MAT.x_axis), VEC.dot(MAT.y_axis), VEC.dot(MAT.z_axis)));
    }
}
