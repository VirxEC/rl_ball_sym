use glam::{Mat3A, Vec3A};

pub fn axis_to_rotation(axis: Vec3A) -> Mat3A {
    let angle = axis.length();

    Mat3A::from_axis_angle(axis.into(), angle)
}

pub fn dot(matrix: Mat3A, vector: Vec3A) -> Vec3A {
    matrix.transpose().mul_vec3a(vector)
}

#[cfg(test)]
mod test {
    use glam::{const_mat3a, const_vec3a, Mat3A, Vec3A};

    use crate::linear_algebra::math::dot;

    #[allow(clippy::approx_constant)]
    const MAT: Mat3A = const_mat3a!([-0.0, -0.16666667, -0.16666667], [0.16666667, 0.083333336, 0.083333336], [0.0, -0.7071068, 0.7071068]);
    const VEC: Vec3A = const_vec3a!([3., -1., -1.]);
    const RES: Vec3A = const_vec3a!([0.33333334, 0.3333333, 0.0]);

    #[test]
    fn test_dot() {
        assert_eq!(dot(MAT, VEC), RES);
    }
}
