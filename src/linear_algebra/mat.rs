use glam::Mat3A;

pub(crate) trait MatrixExt {
    fn dot(&self, other: Self) -> Self;
}

impl MatrixExt for Mat3A {
    #[inline]
    fn dot(&self, other: Self) -> Self {
        // For some reason, glam does matrix multiplication in column order. So,
        // we have to transpose both matrices prior to multiplying them. Then,
        // transpose the final result.
        (self.transpose() * other.transpose()).transpose()
    }
}

#[cfg(test)]
mod test {
    use super::MatrixExt;
    use glam::Mat3A;

    const A: Mat3A = Mat3A::from_cols_array_2d(&[[-1., 2., 3.], [4., 5., 6.], [7., 8., 9.]]);
    const B: Mat3A = Mat3A::from_cols_array_2d(&[[9., 8., 7.], [6., -5., 4.], [3., 2., 1.]]);
    const C: Mat3A = Mat3A::from_cols_array_2d(&[[12.0, -12.0, 4.0], [84.0, 19.0, 54.0], [138.0, 34.0, 90.0]]);

    #[test]
    fn mat3_dot() {
        assert_eq!(A.dot(B), C);
    }
}
