pub type Mat3 = [[f64; 3]; 3];

pub fn empty_mat3() -> Mat3 {
    [[0., 0., 0.], [0., 0., 0.], [0., 0., 0.]]
}

pub fn det(a: &Mat3) -> f64 {
    a[0][0] * a[1][1] * a[2][2] + a[0][1] * a[1][2] * a[2][0] + a[0][2] * a[1][0] * a[2][1] - a[0][0] * a[1][2] * a[2][1] - a[0][1] * a[1][0] * a[2][2] - a[0][2] * a[1][1] * a[2][0]
}

pub fn dot(a: &Mat3, b: &Mat3) -> Mat3 {
    let mut c: Mat3 = empty_mat3();

    for i in 0..3 {
        for j in 0..3 {
            c[i][j] = 0.;
            for k in 0..3 {
                c[i][j] += a[i][k] * b[k][j];
            }
        }
    }

    c
}
