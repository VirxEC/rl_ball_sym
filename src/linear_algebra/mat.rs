pub struct Mat3 {
    pub m: [[f32; 3]; 3],
}

impl Default for Mat3 {
    fn default() -> Self {
        Self {
            m: [[0.; 3]; 3],
        }
    }
}

impl Mat3 {
    pub fn det(&self) -> f32 {
        self.m[0][0] * self.m[1][1] * self.m[2][2] + self.m[0][1] * self.m[1][2] * self.m[2][0] + self.m[0][2] * self.m[1][0] * self.m[2][1] - self.m[0][0] * self.m[1][2] * self.m[2][1] - self.m[0][1] * self.m[1][0] * self.m[2][2] - self.m[0][2] * self.m[1][1] * self.m[2][0]
    }

    pub fn dot(&self, b: &Mat3) -> Mat3 {
        let mut c: Mat3 = Mat3::default();

        for i in 0..3 {
            for j in 0..3 {
                c.m[i][j] = 0.;
                for k in 0..3 {
                    c.m[i][j] += self.m[i][k] * b.m[k][j];
                }
            }
        }

        c
    }

    pub fn inv(&self) -> Mat3 {
        let inv_det_a = 1. / self.det();

        Self {
            m: [
                [(self.m[1][1] * self.m[2][2] - self.m[1][2] * self.m[2][1]) * inv_det_a, (self.m[0][2] * self.m[2][1] - self.m[0][1] * self.m[2][2]) * inv_det_a, (self.m[0][1] * self.m[1][2] - self.m[0][2] * self.m[1][1]) * inv_det_a],
                [(self.m[1][2] * self.m[2][0] - self.m[1][0] * self.m[2][2]) * inv_det_a, (self.m[0][0] * self.m[2][2] - self.m[0][2] * self.m[2][0]) * inv_det_a, (self.m[0][2] * self.m[1][0] - self.m[0][0] * self.m[1][2]) * inv_det_a],
                [(self.m[1][0] * self.m[2][1] - self.m[1][1] * self.m[2][0]) * inv_det_a, (self.m[0][1] * self.m[2][0] - self.m[0][0] * self.m[2][1]) * inv_det_a, (self.m[0][0] * self.m[1][1] - self.m[0][1] * self.m[1][0]) * inv_det_a],
            ],
        }
    }
}
