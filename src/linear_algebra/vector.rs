use super::mat::Mat3;
use std::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub};

#[derive(Clone, Copy, Debug)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    pub fn dot(&self, vec: &Self) -> f32 {
        self.x * vec.x + self.y * vec.y + self.z * vec.z
    }

    pub fn magnitude(&self) -> f32 {
        self.dot(self).sqrt()
    }

    pub fn normalize(&self) -> Self {
        let size = self.magnitude();
        if size == 0. {
            return Vec3::default();
        }

        *self / size
    }

    pub fn normalized(&mut self) {
        let size = self.magnitude();
        if size != 0. {
            *self /= size;
        }
    }

    pub fn scale(&self, value: f32) -> Self {
        self.normalize() * value
    }

    pub fn cross(&self, vec: &Self) -> Self {
        Self {
            x: self.y * vec.z - self.z * vec.y,
            y: self.z * vec.x - self.x * vec.z,
            z: self.x * vec.y - self.y * vec.x,
        }
    }

    pub fn axis_to_rotation(&self) -> Mat3 {
        let norm_omega = self.magnitude();

        if norm_omega.abs() < 0.000001 {
            return Mat3::eye();
        }
        let u = *self / norm_omega;

        let c = norm_omega.cos();
        let s = norm_omega.sin();

        Mat3 {
            m: [[u.x * u.x * (1. - c) + c, u.x * u.y * (1. - c) - u.z * s, u.x * u.z * (1. - c) + u.y * s], [u.y * u.x * (1. - c) + u.z * s, u.y * u.y * (1. - c) + c, u.y * u.z * (1. - c) - u.x * s], [u.y * u.x * (1. - c) - u.y * s, u.y * u.y * (1. - c) + u.x * s, u.y * u.z * (1. - c) + c]],
        }
    }

    pub fn min(&self, vec: &Self) -> Self {
        Self {
            x: self.x.min(vec.x),
            y: self.y.min(vec.y),
            z: self.z.min(vec.z),
        }
    }

    pub fn max(&self, vec: &Self) -> Self {
        Self {
            x: self.x.max(vec.x),
            y: self.y.max(vec.y),
            z: self.z.max(vec.z),
        }
    }

    pub fn clamp(&self, min: &Self, max: &Self) -> Self {
        Self {
            x: self.x.max(min.x).min(max.x),
            y: self.y.max(min.y).min(max.y),
            z: self.z.max(min.z).min(max.z),
        }
    }
}

impl Add for Vec3 {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }
}

impl AddAssign for Vec3 {
    fn add_assign(&mut self, other: Self) {
        self.x += other.x;
        self.y += other.y;
        self.z += other.z;
    }
}

impl Sub for Vec3 {
    type Output = Self;

    fn sub(self, other: Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        }
    }
}

impl Mul<Vec3> for Vec3 {
    type Output = Self;

    fn mul(self, other: Self) -> Self {
        Self {
            x: self.x * other.x,
            y: self.y * other.y,
            z: self.z * other.z,
        }
    }
}

impl Mul<f32> for Vec3 {
    type Output = Self;

    fn mul(self, other: f32) -> Self {
        Self {
            x: self.x * other,
            y: self.y * other,
            z: self.z * other,
        }
    }
}

impl Mul<Vec3> for f32 {
    type Output = Vec3;

    fn mul(self, other: Vec3) -> Vec3 {
        Vec3 {
            x: self * other.x,
            y: self * other.y,
            z: self * other.z,
        }
    }
}

impl MulAssign<Vec3> for Vec3 {
    fn mul_assign(&mut self, other: Self) {
        self.x *= other.x;
        self.y *= other.y;
        self.z *= other.z;
    }
}

impl MulAssign<f32> for Vec3 {
    fn mul_assign(&mut self, other: f32) {
        self.x *= other;
        self.y *= other;
        self.z *= other;
    }
}

impl Div<Vec3> for Vec3 {
    type Output = Self;

    fn div(self, other: Self) -> Self {
        Self {
            x: self.x / other.x,
            y: self.y / other.y,
            z: self.z / other.z,
        }
    }
}

impl Div<f32> for Vec3 {
    type Output = Self;

    fn div(self, other: f32) -> Self {
        Self {
            x: self.x / other,
            y: self.y / other,
            z: self.z / other,
        }
    }
}

impl Div<Vec3> for f32 {
    type Output = Vec3;

    fn div(self, other: Vec3) -> Vec3 {
        Vec3 {
            x: self / other.x,
            y: self / other.y,
            z: self / other.z,
        }
    }
}

impl DivAssign<f32> for Vec3 {
    fn div_assign(&mut self, other: f32) {
        self.x /= other;
        self.y /= other;
        self.z /= other;
    }
}

impl Neg for Vec3 {
    type Output = Self;

    fn neg(self) -> Self {
        Self {
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }
}

impl Default for Vec3 {
    fn default() -> Self {
        Self {
            x: 0.,
            y: 0.,
            z: 0.,
        }
    }
}
