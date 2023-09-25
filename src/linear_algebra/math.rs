use glam::{Mat3A, Vec3, Vec3A};

pub trait Vec3AExt {
    fn round_vec_bullet(&mut self, scale: f32, precision: f32);
}

impl Vec3AExt for Vec3A {
    #[inline]
    fn round_vec_bullet(&mut self, scale: f32, precision: f32) {
        *self = (*self / scale / precision).round() * precision * scale;
    }
}

#[must_use]
#[inline]
/// Convert an axis-angle vector to a rotation matrix.
pub fn z_axis_to_rotation(axis: f32) -> Mat3A {
    Mat3A::from_axis_angle(Vec3::new(0., 0., axis), axis)
}
