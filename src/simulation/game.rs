use crate::linear_algebra::vector::Vec3;
use super::ball::Ball;
use super::bvh::Bvh;

#[derive(Clone)]
pub struct Game {
    pub gravity: Vec3,
    pub collision_mesh: Bvh,
    pub ball: Ball,
}

impl Default for Game {
    fn default() -> Self {
        Self {
            gravity: Vec3::default(),
            collision_mesh: Bvh::default(),
            ball: Ball::default(),
        }
    }
}
