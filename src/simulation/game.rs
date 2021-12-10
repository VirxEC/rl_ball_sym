use glam::Vec3A;

use super::ball::Ball;
use super::bvh::Bvh;

#[derive(Clone)]
pub struct Game {
    pub gravity: Vec3A,
    pub collision_mesh: Bvh,
    pub ball: Ball,
}

impl Default for Game {
    fn default() -> Self {
        Self {
            gravity: Vec3A::default(),
            collision_mesh: Bvh::default(),
            ball: Ball::default(),
        }
    }
}
