use glam::Vec3A;

use super::ball::Ball;
use super::bvh::Bvh;

#[derive(Clone, Default)]
pub struct Game {
    pub gravity: Vec3A,
    pub collision_mesh: Bvh,
    pub ball: Ball,
}
