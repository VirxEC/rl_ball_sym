use glam::Vec3A;

use super::ball::Ball;
use super::bvh::Bvh;

/// All of the game information.
#[derive(Clone, Debug, Default)]
pub struct Game {
    /// The gravity of the game
    pub gravity: Vec3A,
    /// The Bvh generated from the field
    pub collision_mesh: Bvh,
    /// The game's ball
    pub ball: Ball,
}
