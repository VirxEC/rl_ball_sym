use super::bvh::Bvh;
use glam::Vec3A;

/// All of the game information.
#[derive(Clone, Debug)]
pub struct Game {
    /// The gravity of the game
    pub gravity: Vec3A,
    /// The Bvh generated from the field
    pub collision_mesh: Bvh,
}
