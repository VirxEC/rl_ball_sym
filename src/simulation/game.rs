//! All the data about the game to simulate it.

use super::bvh::Bvh;
use glam::Vec3A;

/// All of the game information.
#[derive(Clone, Debug)]
pub struct Game {
    /// The gravity of the game
    pub gravity: Vec3A,
    /// The Bvh generated from the field
    pub(crate) collision_mesh: Bvh,
}

impl Game {
    #[inline]
    pub(crate) const fn new(collision_mesh: Bvh) -> Self {
        Self {
            gravity: Vec3A::new(0., 0., -650.),
            collision_mesh,
        }
    }
}
