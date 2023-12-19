//! All the data about the game to simulate it.

use super::tri_bvh::TriangleBvh;
use glam::Vec3A;

/// All of the game information.
#[derive(Clone, Debug)]
pub struct Game {
    /// The gravity of the game
    pub gravity: Vec3A,
    /// The Bvh generated from the field
    pub(crate) triangle_collisions: TriangleBvh,
}

impl Game {
    #[inline]
    pub(crate) const fn new(triangle_collisions: TriangleBvh) -> Self {
        Self {
            gravity: Vec3A::new(0., 0., -650.),
            triangle_collisions,
        }
    }
}
