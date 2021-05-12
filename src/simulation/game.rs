use crate::linear_algebra::vector::Vec3;
use crate::simulation::ball::Ball;
use crate::simulation::field::Field;

#[derive(Clone)]
pub struct Game {
    pub gravity: Vec3,
    pub field: Field,
    pub ball: Ball,
}

impl Default for Game {
    fn default() -> Self {
        Self {
            gravity: Vec3::default(),
            field: Field::default(),
            ball: Ball::default()
        }
    }
}
