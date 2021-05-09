use crate::linear_algebra::vector::Vec3;
use crate::simulation::ball::Ball;
use crate::simulation::field::Field;

pub struct Game {
    pub index: u8, // supports up to 255 cars!
    pub team: u8,
    pub gravity: Vec3,
    pub field: Field,
    pub ball: Ball,
}
