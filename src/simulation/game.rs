use crate::simulation::field::Field;
use crate::simulation::ball::Ball;

pub struct Game {
    pub index: u8, // supports up to 255 cars!
    pub team: u8,
    pub field: Field,
    pub ball: Ball
}