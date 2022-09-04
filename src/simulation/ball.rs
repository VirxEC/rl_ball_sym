//! Tools for simulation a Rocket League ball.

use crate::simulation::{game::Game, geometry::Sphere};
use glam::Vec3A;

/// Represents the game's ball
#[derive(Clone, Copy, Debug, Default)]
pub struct Ball {
    /// Game time of the ball
    pub time: f32,
    /// Position of the ball
    pub location: Vec3A,
    /// Linear velocity of the ball
    pub velocity: Vec3A,
    /// Rotational velocity of the ball
    pub angular_velocity: Vec3A,
    /// Size of the ball
    pub radius: f32,
    /// Size of the ball for collisions
    pub collision_radius: f32,
    /// Momemnt of inertia of the ball
    pub moi: f32,
}

/// Collection of Balls representing future predictions based on field geometry
pub type BallPrediction = Vec<Ball>;

impl Ball {
    const RESTITUTION: f32 = 0.6;
    const DRAG: f32 = -0.0305;
    const MU: f32 = 2.;

    const V_MAX: f32 = 6000.;
    const W_MAX: f32 = 6.;

    const M: f32 = 30.;

    const SOCCAR_RADIUS: f32 = 91.25;
    const HOOPS_RADIUS: f32 = 91.25;
    const DROPSHOT_RADIUS: f32 = 100.45;
    const SOCCAR_COLLISION_RADIUS: f32 = 93.15;
    const HOOPS_COLLISION_RADIUS: f32 = 93.15;
    const DROPSHOT_COLLISION_RADIUS: f32 = 103.6;

    const INV_M: f32 = 1. / 30.;
    const RESTITUTION_M: f32 = -(1. + Self::RESTITUTION) * Self::M;

    const SIMULATION_DT: f32 = 1. / 120.;
    const STANDARD_NUM_SLICES: usize = 720;

    /// Sets the default values for a soccar ball
    #[must_use]
    pub fn initialize_soccar() -> Self {
        let mut ball = Self {
            radius: Self::SOCCAR_RADIUS,
            collision_radius: Self::SOCCAR_COLLISION_RADIUS,
            ..Default::default()
        };

        ball.initialize();

        ball
    }

    /// Sets the default values for a hoops ball
    #[must_use]
    pub fn initialize_hoops() -> Self {
        let mut ball = Self {
            radius: Self::HOOPS_RADIUS,
            collision_radius: Self::HOOPS_COLLISION_RADIUS,
            ..Default::default()
        };

        ball.initialize();

        ball
    }

    /// Sets the default values for a dropshot ball
    #[must_use]
    pub fn initialize_dropshot() -> Self {
        let mut ball = Self {
            radius: Self::DROPSHOT_RADIUS,
            collision_radius: Self::DROPSHOT_COLLISION_RADIUS,
            ..Default::default()
        };

        ball.initialize();

        ball
    }

    /// Sets a value location and calculates the moi
    pub fn initialize(&mut self) {
        self.location.z = 1.1 * self.collision_radius;
        self.calculate_moi();
    }

    /// Calculates the moment of inertia of the ball
    pub fn calculate_moi(&mut self) {
        self.moi = 0.4 * Self::M * self.radius * self.radius;
    }

    /// Updates the ball with everything that changes from game tick to game tick
    pub fn update(&mut self, time: f32, location: Vec3A, velocity: Vec3A, angular_velocity: Vec3A) {
        self.time = time;
        self.location = location;
        self.velocity = velocity;
        self.angular_velocity = angular_velocity;
    }

    /// Converts the ball into a sphere
    #[must_use]
    #[inline]
    pub const fn hitbox(&self) -> Sphere {
        Sphere {
            center: self.location,
            radius: self.collision_radius,
        }
    }

    /// Simulate the ball for one game tick
    ///
    /// `dt` - The delta time (game tick length)
    pub fn step(&mut self, game: &Game, dt: f32) {
        let g = if self.velocity.length_squared() != 0. { game.gravity } else { Vec3A::ZERO };

        match game.collision_mesh.collide(&self.hitbox()) {
            Some(contact) => {
                let p = contact.start;
                let n = contact.direction;

                let loc = p - self.location;

                let m_reduced = 1. / (Self::INV_M + loc.length_squared() / self.moi);

                let v_perp = n * self.velocity.dot(n).min(0.);
                let v_para = self.velocity - v_perp - loc.cross(self.angular_velocity);

                let ratio = v_perp.length() / v_para.length().max(0.0001);

                let j_perp = v_perp * Self::RESTITUTION_M;
                let j_para = -(Self::MU * ratio).min(1.) * m_reduced * v_para;

                let j = j_perp + j_para;

                self.angular_velocity += loc.cross(j) / self.moi;
                self.velocity += (j / Self::M) + (Self::DRAG * self.velocity + g) * dt;
                self.location += self.velocity * dt;

                let penetration = self.collision_radius - (self.location - p).dot(n);
                if penetration > 0. {
                    self.location += n * (1.001 * penetration);
                }
            }
            None => {
                self.velocity += (self.velocity * Self::DRAG + g) * dt;
                self.location += self.velocity * dt;
            }
        }

        self.angular_velocity *= (Self::W_MAX * self.angular_velocity.length_recip()).min(1.);
        self.velocity *= (Self::V_MAX * self.velocity.length_recip()).min(1.);
        self.time += dt;
    }

    #[inline]
    /// Simulate the ball for a given amount of time
    pub fn get_ball_prediction_struct_for_time(self, game: &Game, time: &f32) -> BallPrediction {
        self.get_ball_prediction_struct_for_slices(game, (time / Self::SIMULATION_DT).round() as usize)
    }

    #[inline]
    /// Simulate the ball for the stand amount of time
    pub fn get_ball_prediction_struct(self, game: &Game) -> BallPrediction {
        self.get_ball_prediction_struct_for_slices(game, Self::STANDARD_NUM_SLICES)
    }

    /// Simulate the ball for a given amount of ticks
    pub fn get_ball_prediction_struct_for_slices(self, game: &Game, num_slices: usize) -> BallPrediction {
        let mut ball = self;
        let mut slices = Vec::with_capacity(num_slices);

        for _ in 0..num_slices {
            ball.step(game, Self::SIMULATION_DT);
            slices.push(ball);
        }

        slices
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::load_soccar;

    #[test]
    fn check_standard_num_slices() {
        let (game, ball) = load_soccar();

        let prediction = ball.get_ball_prediction_struct(&game);

        assert_eq!(prediction.len(), Ball::STANDARD_NUM_SLICES);
    }

    #[test]
    fn check_custom_num_slices() {
        const REQUESTED_SLICES: usize = 200;

        let (game, ball) = load_soccar();

        let prediction = ball.get_ball_prediction_struct_for_slices(&game, REQUESTED_SLICES);

        assert_eq!(prediction.len(), REQUESTED_SLICES);
    }

    #[test]
    fn check_num_slices_for_time() {
        const REQUESTED_TIME: f32 = 8.0;

        let (game, ball) = load_soccar();

        let prediction = ball.get_ball_prediction_struct_for_time(&game, &REQUESTED_TIME);

        let predicted_slices = (REQUESTED_TIME / Ball::SIMULATION_DT).round() as usize;

        assert_eq!(prediction.len(), predicted_slices);
    }
}
