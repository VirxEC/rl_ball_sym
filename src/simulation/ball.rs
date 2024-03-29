//! Tools for simulation a Rocket League ball.

use super::{game::Game, geometry::Sphere};
use glam::Vec3A;

/// Represents the game's ball
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
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
    pub(crate) radius: f32,
    /// 1 / Moment of inertia of the ball in the form of a diagonal matrix
    pub(crate) inv_inertia: f32,
}

impl Default for Ball {
    #[inline]
    fn default() -> Self {
        Self::const_default()
    }
}

/// Collection of Balls representing future predictions based on field geometry
pub type Predictions = Vec<Ball>;

impl Ball {
    const RESTITUTION: f32 = 0.6;
    const DRAG: f32 = 0.03;
    const MU: f32 = 2.;

    const V_MAX: f32 = 6000.;
    const W_MAX: f32 = 6.;

    const M: f32 = 30.;
    const INV_M: f32 = 1. / Self::M;
    const RESTITUTION_M: f32 = -(1. + Self::RESTITUTION) * Self::M;

    const STANDARD_RADIUS: f32 = 91.25;
    const HOOPS_RADIUS: f32 = 96.38307;
    const DROPSHOT_RADIUS: f32 = 100.2565;

    const SIMULATION_DT: f32 = 1. / 120.;
    const STANDARD_NUM_SLICES: usize = 720;

    const DEFAULT_INERTIA: f32 = 1. / (0.4 * Self::M);

    #[inline]
    #[must_use]
    /// `Ball::default()`, but const
    pub const fn const_default() -> Self {
        Self {
            time: 0.,
            location: Vec3A::Z,
            velocity: Vec3A::ZERO,
            angular_velocity: Vec3A::ZERO,
            radius: 1.,
            inv_inertia: Self::DEFAULT_INERTIA,
        }
    }

    #[inline]
    #[must_use]
    /// Sets the default values for a standard ball
    pub fn initialize_standard() -> Self {
        Self {
            radius: Self::STANDARD_RADIUS,
            inv_inertia: Self::get_inv_inertia(Self::STANDARD_RADIUS),
            location: Self::default_height(Self::STANDARD_RADIUS),
            ..Default::default()
        }
    }

    #[inline]
    #[must_use]
    /// Sets the default values for a hoops ball
    pub fn initialize_hoops() -> Self {
        Self {
            radius: Self::HOOPS_RADIUS,
            inv_inertia: Self::get_inv_inertia(Self::HOOPS_RADIUS),
            location: Self::default_height(Self::HOOPS_RADIUS),
            ..Default::default()
        }
    }

    #[inline]
    #[must_use]
    /// Sets the default values for a dropshot ball
    pub fn initialize_dropshot() -> Self {
        Self {
            radius: Self::DROPSHOT_RADIUS,
            inv_inertia: Self::get_inv_inertia(Self::DROPSHOT_RADIUS),
            location: Self::default_height(Self::DROPSHOT_RADIUS),
            ..Default::default()
        }
    }

    /// Set a custom radius for the ball
    pub fn set_radius(&mut self, radius: f32) {
        debug_assert!(radius > 0.);
        self.radius = radius;
        self.inv_inertia = Self::get_inv_inertia(radius);
    }

    #[inline]
    /// Calculates the default ball height based on the collision radius (arbitrary)
    fn default_height(radius: f32) -> Vec3A {
        Vec3A::new(0., 0., 1.1 * radius)
    }

    #[inline]
    fn get_inv_inertia(radius: f32) -> f32 {
        1. / (0.4 * Self::M * radius.powi(2))
    }

    #[inline]
    #[must_use]
    /// Get the radius of the ball
    pub const fn radius(&self) -> f32 {
        self.radius
    }

    /// Updates the ball with everything that changes from game tick to game tick
    pub fn update(&mut self, time: f32, location: Vec3A, velocity: Vec3A, angular_velocity: Vec3A) {
        self.time = time;
        self.location = location;
        self.velocity = velocity;
        self.angular_velocity = angular_velocity;
    }

    #[inline]
    fn hitbox(&self) -> Sphere {
        Sphere::new(self.location, self.radius)
    }

    /// Simulate the ball for one game tick
    ///
    /// `dt` - The delta time (game tick length)
    pub fn step(&mut self, game: &Game, dt: f32) {
        self.time += dt;

        if self.velocity.length_squared() != 0. || self.angular_velocity.length_squared() != 0. {
            if let Some(contact) = game.triangle_collisions.collide(self.hitbox()) {
                let p = contact.start;
                let n = contact.direction;

                let loc = p - self.location;

                let m_reduced = 1. / (Self::INV_M + loc.length_squared() * self.inv_inertia);

                let v_perp = n * self.velocity.dot(n).min(0.);
                let v_para = self.velocity - v_perp - loc.cross(self.angular_velocity);

                let ratio = v_perp.length() / v_para.length().max(0.0001);

                let j_perp = v_perp * Self::RESTITUTION_M;
                let j_para = -(Self::MU * ratio).min(1.) * m_reduced * v_para;

                let j = j_perp + j_para;

                self.velocity *= (1. - Self::DRAG).powf(dt);
                self.velocity += game.gravity * dt;
                self.velocity += j * Self::INV_M;

                self.location += self.velocity * dt;

                self.angular_velocity += loc.cross(j) * self.inv_inertia;

                let penetration = self.radius + Sphere::CONTACT_BREAKING_THRESHOLD - (self.location - p).dot(n);
                if penetration > 0. {
                    self.location += 1.001 * penetration * n;
                }
            } else {
                self.velocity *= (1. - Self::DRAG).powf(dt);
                self.velocity += game.gravity * dt;
                self.location += self.velocity * dt;
            }

            self.angular_velocity *= (Self::W_MAX * self.angular_velocity.length_recip()).min(1.);
            self.velocity *= (Self::V_MAX * self.velocity.length_recip()).min(1.);
        }
    }

    /// Simulate the ball for at least the given amount of time
    #[inline]
    #[must_use]
    pub fn get_ball_prediction_struct_for_time(self, game: &Game, time: f32) -> Predictions {
        debug_assert!(time >= 0.);
        // Ignoring these warnings is ok because:
        // We are rounding up to the nearest integer so no truncation will occur
        // We are making sure that the minimum possible value is 0 so no sign loss will occur
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        self.get_ball_prediction_struct_for_slices(game, (time / Self::SIMULATION_DT).round() as usize)
    }

    /// Simulate the ball for the standard amount of time (6 seconds)
    #[inline]
    #[must_use]
    pub fn get_ball_prediction_struct(self, game: &Game) -> Predictions {
        self.get_ball_prediction_struct_for_slices(game, Self::STANDARD_NUM_SLICES)
    }

    /// Simulate the ball for a given amount of ticks
    #[inline]
    #[must_use]
    pub fn get_ball_prediction_struct_for_slices(mut self, game: &Game, num_slices: usize) -> Predictions {
        (0..num_slices)
            .map(|_| {
                self.step(game, Self::SIMULATION_DT);
                self
            })
            .collect()
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::load_standard;

    #[test]
    fn check_standard_num_slices() {
        let (game, ball) = load_standard();

        let prediction = ball.get_ball_prediction_struct(&game);

        assert_eq!(prediction.len(), Ball::STANDARD_NUM_SLICES);
    }

    #[test]
    fn check_custom_num_slices() {
        const REQUESTED_SLICES: usize = 200;

        let (game, ball) = load_standard();

        let prediction = ball.get_ball_prediction_struct_for_slices(&game, REQUESTED_SLICES);

        assert_eq!(prediction.len(), REQUESTED_SLICES);
    }

    #[test]
    fn check_num_slices_for_time() {
        const REQUESTED_TIME: f32 = 8.0;

        let (game, ball) = load_standard();

        let prediction = ball.get_ball_prediction_struct_for_time(&game, REQUESTED_TIME);

        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let predicted_slices = (REQUESTED_TIME / Ball::SIMULATION_DT).ceil().max(0.) as usize;

        assert_eq!(prediction.len(), predicted_slices);
    }
}
