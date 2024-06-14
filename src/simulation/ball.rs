//! Tools for simulation a Rocket League ball.

use super::{
    game::{Constraints, Game},
    geometry::Sphere,
};
use glam::Vec3A;

#[cfg(feature = "heatseeker")]
use super::geometry::Angle;

#[cfg(feature = "heatseeker")]
mod heatseeker {
    use glam::Vec3A;

    pub const INITIAL_TARGET_SPEED: f32 = 2900.;
    pub const TARGET_SPEED_INCREMENT: f32 = 85.;
    pub const TARGET_Y: f32 = 5120.;
    pub const TARGET_Z: f32 = 320.;
    pub const HORIZONTAL_BLEND: f32 = 1.45;
    pub const VERTICAL_BLEND: f32 = 0.78;
    pub const SPEED_BLEND: f32 = 0.3;
    pub const MAX_TURN_PITCH: f32 = 0.671;
    pub const MAX_SPEED: f32 = 4600.;
    pub const WALL_BOUNCE_CHANGE_NORMAL_Y: f32 = 0.75;
    pub const BALL_START_POS: Vec3A = Vec3A::new(-1000., -2220., 92.75);
    pub const BALL_START_VEL: Vec3A = Vec3A::new(0., -65., 650.);
}

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
    #[cfg(feature = "heatseeker")]
    pub(crate) y_target_dir: f32,
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
    const DRAG: f32 = 0.03;

    const V_MAX: f32 = 6000.;
    const W_MAX: f32 = 6.;

    const M: f32 = 30.;
    pub(crate) const INV_M: f32 = 1. / Self::M;

    const STANDARD_RADIUS: f32 = 91.25;
    const HOOPS_RADIUS: f32 = 96.3831;
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
            #[cfg(feature = "heatseeker")]
            y_target_dir: 0.,
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
    #[cfg(feature = "heatseeker")]
    /// Sets the default values for a heatseeker ball
    pub fn initialize_heatseeker() -> Self {
        Self {
            radius: Self::STANDARD_RADIUS,
            inv_inertia: Self::get_inv_inertia(Self::STANDARD_RADIUS),
            location: heatseeker::BALL_START_POS,
            velocity: heatseeker::BALL_START_VEL,
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

    #[inline]
    pub(crate) fn get_velocity_in_local_point(&self, rel_pos: Vec3A) -> Vec3A {
        self.velocity + self.angular_velocity.cross(rel_pos)
    }

    /// Simulate the ball for one game tick,
    /// returning whether the ball has made contact with the world
    fn internal_step(&mut self, game: &Game, dt: f32) -> Option<Vec3A> {
        self.velocity *= (1. - Self::DRAG).powf(dt);
        let external_force_impulse = game.gravity * dt;

        let contacts = game.triangle_collisions.collide(self.hitbox());

        let world_contact_normal = if contacts.is_empty() {
            None
        } else {
            let mut constraints = Constraints::new(Self::INV_M, external_force_impulse);
            constraints.add_contacts(contacts, self, dt);

            let (delta_velocity, push_velocity, normal) = constraints.solve(self);

            self.velocity += delta_velocity.linear;
            self.angular_velocity += delta_velocity.angular;
            self.location += push_velocity * dt;

            Some(normal)
        };

        self.velocity += external_force_impulse;
        self.location += self.velocity * dt;
        self.angular_velocity = self.angular_velocity.clamp_length_max(Self::W_MAX);
        self.velocity = self.velocity.clamp_length_max(Self::V_MAX);

        world_contact_normal
    }

    /// Set the target direction for the ball in Heatseeker mode
    ///
    /// `blue_goal` - Whether the ball should target the blue goal
    #[cfg(feature = "heatseeker")]
    pub fn set_heatseeker_target(&mut self, blue_goal: bool) {
        self.y_target_dir = if blue_goal { -1. } else { 1. };
    }

    /// Simulate the ball for one game tick in Heatseeker mode
    ///
    /// `dt` - The delta time (game tick length)
    #[cfg(feature = "heatseeker")]
    pub fn step_heatseeker(&mut self, game: &Game, dt: f32) {
        self.time += dt;

        if self.velocity.length_squared() != 0. || self.angular_velocity.length_squared() != 0. {
            if self.velocity.x != 0. || self.velocity.y.abs() > 70. {
                let vel_angle = Angle::from_vec(self.velocity.normalize());

                // Determine angle to goal
                let goal_target_pos = Vec3A::new(0., heatseeker::TARGET_Y * self.velocity.y.signum(), heatseeker::TARGET_Z);
                let angle_to_goal = (goal_target_pos - self.location).normalize();
                let delta_angle = Angle::from_vec(angle_to_goal - self.velocity.normalize());

                // Determine speed ratio
                let cur_speed = self.velocity.length();
                let speed_ratio = cur_speed / heatseeker::MAX_SPEED;

                // Interpolate delta
                let base_interp_factor = speed_ratio * dt;
                let mut new_angle = vel_angle;
                new_angle.yaw += delta_angle.yaw * base_interp_factor * heatseeker::HORIZONTAL_BLEND;
                new_angle.pitch += delta_angle.pitch * base_interp_factor * heatseeker::VERTICAL_BLEND;
                new_angle.normalize_fix();

                // Limit pitch
                new_angle.pitch = new_angle.pitch.clamp(-heatseeker::MAX_TURN_PITCH, heatseeker::MAX_TURN_PITCH);

                // Determine new interpolated speed
                let current_state = ((cur_speed - heatseeker::INITIAL_TARGET_SPEED) / heatseeker::TARGET_SPEED_INCREMENT)
                    .floor()
                    .clamp(0., 20.);
                let target_speed = current_state * heatseeker::TARGET_SPEED_INCREMENT + heatseeker::INITIAL_TARGET_SPEED;
                let new_speed = cur_speed + ((target_speed - cur_speed) * heatseeker::SPEED_BLEND);

                // Update velocity
                self.velocity = new_angle.get_forward_vec() * new_speed;
            }

            if let Some(normal) = self.internal_step(game, dt) {
                if normal.y > heatseeker::WALL_BOUNCE_CHANGE_NORMAL_Y {
                    self.y_target_dir = normal.y.signum();
                }
            }
        }
    }

    /// Simulate the ball for one game tick
    ///
    /// `dt` - The delta time (game tick length)
    pub fn step(&mut self, game: &Game, dt: f32) {
        self.time += dt;

        if self.velocity.length_squared() != 0. || self.angular_velocity.length_squared() != 0. {
            self.internal_step(game, dt);
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

    /// Simulate the ball in Heatseeker mode for at least the given amount of time
    #[inline]
    #[must_use]
    #[cfg(feature = "heatseeker")]
    pub fn get_heatseeker_prediction_struct_for_time(self, game: &Game, time: f32) -> Predictions {
        debug_assert!(time >= 0.);
        // Ignoring these warnings is ok because:
        // We are rounding up to the nearest integer so no truncation will occur
        // We are making sure that the minimum possible value is 0 so no sign loss will occur
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        self.get_heatseeker_prediction_struct_for_slices(game, (time / Self::SIMULATION_DT).round() as usize)
    }

    /// Simulate the ball in Heatseeker mode for the standard amount of time (6 seconds)
    #[inline]
    #[must_use]
    #[cfg(feature = "heatseeker")]
    pub fn get_heatseeker_prediction_struct(self, game: &Game) -> Predictions {
        self.get_heatseeker_prediction_struct_for_slices(game, Self::STANDARD_NUM_SLICES)
    }

    /// Simulate the ball in Heatseeker mode for a given amount of ticks
    #[inline]
    #[must_use]
    #[cfg(feature = "heatseeker")]
    pub fn get_heatseeker_prediction_struct_for_slices(mut self, game: &Game, num_slices: usize) -> Predictions {
        (0..num_slices)
            .map(|_| {
                self.step_heatseeker(game, Self::SIMULATION_DT);
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
