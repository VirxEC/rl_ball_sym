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
    use std::f32::consts::PI;

    pub const INITIAL_TARGET_SPEED: f32 = 2900.;
    pub const TARGET_SPEED_INCREMENT: f32 = 85.;
    pub const TARGET_Y: f32 = 5120.;
    pub const TARGET_Z: f32 = 320.;
    pub const HORIZONTAL_BLEND: f32 = 1.45;
    pub const VERTICAL_BLEND: f32 = 0.78;
    pub const SPEED_BLEND: f32 = 0.3;
    pub const MAX_TURN_PITCH: f32 = 7000. * PI / (1u16 << 15) as f32;
    pub const MAX_SPEED: f32 = 4600.;
    pub const WALL_BOUNCE_CHANGE_Y_THRESH: f32 = 300.;
    pub const WALL_BOUNCE_CHANGE_NORMAL_Y: f32 = 0.5;
    pub const WALL_BOUNCE_FORCE_SCALE: f32 = 1. / 3.;
    pub const WALL_BOUNCE_UP_FRAC: f32 = 0.3;
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
    radius: f32,
    /// 1 / Moment of inertia of the ball in the form of a diagonal matrix
    pub(crate) inv_inertia: f32,
    #[cfg(feature = "heatseeker")]
    pub(crate) y_target_dir: f32,
}

impl Default for Ball {
    #[inline]
    fn default() -> Self {
        Self::DEFAULT_STANDARD
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

    const DEFAULT: Self = Self {
        time: 0.,
        location: Vec3A::Z,
        velocity: Vec3A::ZERO,
        angular_velocity: Vec3A::ZERO,
        radius: 1.,
        inv_inertia: 1. / (0.4 * Self::M),
        #[cfg(feature = "heatseeker")]
        y_target_dir: 0.,
    };

    /// Calculates the default ball height based on the collision radius (arbitrary)
    const fn default_height(radius: f32) -> Vec3A {
        Vec3A::new(0., 0., 1.1 * radius)
    }

    /// Calculates the inverse inertia of the ball based on the collision radius
    const fn get_inv_inertia(radius: f32) -> f32 {
        1. / (0.4 * Self::M * radius * radius)
    }

    /// A ball with default values for the standard game mode
    pub const DEFAULT_STANDARD: Self = Self {
        radius: Self::STANDARD_RADIUS,
        inv_inertia: Self::get_inv_inertia(Self::STANDARD_RADIUS),
        location: Self::default_height(Self::STANDARD_RADIUS),
        ..Self::DEFAULT
    };

    /// A ball with default values for the heatseeker game mode
    #[cfg(feature = "heatseeker")]
    pub const DEFAULT_HEATSEEKER: Self = Self {
        radius: Self::STANDARD_RADIUS,
        inv_inertia: Self::get_inv_inertia(Self::STANDARD_RADIUS),
        location: Vec3A::new(-1000., -2220., 92.75),
        velocity: Vec3A::new(0., -65., 650.),
        ..Self::DEFAULT
    };

    /// A ball with default values for the hoops game mode
    pub const DEFAULT_HOOPS: Self = Self {
        radius: Self::HOOPS_RADIUS,
        inv_inertia: Self::get_inv_inertia(Self::HOOPS_RADIUS),
        location: Self::default_height(Self::HOOPS_RADIUS),
        ..Self::DEFAULT
    };

    /// A ball with default values for the dropshot game mode
    pub const DEFAULT_DROPSHOT: Self = Self {
        radius: Self::DROPSHOT_RADIUS,
        inv_inertia: Self::get_inv_inertia(Self::DROPSHOT_RADIUS),
        location: Self::default_height(Self::DROPSHOT_RADIUS),
        ..Self::DEFAULT
    };

    /// Set a custom radius for the ball
    pub const fn set_radius(&mut self, radius: f32) {
        debug_assert!(radius > 0.);
        self.radius = radius;
        self.inv_inertia = Self::get_inv_inertia(radius);
    }

    #[inline]
    #[must_use]
    /// Get the radius of the ball
    pub const fn radius(&self) -> f32 {
        self.radius
    }

    /// Updates the ball with everything that changes from game tick to game tick
    pub const fn update(&mut self, time: f32, location: Vec3A, velocity: Vec3A, angular_velocity: Vec3A) {
        self.time = time;
        self.location = location;
        self.velocity = velocity;
        self.angular_velocity = angular_velocity;
    }

    #[inline]
    const fn hitbox(&self) -> Sphere {
        Sphere::new(self.location, self.radius)
    }

    #[inline]
    pub(crate) fn get_velocity_in_local_point(&self, rel_pos: Vec3A) -> Vec3A {
        self.velocity + self.angular_velocity.cross(rel_pos)
    }

    fn limit_velocities(&mut self) {
        self.angular_velocity = self.angular_velocity.clamp_length_max(Self::W_MAX);
        self.velocity = self.velocity.clamp_length_max(Self::V_MAX);
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

        world_contact_normal
    }

    /// Set the target direction for the ball in Heatseeker mode
    ///
    /// `blue_goal` - Whether the ball should target the blue goal
    #[cfg(feature = "heatseeker")]
    pub const fn set_heatseeker_target(&mut self, blue_goal: bool) {
        self.y_target_dir = if blue_goal { -1. } else { 1. };
    }

    /// Reset the target direction for the ball in Heatseeker mode
    ///
    /// This will make the ball behave normally and not target any goal
    #[cfg(feature = "heatseeker")]
    pub const fn reset_heatseeker_target(&mut self) {
        self.y_target_dir = 0.;
    }

    /// Get the target position for the ball in Heatseeker mode
    #[inline]
    #[must_use]
    #[cfg(feature = "heatseeker")]
    pub const fn get_heatseeker_target(&self) -> Vec3A {
        Vec3A::new(
            0.,
            heatseeker::TARGET_Y * self.y_target_dir,
            heatseeker::TARGET_Z,
        )
    }

    /// Simulate the ball for one game tick in Heatseeker mode
    ///
    /// `dt` - The delta time (game tick length)
    #[cfg(feature = "heatseeker")]
    pub fn step_heatseeker(&mut self, game: &Game, dt: f32) {
        self.time += dt;

        if self.velocity.length_squared() != 0. || self.angular_velocity.length_squared() != 0. {
            if self.y_target_dir != 0. {
                let vel = self.velocity.length();
                let vel_norm = self.velocity / vel;

                let vel_angle = Angle::from_vec(vel_norm);
                let angle_to_goal =
                    Angle::from_vec((self.get_heatseeker_target() - self.location).normalize());
                let delta_angle = angle_to_goal - vel_angle;

                // Determine speed ratio
                let speed_ratio = vel / heatseeker::MAX_SPEED;

                // Interpolate delta
                let base_interp_factor = speed_ratio * dt;
                let mut new_angle = vel_angle;
                new_angle.yaw +=
                    delta_angle.yaw * base_interp_factor * heatseeker::HORIZONTAL_BLEND;
                new_angle.pitch +=
                    delta_angle.pitch * base_interp_factor * heatseeker::VERTICAL_BLEND;
                new_angle.normalize_fix();

                // Limit pitch
                new_angle.pitch = new_angle
                    .pitch
                    .clamp(-heatseeker::MAX_TURN_PITCH, heatseeker::MAX_TURN_PITCH);

                // Determine new interpolated speed
                let current_state = ((vel - heatseeker::INITIAL_TARGET_SPEED)
                    / heatseeker::TARGET_SPEED_INCREMENT)
                    .floor()
                    .clamp(0., 20.);
                let target_speed = current_state * heatseeker::TARGET_SPEED_INCREMENT
                    + heatseeker::INITIAL_TARGET_SPEED;
                let new_speed = vel + ((target_speed - vel) * heatseeker::SPEED_BLEND);

                // Update velocity
                // dbg!(new_angle);
                self.velocity = new_angle.get_forward_vec() * new_speed;
            }

            if let Some(normal) = self.internal_step(game, dt) {
                if self.y_target_dir != 0. {
                    let rel_normal_y = normal.y * self.y_target_dir;
                    let rel_y = self.location.y * self.y_target_dir;

                    if rel_y >= 5120. - heatseeker::WALL_BOUNCE_CHANGE_Y_THRESH
                        && rel_normal_y <= -heatseeker::WALL_BOUNCE_CHANGE_NORMAL_Y
                    {
                        self.y_target_dir *= -1.;

                        let dir_to_goal =
                            (self.get_heatseeker_target() - self.location).normalize();

                        let bounce_dir = dir_to_goal * (1. - heatseeker::WALL_BOUNCE_UP_FRAC)
                            + Vec3A::Z * heatseeker::WALL_BOUNCE_UP_FRAC;
                        let bounce_impulse = bounce_dir
                            * self.velocity.length()
                            * heatseeker::WALL_BOUNCE_FORCE_SCALE;
                        self.velocity += bounce_impulse;
                    }
                }
            }

            self.limit_velocities();
        }
    }

    /// Simulate the ball for one game tick
    ///
    /// `dt` - The delta time (game tick length)
    pub fn step(&mut self, game: &Game, dt: f32) {
        self.time += dt;

        if self.velocity.length_squared() != 0. || self.angular_velocity.length_squared() != 0. {
            self.internal_step(game, dt);
            self.limit_velocities();
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
        self.get_ball_prediction_struct_for_slices(
            game,
            (time / Self::SIMULATION_DT).round() as usize,
        )
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
    pub fn get_ball_prediction_struct_for_slices(
        mut self,
        game: &Game,
        num_slices: usize,
    ) -> Predictions {
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
        self.get_heatseeker_prediction_struct_for_slices(
            game,
            (time / Self::SIMULATION_DT).round() as usize,
        )
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
    pub fn get_heatseeker_prediction_struct_for_slices(
        mut self,
        game: &Game,
        num_slices: usize,
    ) -> Predictions {
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
