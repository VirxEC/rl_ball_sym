//! Tools for simulation a Rocket League ball.

use crate::{
    linear_algebra::math::Vec3AExt,
    simulation::{game::Game, geometry::Sphere},
};
use glam::Vec3A;

use super::geometry::Ray;

/// Represents the game's ball
#[derive(Clone, Copy, Debug)]
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
    const FRICTION: f32 = 0.35;
    const RESTITUTION: f32 = 0.6;
    const DRAG: f32 = 0.03;

    const V_MAX: f32 = 6000.;
    const W_MAX: f32 = 6.;

    const M: f32 = 30.;
    const INV_M: f32 = 1. / Self::M;
    const DEFAULT_INERTIA: f32 = 1. / (0.4 * Self::M);

    const VELOCITY_THRESHOLD: f32 = 10.;
    const CONTACT_BREAKING_THRESHOLD: f32 = 1.905;

    const STANDARD_RADIUS: f32 = 91.25;
    const HOOPS_RADIUS: f32 = 96.38307;
    const DROPSHOT_RADIUS: f32 = 100.2565;

    const SIMULATION_DT: f32 = 1. / 120.;
    const STANDARD_NUM_SLICES: usize = 720;

    #[must_use]
    #[inline]
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

    /// Sets the default values for a standard ball
    #[must_use]
    #[inline]
    pub fn initialize_standard() -> Self {
        Self {
            radius: Self::STANDARD_RADIUS,
            inv_inertia: Self::get_inv_inertia(Self::STANDARD_RADIUS),
            location: Self::default_height(Self::STANDARD_RADIUS),
            ..Default::default()
        }
    }

    /// Sets the default values for a hoops ball
    #[must_use]
    #[inline]
    pub fn initialize_hoops() -> Self {
        Self {
            radius: Self::HOOPS_RADIUS,
            inv_inertia: Self::get_inv_inertia(Self::HOOPS_RADIUS),
            location: Self::default_height(Self::HOOPS_RADIUS),
            ..Default::default()
        }
    }

    /// Sets the default values for a dropshot ball
    #[must_use]
    #[inline]
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

    /// Calculates the default ball height based on the collision radius (arbitrary)
    #[must_use]
    #[inline]
    fn default_height(radius: f32) -> Vec3A {
        Vec3A::new(0., 0., 1.1 * radius + Self::CONTACT_BREAKING_THRESHOLD)
    }

    #[inline]
    fn get_inv_inertia(radius: f32) -> f32 {
        1. / (0.4 * Self::M * radius.powi(2))
    }

    /// Get the radius of the ball
    #[must_use]
    #[inline]
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

    /// Converts the ball into a sphere
    #[must_use]
    #[inline]
    pub fn hitbox(&self) -> Sphere {
        Sphere {
            center: self.location,
            radius: self.radius + Self::CONTACT_BREAKING_THRESHOLD,
        }
    }

    fn get_delta_from_contact(&self, contact: Ray, gravity: Vec3A, dt: f32) -> (Vec3A, Vec3A) {
        // dbg!(contact.start / 50.);
        // dbg!(contact.direction);
        // dbg!(self.location / 50.);
        let rel_pos = contact.start - self.location;

        let external_force_impluse = gravity * dt;
        let vel = self.velocity + external_force_impluse + self.angular_velocity.cross(rel_pos);
        let rel_vel = contact.direction.dot(vel);

        // collision impulse
        let (mut delta_velocity, mut delta_ang_vel, applied_impulse) = {
            let rel_pos_cross_normal = rel_pos.cross(contact.direction);

            let restitution = {
                let vel = self.velocity + self.angular_velocity.cross(rel_pos);
                let rel_vel = contact.direction.dot(vel);

                if rel_vel.abs() < Self::VELOCITY_THRESHOLD {
                    0.
                } else {
                    (Self::RESTITUTION * -rel_vel).max(0.)
                }
            };

            let velocity_error = restitution - rel_vel;
            let velocity_impulse = velocity_error * Self::M;

            let angular_component = self.inv_inertia * rel_pos_cross_normal;

            let impulse_magnitude = velocity_impulse.clamp(0., 5e11);
            let linear_component = contact.direction * Self::INV_M;

            (
                linear_component * impulse_magnitude,
                angular_component * impulse_magnitude,
                impulse_magnitude,
            )
        };
        // dbg!(applied_impulse / 50.);
        // friction impulse
        if applied_impulse > 0. {
            let mut lateral_friction_dir = vel - contact.direction * rel_vel;
            let lat_rel_vel = lateral_friction_dir.length_squared();

            let (jac_diag_ab_inv, rel_pos_cross_normal, angular_component) = if lat_rel_vel > f32::EPSILON {
                lateral_friction_dir /= lat_rel_vel.sqrt();
                // dbg!(lateral_friction_dir);

                // dbg!(rel_pos / 50.);
                let torque_axis = rel_pos.cross(lateral_friction_dir);
                // dbg!(torque_axis / 50.);
                let angular_component = self.inv_inertia * torque_axis;
                // dbg!(angular_component);

                let denom = Self::INV_M + lateral_friction_dir.dot(angular_component.cross(rel_pos));
                (1. / denom, torque_axis, angular_component)
            } else {
                (Self::M, Vec3A::ZERO, Vec3A::ZERO)
            };

            let rel_vel = lateral_friction_dir.dot(self.velocity + external_force_impluse)
                + rel_pos_cross_normal.dot(self.angular_velocity);
            let velocity_error = -rel_vel;
            // dbg!(velocity_error / 50.);
            // dbg!(jac_diag_ab_inv);
            let velocity_impulse = velocity_error * jac_diag_ab_inv;
            // dbg!(velocity_impulse / 50.);
            let limit: f32 = Self::FRICTION * applied_impulse;
            // dbg!(limit / 50.);
            let impulse_magnitude = velocity_impulse.clamp(-limit, limit);
            let linear_component = lateral_friction_dir * Self::INV_M;

            delta_velocity += linear_component * impulse_magnitude;
            delta_ang_vel += angular_component * impulse_magnitude;
        }

        (delta_velocity + external_force_impluse, delta_ang_vel)
    }

    /// Simulate the ball for one game tick
    ///
    /// `dt` - The delta time (game tick length)
    pub fn step(&mut self, game: &Game, dt: f32) {
        self.time += dt;

        if self.velocity.length_squared() != 0. || self.angular_velocity.length_squared() != 0. {
            self.velocity *= (1. - Self::DRAG).powf(dt);

            if let Some(contact) = game.collision_mesh.collide(self.hitbox()) {
                // dbg!(self.time);
                let (dt_velocity, dt_ang_vel) = self.get_delta_from_contact(contact, game.gravity, dt);

                self.velocity += dt_velocity;
                self.angular_velocity += dt_ang_vel;
            } else {
                self.velocity += game.gravity * dt;
            }

            self.location += self.velocity * dt;
            self.angular_velocity = self.angular_velocity.clamp_length_max(Self::W_MAX);
            self.velocity = self.velocity.clamp_length_max(Self::V_MAX);

            self.location.round_vec_bullet(50., 0.0002);
            self.velocity.round_vec_bullet(50., 0.0002);
            self.angular_velocity.round_vec_bullet(1., 0.00001);
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
