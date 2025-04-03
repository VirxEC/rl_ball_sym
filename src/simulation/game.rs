//! All the data about the game to simulate it.

use super::{ball::Ball, geometry::Contact, tri_bvh::TriangleBvh};
use arrayvec::ArrayVec;
use glam::Vec3A;
use std::f32::consts::FRAC_1_SQRT_2;

const COEFF_FRICTION: f32 = 0.35;

/// All of the game information.
#[derive(Clone, Debug)]
pub struct Game {
    /// The gravity of the game
    pub gravity: Vec3A,
    /// The Bvh generated from the field
    pub(crate) triangle_collisions: TriangleBvh,
}

impl Game {
    const RESTITUTION: f32 = 0.6;
    const RESTITUTION_VELOCITY_THRESHOLD: f32 = 0.2;
    const GRAVITY: Vec3A = Vec3A::new(0., 0., -650.);

    #[inline]
    pub(crate) const fn new(triangle_collisions: TriangleBvh) -> Self {
        Self {
            gravity: Self::GRAVITY,
            triangle_collisions,
        }
    }

    pub(crate) fn plane_space_1(n: Vec3A) -> Vec3A {
        if n.z.abs() > FRAC_1_SQRT_2 {
            // choose p in y-z plane
            let a = n.y * n.y + n.z * n.z;
            let k = 1. / a.sqrt();
            Vec3A::new(0., -n.z * k, n.y * k)
        } else {
            // choose p in x-y plane
            let a = n.x * n.x + n.y * n.y;
            let k = 1. / a.sqrt();
            Vec3A::new(-n.y * k, n.x * k, 0.)
        }
    }

    #[inline]
    pub(crate) fn restitution_curve(rel_vel: f32) -> f32 {
        if -rel_vel > Self::RESTITUTION_VELOCITY_THRESHOLD {
            Self::RESTITUTION * -rel_vel
        } else {
            0.
        }
    }
}

#[derive(Clone, Debug)]
pub struct Constraint {
    pub contact_normal: Vec3A,
    pub rel_pos_cross_normal: Vec3A,
    pub angular_component: Vec3A,
    pub rhs: f32,
    pub rhs_penetration: f32,
    pub lower_limit: f32,
    pub upper_limit: f32,
    pub jac_diag_ab_inv: f32,
    pub applied_impulse: f32,
    pub applied_push_impulse: f32,
}

impl Constraint {
    fn resolve_single_constraint_row_lower_limit(
        &mut self,
        deltas: &mut VelocityPair,
        inv_mass: f32,
    ) -> f32 {
        let mut delta_impulse: f32 = self.rhs;

        let delta_vel_1_dot_n =
            self.contact_normal.dot(deltas.linear) + self.rel_pos_cross_normal.dot(deltas.angular);
        delta_impulse -= delta_vel_1_dot_n * self.jac_diag_ab_inv;

        let sum = self.applied_impulse + delta_impulse;
        let low_min_applied = self.lower_limit - self.applied_impulse;
        delta_impulse = if sum < self.lower_limit {
            low_min_applied
        } else {
            delta_impulse
        };

        self.applied_impulse = sum.max(self.lower_limit);

        let linear_component_a = self.contact_normal * inv_mass;
        let impulse_magnitude = Vec3A::splat(delta_impulse);

        deltas.linear += linear_component_a * impulse_magnitude;
        deltas.angular += self.angular_component * impulse_magnitude;

        delta_impulse / self.jac_diag_ab_inv
    }

    fn resolve_single_constraint_row_generic(
        &mut self,
        deltas: &mut VelocityPair,
        inv_mass: f32,
    ) -> f32 {
        let applied_impulse = self.applied_impulse;

        let mut delta_impulse = self.rhs;
        let delta_vel_1_dot_n =
            self.contact_normal.dot(deltas.linear) + self.rel_pos_cross_normal.dot(deltas.angular);
        delta_impulse -= delta_vel_1_dot_n * self.jac_diag_ab_inv;

        let sum = applied_impulse + delta_impulse;
        delta_impulse = if sum < self.lower_limit {
            self.lower_limit - applied_impulse
        } else {
            delta_impulse
        };

        self.applied_impulse = sum.max(self.lower_limit);

        let upper_min_applied = self.upper_limit - applied_impulse;
        delta_impulse = if sum < self.upper_limit {
            delta_impulse
        } else {
            upper_min_applied
        };

        self.applied_impulse = if sum < self.upper_limit {
            self.applied_impulse
        } else {
            self.upper_limit
        };

        let linear_component_a = self.contact_normal * inv_mass;
        let impulse_magnitude = Vec3A::splat(delta_impulse);

        deltas.linear += linear_component_a * impulse_magnitude;
        deltas.angular += self.angular_component * impulse_magnitude;

        delta_impulse / self.jac_diag_ab_inv
    }

    fn resolve_split_penetration_impulse(
        &mut self,
        velocities: &mut VelocityPair,
        inv_mass: f32,
    ) -> f32 {
        if self.rhs_penetration == 0. {
            return 0.;
        }

        let mut delta_impulse = self.rhs_penetration;

        let delta_vel_1_dot_n = self.contact_normal.dot(velocities.linear)
            + self.rel_pos_cross_normal.dot(velocities.angular);
        delta_impulse -= delta_vel_1_dot_n * self.jac_diag_ab_inv;

        let sum = self.applied_push_impulse + delta_impulse;
        delta_impulse = if sum < self.lower_limit {
            self.lower_limit - self.applied_push_impulse
        } else {
            delta_impulse
        };

        self.applied_push_impulse = sum.max(self.lower_limit);

        let linear_component_a = self.contact_normal * inv_mass;
        let impulse_magnitude = Vec3A::splat(delta_impulse);

        velocities.linear += linear_component_a * impulse_magnitude;
        velocities.angular += self.angular_component * impulse_magnitude;

        delta_impulse / self.jac_diag_ab_inv
    }
}

#[derive(Clone, Debug)]
pub struct ConstraintPair {
    pub contact: Constraint,
    pub friction: Constraint,
}

impl ConstraintPair {
    fn solve_single_iteration(&mut self, deltas: &mut VelocityPair, inv_mass: f32) -> f32 {
        let residual = self
            .contact
            .resolve_single_constraint_row_lower_limit(deltas, inv_mass);
        let mut least_squares_residual = residual * residual;

        let total_impulse = self.contact.applied_impulse;
        if total_impulse > 0. {
            self.friction.lower_limit = -COEFF_FRICTION * total_impulse;
            self.friction.upper_limit = COEFF_FRICTION * total_impulse;

            let residual = self
                .friction
                .resolve_single_constraint_row_generic(deltas, inv_mass);
            least_squares_residual = least_squares_residual.max(residual * residual);
        }

        least_squares_residual
    }
}

#[derive(Clone, Copy, Debug)]
pub struct VelocityPair {
    pub linear: Vec3A,
    pub angular: Vec3A,
}

impl VelocityPair {
    pub const ZERO: Self = Self::new(Vec3A::ZERO, Vec3A::ZERO);

    #[inline]
    #[must_use]
    pub const fn new(linear: Vec3A, angular: Vec3A) -> Self {
        Self { linear, angular }
    }
}

#[derive(Clone, Debug)]
pub struct Constraints {
    contacts: ArrayVec<Constraint, { Self::MAX_CONTACTS }>,
    normal_sum: Vec3A,
    depth_sum: f32,
    count: u8,
    inv_mass: f32,
    external_force_impulse: Vec3A,
}

impl Constraints {
    pub const MAX_CONTACTS: usize = 4;

    const NUM_ITERATIONS: usize = 10;
    const RELAXATION: f32 = 1.;
    const ERP2: f32 = 0.8;

    #[inline]
    #[must_use]
    pub const fn new(inv_mass: f32, external_force_impulse: Vec3A) -> Self {
        Self {
            contacts: const { ArrayVec::new_const() },
            normal_sum: Vec3A::ZERO,
            depth_sum: 0.,
            count: 0,
            inv_mass,
            external_force_impulse,
        }
    }

    pub fn add_contacts(
        &mut self,
        contacts: ArrayVec<Contact, { Self::MAX_CONTACTS }>,
        ball: &Ball,
        dt: f32,
    ) {
        for contact in contacts {
            self.normal_sum += contact.triangle_normal;
            self.depth_sum += contact.local_position.length();
            self.count += 1;
            self.contacts
                .push(self.setup_contact_constraint(ball, contact, dt));
        }
    }

    fn setup_contact_constraint(&self, ball: &Ball, contact: Contact, dt: f32) -> Constraint {
        let torque_axis = contact.local_position.cross(contact.triangle_normal);
        let angular_component = ball.inv_inertia * torque_axis;
        let vec = angular_component.cross(contact.local_position);

        let denom = self.inv_mass + contact.triangle_normal.dot(vec);
        let jac_diag_ab_inv = Self::RELAXATION / denom;
        let rel_pos_cross_normal = torque_axis;

        let abs_vel = ball.get_velocity_in_local_point(contact.local_position);
        let rel_vel = contact.triangle_normal.dot(abs_vel);
        let restitution = Game::restitution_curve(rel_vel);

        let rel_vel = contact
            .triangle_normal
            .dot(ball.velocity + self.external_force_impulse)
            + rel_pos_cross_normal.dot(ball.angular_velocity);

        let velocity_error = restitution - rel_vel;

        let penetration_impulse = if contact.depth > 0. {
            0.
        } else {
            -contact.depth * Self::ERP2 / dt * jac_diag_ab_inv
        };

        let velocity_impulse = velocity_error * jac_diag_ab_inv;

        Constraint {
            contact_normal: contact.triangle_normal,
            rel_pos_cross_normal,
            angular_component,
            rhs: velocity_impulse,
            rhs_penetration: penetration_impulse,
            lower_limit: 0.,
            upper_limit: 1e10,
            jac_diag_ab_inv,
            applied_impulse: 0.,
            applied_push_impulse: 0.,
        }
    }

    fn setup_special_contact_constraint(
        &self,
        ball: &Ball,
        normal_world_on_b: Vec3A,
        rel_pos: Vec3A,
    ) -> Constraint {
        let torque_axis = rel_pos.cross(normal_world_on_b);
        let angular_component = ball.inv_inertia * torque_axis;
        let vec = angular_component.cross(rel_pos);

        let denom = self.inv_mass + normal_world_on_b.dot(vec);
        let jac_diag_ab_inv = Self::RELAXATION / denom;

        let abs_vel = ball.get_velocity_in_local_point(rel_pos);
        let rel_vel = normal_world_on_b.dot(abs_vel);

        let restitution = Game::restitution_curve(rel_vel);

        let rel_vel = normal_world_on_b.dot(ball.velocity + self.external_force_impulse)
            + torque_axis.dot(ball.angular_velocity);

        let velocity_error = restitution - rel_vel;
        let velocity_impulse = velocity_error * jac_diag_ab_inv;

        Constraint {
            contact_normal: normal_world_on_b,
            rel_pos_cross_normal: torque_axis,
            angular_component,
            rhs: velocity_impulse,
            rhs_penetration: 0.,
            lower_limit: 0.,
            upper_limit: 1e10,
            jac_diag_ab_inv,
            applied_impulse: 0.,
            applied_push_impulse: 0.,
        }
    }

    fn setup_friction_constraint(
        &self,
        ball: &Ball,
        normal_axis: Vec3A,
        rel_pos: Vec3A,
    ) -> Constraint {
        let contact_normal = normal_axis;
        let rel_pos_cross_normal = rel_pos.cross(contact_normal);
        let angular_component = ball.inv_inertia * rel_pos_cross_normal;

        let vec = angular_component.cross(rel_pos);
        let denom: f32 = self.inv_mass + normal_axis.dot(vec);

        let jac_diag_ab_inv = Self::RELAXATION / denom;

        let rel_vel = contact_normal.dot(ball.velocity + self.external_force_impulse)
            + rel_pos_cross_normal.dot(ball.angular_velocity);
        let velocity_error = -rel_vel;
        let velocity_impulse = velocity_error * jac_diag_ab_inv;

        Constraint {
            contact_normal,
            rel_pos_cross_normal,
            angular_component,
            rhs: velocity_impulse,
            rhs_penetration: 0.,
            lower_limit: -COEFF_FRICTION,
            upper_limit: COEFF_FRICTION,
            jac_diag_ab_inv,
            applied_impulse: 0.,
            applied_push_impulse: 0.,
        }
    }

    fn process_special_contact(&self, ball: &Ball) -> (ConstraintPair, Vec3A) {
        debug_assert!(self.count > 0);

        let count = Vec3A::splat(f32::from(self.count));
        let average_normal = self.normal_sum / count;
        let average_distance = self.depth_sum / count;

        let rel_pos = average_normal * -average_distance;
        let vel = self.external_force_impulse + ball.get_velocity_in_local_point(rel_pos);
        let rel_vel = average_normal.dot(vel);

        let contact_constraint =
            self.setup_special_contact_constraint(ball, average_normal, rel_pos);

        let mut lateral_friction_dir = vel - average_normal * rel_vel;
        let lat_rel_vel = lateral_friction_dir.length_squared();

        if lat_rel_vel > f32::EPSILON {
            lateral_friction_dir /= lat_rel_vel.sqrt();
        } else {
            lateral_friction_dir = Game::plane_space_1(average_normal);
        }

        let friction_constraint =
            self.setup_friction_constraint(ball, lateral_friction_dir, rel_pos);

        (
            ConstraintPair {
                contact: contact_constraint,
                friction: friction_constraint,
            },
            average_normal,
        )
    }

    #[must_use]
    pub fn solve(mut self, ball: &Ball) -> (VelocityPair, Vec3A, Vec3A) {
        let (mut average_constraint, normal) = self.process_special_contact(ball);
        let velocities = self.solve_split_impulse_iterations();

        let mut deltas = VelocityPair::ZERO;
        for _ in 0..Self::NUM_ITERATIONS {
            let least_squares_residual =
                average_constraint.solve_single_iteration(&mut deltas, self.inv_mass);

            if least_squares_residual <= f32::EPSILON {
                break;
            }
        }

        (deltas, velocities, normal)
    }

    fn solve_split_impulse_iterations(&mut self) -> Vec3A {
        let mut velocities = VelocityPair::ZERO;
        let mut should_runs = const { [true; Self::MAX_CONTACTS] };

        for _ in 0..Self::NUM_ITERATIONS {
            let mut any_run_next = false;

            for (should_run, contact) in should_runs.iter_mut().zip(&mut self.contacts) {
                if !*should_run {
                    continue;
                }

                let residual =
                    contact.resolve_split_penetration_impulse(&mut velocities, self.inv_mass);

                if residual * residual <= f32::EPSILON {
                    *should_run = false;
                } else {
                    any_run_next = true;
                }
            }

            if !any_run_next {
                break;
            }
        }

        // we skip calculating the ball's rotation so just return the linear velocity
        velocities.linear
    }
}
