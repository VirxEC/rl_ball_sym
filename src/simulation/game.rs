//! All the data about the game to simulate it.

use super::tri_bvh::TriangleBvh;
use glam::Vec3A;
use std::f32::consts::FRAC_1_SQRT_2;

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
    pub(crate) fn restitution_curve(rel_vel: f32, restitution: f32, velocity_threshold: f32) -> f32 {
        if rel_vel.abs() < velocity_threshold {
            0.
        } else {
            restitution * -rel_vel
        }
    }
}

#[derive(Clone, Debug)]
pub(crate) struct Constraint {
    pub contact_normal_1: Vec3A,
    pub rel_pos1_cross_normal: Vec3A,
    pub angular_component_a: Vec3A,
    pub rhs: f32,
    pub lower_limit: f32,
    pub upper_limit: f32,
    pub jac_diag_ab_inv: f32,
    pub applied_impulse: f32,
}

#[derive(Clone, Debug)]
pub(crate) struct Constraints {
    contact: Vec<Constraint>,
    friction: Vec<Constraint>,
    inv_mass: f32,
    coeff_friction: f32,
}

impl Constraints {
    pub const MAX_CONTACTS: usize = 4;

    const NUM_ITERATIONS: usize = 10;

    #[inline]
    #[must_use]
    pub fn new(inv_mass: f32, coeff_friction: f32) -> Self {
        Self {
            contact: Vec::with_capacity(Self::MAX_CONTACTS),
            friction: Vec::with_capacity(Self::MAX_CONTACTS),
            inv_mass,
            coeff_friction,
        }
    }

    pub fn push(&mut self, (contact_constraint, friction_constraint): (Constraint, Constraint)) {
        self.contact.push(contact_constraint);
        self.friction.push(friction_constraint);
    }

    // called by resolveSingleConstraintRowLowerLimit
    // static btScalar gResolveSingleConstraintRowLowerLimit_sse2(btSolverBody& bodyA, btSolverBody& bodyB, const btSolverConstraint& c)
    fn resolve_single_constraint_row_lower_limit(
        constraint: &mut Constraint,
        deltas: &mut (Vec3A, Vec3A),
        inv_mass: f32,
    ) -> f32 {
        // m_cfm is 0
        // __m128 cpAppliedImp = _mm_set1_ps(c.m_appliedImpulse);
        // __m128 lowerLimit1 = _mm_set1_ps(c.m_lowerLimit);
        // __m128 upperLimit1 = _mm_set1_ps(c.m_upperLimit);
        // btSimdScalar deltaImpulse = _mm_sub_ps(_mm_set1_ps(c.m_rhs), _mm_mul_ps(_mm_set1_ps(c.m_appliedImpulse), _mm_set1_ps(c.m_cfm)));
        let mut delta_impulse = constraint.rhs - 0.;
        // __m128 deltaVel1Dotn = _mm_add_ps(btSimdDot3(c.m_contactNormal1.mVec128, bodyA.internalGetDeltaLinearVelocity().mVec128), btSimdDot3(c.m_relpos1CrossNormal.mVec128, bodyA.internalGetDeltaAngularVelocity().mVec128));
        let delta_vel_1_dot_n = constraint.contact_normal_1.dot(deltas.0) + constraint.rel_pos1_cross_normal.dot(deltas.1);
        // __m128 deltaVel2Dotn = _mm_add_ps(btSimdDot3(c.m_contactNormal2.mVec128, bodyB.internalGetDeltaLinearVelocity().mVec128), btSimdDot3(c.m_relpos2CrossNormal.mVec128, bodyB.internalGetDeltaAngularVelocity().mVec128));
        // let delta_vel_2_dot_n = constraint.contact_normal_2.dot(deltas.0) + constraint.rel_pos2_cross_normal.dot(deltas.1);
        // deltaImpulse = _mm_sub_ps(deltaImpulse, _mm_mul_ps(deltaVel1Dotn, _mm_set1_ps(c.m_jacDiagABInv)));
        delta_impulse -= delta_vel_1_dot_n * constraint.jac_diag_ab_inv;
        // deltaImpulse = _mm_sub_ps(deltaImpulse, _mm_mul_ps(deltaVel2Dotn, _mm_set1_ps(c.m_jacDiagABInv)));
        // delta_impulse -= delta_vel_2_dot_n * constraint.jac_diag_ab_inv;
        // btSimdScalar sum = _mm_add_ps(cpAppliedImp, deltaImpulse);
        let sum = constraint.applied_impulse + delta_impulse;
        // btSimdScalar resultLowerLess, resultUpperLess;
        // resultLowerLess = _mm_cmplt_ps(sum, lowerLimit1);
        // resultUpperLess = _mm_cmplt_ps(sum, upperLimit1);
        // __m128 lowMinApplied = _mm_sub_ps(lowerLimit1, cpAppliedImp);
        let low_min_applied = constraint.lower_limit - constraint.applied_impulse;
        // deltaImpulse = _mm_or_ps(_mm_and_ps(resultLowerLess, lowMinApplied), _mm_andnot_ps(resultLowerLess, deltaImpulse));
        delta_impulse = if sum < constraint.lower_limit {
            low_min_applied
        } else {
            delta_impulse
        };
        // c.m_appliedImpulse = _mm_or_ps(_mm_and_ps(resultLowerLess, lowerLimit1), _mm_andnot_ps(resultLowerLess, sum));
        constraint.applied_impulse = sum.max(constraint.lower_limit);
        // __m128 linearComponentA = _mm_mul_ps(c.m_contactNormal1.mVec128, bodyA.internalGetInvMass().mVec128);
        let linear_component_a = constraint.contact_normal_1 * inv_mass;
        // __m128 linearComponentB = _mm_mul_ps(c.m_contactNormal2.mVec128, bodyB.internalGetInvMass().mVec128);
        // let linear_component_b = constraint.contact_normal_2 * Self::INV_M;
        // __m128 impulseMagnitude = deltaImpulse;
        let impulse_magnitude = delta_impulse;
        // bodyA.internalGetDeltaLinearVelocity().mVec128 = _mm_add_ps(bodyA.internalGetDeltaLinearVelocity().mVec128, _mm_mul_ps(linearComponentA, impulseMagnitude));
        deltas.0 += linear_component_a * impulse_magnitude;
        // bodyA.internalGetDeltaAngularVelocity().mVec128 = _mm_add_ps(bodyA.internalGetDeltaAngularVelocity().mVec128, _mm_mul_ps(c.m_angularComponentA.mVec128, impulseMagnitude));
        deltas.1 += constraint.angular_component_a * impulse_magnitude;
        // bodyB.internalGetDeltaLinearVelocity().mVec128 = _mm_add_ps(bodyB.internalGetDeltaLinearVelocity().mVec128, _mm_mul_ps(linearComponentB, impulseMagnitude));
        // bodyB.internalGetDeltaAngularVelocity().mVec128 = _mm_add_ps(bodyB.internalGetDeltaAngularVelocity().mVec128, _mm_mul_ps(c.m_angularComponentB.mVec128, impulseMagnitude));
        // return deltaImpulse.m_floats[0] / c.m_jacDiagABInv;
        delta_impulse / constraint.jac_diag_ab_inv
    }

    // called by resolveSingleConstraintRowGeneric
    // static btScalar gResolveSingleConstraintRowGeneric_sse2(btSolverBody& bodyA, btSolverBody& bodyB, const btSolverConstraint& c)
    fn resolve_single_constraint_row_generic(
        constraint: &mut Constraint,
        deltas: &mut (Vec3A, Vec3A),
        inv_mass: f32,
    ) -> f32 {
        // cfm is still 0
        // __m128 cpAppliedImp = _mm_set1_ps(c.m_appliedImpulse);
        // __m128 lowerLimit1 = _mm_set1_ps(c.m_lowerLimit);
        // __m128 upperLimit1 = _mm_set1_ps(c.m_upperLimit);
        // btSimdScalar deltaImpulse = _mm_sub_ps(_mm_set1_ps(c.m_rhs), _mm_mul_ps(_mm_set1_ps(c.m_appliedImpulse), _mm_set1_ps(c.m_cfm)));
        let mut delta_impulse = constraint.rhs - 0.;
        // __m128 deltaVel1Dotn = _mm_add_ps(btSimdDot3(c.m_contactNormal1.mVec128, bodyA.internalGetDeltaLinearVelocity().mVec128), btSimdDot3(c.m_relpos1CrossNormal.mVec128, bodyA.internalGetDeltaAngularVelocity().mVec128));
        let delta_vel_1_dot_n = constraint.contact_normal_1.dot(deltas.0) + constraint.rel_pos1_cross_normal.dot(deltas.1);
        // __m128 deltaVel2Dotn = _mm_add_ps(btSimdDot3(c.m_contactNormal2.mVec128, bodyB.internalGetDeltaLinearVelocity().mVec128), btSimdDot3(c.m_relpos2CrossNormal.mVec128, bodyB.internalGetDeltaAngularVelocity().mVec128));
        // let delta_vel_2_dot_n = constraint.contact_normal_2.dot(deltas.0) + constraint.rel_pos2_cross_normal.dot(deltas.1);
        // assert_eq!(delta_vel_2_dot_n, 0.);
        // deltaImpulse = _mm_sub_ps(deltaImpulse, _mm_mul_ps(deltaVel1Dotn, _mm_set1_ps(c.m_jacDiagABInv)));
        delta_impulse -= delta_vel_1_dot_n * constraint.jac_diag_ab_inv;
        // deltaImpulse = _mm_sub_ps(deltaImpulse, _mm_mul_ps(deltaVel2Dotn, _mm_set1_ps(c.m_jacDiagABInv)));
        // delta_impulse -= delta_vel_2_dot_n * constraint.jac_diag_ab_inv;
        // btSimdScalar sum = _mm_add_ps(cpAppliedImp, deltaImpulse);
        let sum = constraint.applied_impulse + delta_impulse;
        // btSimdScalar resultLowerLess, resultUpperLess;
        // resultLowerLess = _mm_cmplt_ps(sum, lowerLimit1);
        // resultUpperLess = _mm_cmplt_ps(sum, upperLimit1);
        // __m128 lowMinApplied = _mm_sub_ps(lowerLimit1, cpAppliedImp);
        let low_min_applied = constraint.lower_limit - constraint.applied_impulse;
        // deltaImpulse = _mm_or_ps(_mm_and_ps(resultLowerLess, lowMinApplied), _mm_andnot_ps(resultLowerLess, deltaImpulse));
        delta_impulse = if sum < constraint.lower_limit {
            low_min_applied
        } else {
            delta_impulse
        };
        // c.m_appliedImpulse = _mm_or_ps(_mm_and_ps(resultLowerLess, lowerLimit1), _mm_andnot_ps(resultLowerLess, sum));
        constraint.applied_impulse = sum.max(constraint.lower_limit);
        // __m128 upperMinApplied = _mm_sub_ps(upperLimit1, cpAppliedImp);
        let upper_min_applied = constraint.upper_limit - constraint.applied_impulse;
        // deltaImpulse = _mm_or_ps(_mm_and_ps(resultUpperLess, deltaImpulse), _mm_andnot_ps(resultUpperLess, upperMinApplied));
        delta_impulse = if sum < constraint.upper_limit {
            delta_impulse
        } else {
            upper_min_applied
        };
        // c.m_appliedImpulse = _mm_or_ps(_mm_and_ps(resultUpperLess, c.m_appliedImpulse), _mm_andnot_ps(resultUpperLess, upperLimit1));
        constraint.applied_impulse = if sum < constraint.upper_limit {
            constraint.applied_impulse
        } else {
            constraint.upper_limit
        };
        // __m128 linearComponentA = _mm_mul_ps(c.m_contactNormal1.mVec128, bodyA.internalGetInvMass().mVec128);
        let linear_component_a = constraint.contact_normal_1 * inv_mass;
        // __m128 linearComponentB = _mm_mul_ps((c.m_contactNormal2).mVec128, bodyB.internalGetInvMass().mVec128);
        // let linear_component_b = constraint.contact_normal_2 * Self::INV_M;
        // __m128 impulseMagnitude = deltaImpulse;
        let impulse_magnitude = delta_impulse;
        // bodyA.internalGetDeltaLinearVelocity().mVec128 = _mm_add_ps(bodyA.internalGetDeltaLinearVelocity().mVec128, _mm_mul_ps(linearComponentA, impulseMagnitude));
        deltas.0 += linear_component_a * impulse_magnitude;
        // bodyA.internalGetDeltaAngularVelocity().mVec128 = _mm_add_ps(bodyA.internalGetDeltaAngularVelocity().mVec128, _mm_mul_ps(c.m_angularComponentA.mVec128, impulseMagnitude));
        deltas.1 += constraint.angular_component_a * impulse_magnitude;
        // bodyB.internalGetDeltaLinearVelocity().mVec128 = _mm_add_ps(bodyB.internalGetDeltaLinearVelocity().mVec128, _mm_mul_ps(linearComponentB, impulseMagnitude));
        // bodyB.internalGetDeltaAngularVelocity().mVec128 = _mm_add_ps(bodyB.internalGetDeltaAngularVelocity().mVec128, _mm_mul_ps(c.m_angularComponentB.mVec128, impulseMagnitude));
        // return deltaImpulse.m_floats[0] / c.m_jacDiagABInv;
        delta_impulse / constraint.jac_diag_ab_inv
    }

    fn solve_single_iteration(&mut self, deltas: &mut (Vec3A, Vec3A)) -> f32 {
        let mut least_squares_residual = 0f32;

        for contact_constraint in &mut self.contact {
            let residual = Self::resolve_single_constraint_row_lower_limit(contact_constraint, deltas, self.inv_mass);
            least_squares_residual = least_squares_residual.max(residual * residual);
        }

        for (contact_constraint, friction_constraint) in self.contact.iter().zip(&mut self.friction) {
            let total_impulse = contact_constraint.applied_impulse;

            if total_impulse > 0. {
                friction_constraint.lower_limit = -self.coeff_friction * total_impulse;
                friction_constraint.upper_limit = self.coeff_friction * total_impulse;

                let residual = Self::resolve_single_constraint_row_generic(friction_constraint, deltas, self.inv_mass);
                least_squares_residual = least_squares_residual.max(residual * residual);
            }
        }

        least_squares_residual
    }

    #[must_use]
    pub fn solve(mut self) -> (Vec3A, Vec3A) {
        let mut deltas = (Vec3A::ZERO, Vec3A::ZERO);

        for _ in 0..Self::NUM_ITERATIONS {
            let least_squares_residual = self.solve_single_iteration(&mut deltas);

            if least_squares_residual <= f32::EPSILON {
                break;
            }
        }

        deltas
    }
}
