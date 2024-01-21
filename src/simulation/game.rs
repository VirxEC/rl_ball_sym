//! All the data about the game to simulate it.

use super::{ball::Ball, geometry::Contact, tri_bvh::TriangleBvh};
use combo_vec::ReArr;
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
    // called by resolveSingleConstraintRowLowerLimit
    // static btScalar gResolveSingleConstraintRowLowerLimit_sse2(btSolverBody& bodyA, btSolverBody& bodyB, const btSolverConstraint& c)
    fn resolve_single_constraint_row_lower_limit(&mut self, deltas: &mut VelocityPair, inv_mass: f32) -> f32 {
        // m_cfm is 0
        // __m128 cpAppliedImp = _mm_set1_ps(c.m_appliedImpulse);
        // __m128 lowerLimit1 = _mm_set1_ps(c.m_lowerLimit);
        // __m128 upperLimit1 = _mm_set1_ps(c.m_upperLimit);
        // btSimdScalar deltaImpulse = _mm_sub_ps(_mm_set1_ps(c.m_rhs), _mm_mul_ps(_mm_set1_ps(c.m_appliedImpulse), _mm_set1_ps(c.m_cfm)));
        let mut delta_impulse: f32 = self.rhs - 0.;
        // __m128 deltaVel1Dotn = _mm_add_ps(btSimdDot3(c.m_contactNormal1.mVec128, bodyA.internalGetDeltaLinearVelocity().mVec128), btSimdDot3(c.m_relpos1CrossNormal.mVec128, bodyA.internalGetDeltaAngularVelocity().mVec128));
        let delta_vel_1_dot_n = self.contact_normal.dot(deltas.linear) + self.rel_pos_cross_normal.dot(deltas.angular);
        // __m128 deltaVel2Dotn = _mm_add_ps(btSimdDot3(c.m_contactNormal2.mVec128, bodyB.internalGetDeltaLinearVelocity().mVec128), btSimdDot3(c.m_relpos2CrossNormal.mVec128, bodyB.internalGetDeltaAngularVelocity().mVec128));
        // let delta_vel_2_dot_n = constraint.contact_normal_2.dot(deltas.0) + constraint.rel_pos2_cross_normal.dot(deltas.1);
        // deltaImpulse = _mm_sub_ps(deltaImpulse, _mm_mul_ps(deltaVel1Dotn, _mm_set1_ps(c.m_jacDiagABInv)));
        delta_impulse -= delta_vel_1_dot_n * self.jac_diag_ab_inv;
        // deltaImpulse = _mm_sub_ps(deltaImpulse, _mm_mul_ps(deltaVel2Dotn, _mm_set1_ps(c.m_jacDiagABInv)));
        // delta_impulse -= delta_vel_2_dot_n * constraint.jac_diag_ab_inv;
        // btSimdScalar sum = _mm_add_ps(cpAppliedImp, deltaImpulse);
        let sum = self.applied_impulse + delta_impulse;
        // btSimdScalar resultLowerLess, resultUpperLess;
        // resultLowerLess = _mm_cmplt_ps(sum, lowerLimit1);
        // resultUpperLess = _mm_cmplt_ps(sum, upperLimit1);
        // __m128 lowMinApplied = _mm_sub_ps(lowerLimit1, cpAppliedImp);
        let low_min_applied = self.lower_limit - self.applied_impulse;
        // deltaImpulse = _mm_or_ps(_mm_and_ps(resultLowerLess, lowMinApplied), _mm_andnot_ps(resultLowerLess, deltaImpulse));
        delta_impulse = if sum < self.lower_limit {
            low_min_applied
        } else {
            delta_impulse
        };
        // c.m_appliedImpulse = _mm_or_ps(_mm_and_ps(resultLowerLess, lowerLimit1), _mm_andnot_ps(resultLowerLess, sum));
        self.applied_impulse = sum.max(self.lower_limit);
        // __m128 linearComponentA = _mm_mul_ps(c.m_contactNormal1.mVec128, bodyA.internalGetInvMass().mVec128);
        let linear_component_a = self.contact_normal * inv_mass;
        // __m128 linearComponentB = _mm_mul_ps(c.m_contactNormal2.mVec128, bodyB.internalGetInvMass().mVec128);
        // let linear_component_b = constraint.contact_normal_2 * Self::INV_M;
        // __m128 impulseMagnitude = deltaImpulse;
        let impulse_magnitude = Vec3A::splat(delta_impulse);
        // bodyA.internalGetDeltaLinearVelocity().mVec128 = _mm_add_ps(bodyA.internalGetDeltaLinearVelocity().mVec128, _mm_mul_ps(linearComponentA, impulseMagnitude));
        deltas.linear += linear_component_a * impulse_magnitude;
        // bodyA.internalGetDeltaAngularVelocity().mVec128 = _mm_add_ps(bodyA.internalGetDeltaAngularVelocity().mVec128, _mm_mul_ps(c.m_angularComponentA.mVec128, impulseMagnitude));
        deltas.angular += self.angular_component * impulse_magnitude;
        // bodyB.internalGetDeltaLinearVelocity().mVec128 = _mm_add_ps(bodyB.internalGetDeltaLinearVelocity().mVec128, _mm_mul_ps(linearComponentB, impulseMagnitude));
        // bodyB.internalGetDeltaAngularVelocity().mVec128 = _mm_add_ps(bodyB.internalGetDeltaAngularVelocity().mVec128, _mm_mul_ps(c.m_angularComponentB.mVec128, impulseMagnitude));
        // return deltaImpulse.m_floats[0] / c.m_jacDiagABInv;
        delta_impulse / self.jac_diag_ab_inv
    }

    // called by resolveSingleConstraintRowGeneric
    // static btScalar gResolveSingleConstraintRowGeneric_sse2(btSolverBody& bodyA, btSolverBody& bodyB, const btSolverConstraint& c)
    fn resolve_single_constraint_row_generic(&mut self, deltas: &mut VelocityPair, inv_mass: f32) -> f32 {
        // cfm is still 0
        // __m128 cpAppliedImp = _mm_set1_ps(c.m_appliedImpulse);
        let applied_impulse = self.applied_impulse;
        // __m128 lowerLimit1 = _mm_set1_ps(c.m_lowerLimit);
        // __m128 upperLimit1 = _mm_set1_ps(c.m_upperLimit);
        // btSimdScalar deltaImpulse = _mm_sub_ps(_mm_set1_ps(c.m_rhs), _mm_mul_ps(_mm_set1_ps(c.m_appliedImpulse), _mm_set1_ps(c.m_cfm)));
        let mut delta_impulse = self.rhs - 0.;
        // __m128 deltaVel1Dotn = _mm_add_ps(btSimdDot3(c.m_contactNormal1.mVec128, bodyA.internalGetDeltaLinearVelocity().mVec128), btSimdDot3(c.m_relpos1CrossNormal.mVec128, bodyA.internalGetDeltaAngularVelocity().mVec128));
        let delta_vel_1_dot_n = self.contact_normal.dot(deltas.linear) + self.rel_pos_cross_normal.dot(deltas.angular);
        // __m128 deltaVel2Dotn = _mm_add_ps(btSimdDot3(c.m_contactNormal2.mVec128, bodyB.internalGetDeltaLinearVelocity().mVec128), btSimdDot3(c.m_relpos2CrossNormal.mVec128, bodyB.internalGetDeltaAngularVelocity().mVec128));
        // let delta_vel_2_dot_n = constraint.contact_normal_2.dot(deltas.0) + constraint.rel_pos2_cross_normal.dot(deltas.1);
        // assert_eq!(delta_vel_2_dot_n, 0.);
        // deltaImpulse = _mm_sub_ps(deltaImpulse, _mm_mul_ps(deltaVel1Dotn, _mm_set1_ps(c.m_jacDiagABInv)));
        delta_impulse -= delta_vel_1_dot_n * self.jac_diag_ab_inv;
        // deltaImpulse = _mm_sub_ps(deltaImpulse, _mm_mul_ps(deltaVel2Dotn, _mm_set1_ps(c.m_jacDiagABInv)));
        // delta_impulse -= delta_vel_2_dot_n * constraint.jac_diag_ab_inv;
        // btSimdScalar sum = _mm_add_ps(cpAppliedImp, deltaImpulse);
        let sum = applied_impulse + delta_impulse;
        // btSimdScalar resultLowerLess, resultUpperLess;
        // resultLowerLess = _mm_cmplt_ps(sum, lowerLimit1);
        // resultUpperLess = _mm_cmplt_ps(sum, upperLimit1);
        // deltaImpulse = _mm_or_ps(_mm_and_ps(resultLowerLess, lowMinApplied), _mm_andnot_ps(resultLowerLess, deltaImpulse));
        delta_impulse = if sum < self.lower_limit {
            // __m128 lowMinApplied = _mm_sub_ps(lowerLimit1, cpAppliedImp);
            self.lower_limit - applied_impulse
        } else {
            delta_impulse
        };
        // c.m_appliedImpulse = _mm_or_ps(_mm_and_ps(resultLowerLess, lowerLimit1), _mm_andnot_ps(resultLowerLess, sum));
        self.applied_impulse = sum.max(self.lower_limit);
        // __m128 upperMinApplied = _mm_sub_ps(upperLimit1, cpAppliedImp);
        let upper_min_applied = self.upper_limit - applied_impulse;
        // deltaImpulse = _mm_or_ps(_mm_and_ps(resultUpperLess, deltaImpulse), _mm_andnot_ps(resultUpperLess, upperMinApplied));
        delta_impulse = if sum < self.upper_limit {
            delta_impulse
        } else {
            upper_min_applied
        };
        // c.m_appliedImpulse = _mm_or_ps(_mm_and_ps(resultUpperLess, c.m_appliedImpulse), _mm_andnot_ps(resultUpperLess, upperLimit1));
        self.applied_impulse = if sum < self.upper_limit {
            self.applied_impulse
        } else {
            self.upper_limit
        };
        // __m128 linearComponentA = _mm_mul_ps(c.m_contactNormal1.mVec128, bodyA.internalGetInvMass().mVec128);
        let linear_component_a = self.contact_normal * inv_mass;
        // __m128 linearComponentB = _mm_mul_ps((c.m_contactNormal2).mVec128, bodyB.internalGetInvMass().mVec128);
        // let linear_component_b = constraint.contact_normal_2 * Self::INV_M;
        // __m128 impulseMagnitude = deltaImpulse;
        let impulse_magnitude = Vec3A::splat(delta_impulse);
        // bodyA.internalGetDeltaLinearVelocity().mVec128 = _mm_add_ps(bodyA.internalGetDeltaLinearVelocity().mVec128, _mm_mul_ps(linearComponentA, impulseMagnitude));
        deltas.linear += linear_component_a * impulse_magnitude;
        // bodyA.internalGetDeltaAngularVelocity().mVec128 = _mm_add_ps(bodyA.internalGetDeltaAngularVelocity().mVec128, _mm_mul_ps(c.m_angularComponentA.mVec128, impulseMagnitude));
        deltas.angular += self.angular_component * impulse_magnitude;
        // bodyB.internalGetDeltaLinearVelocity().mVec128 = _mm_add_ps(bodyB.internalGetDeltaLinearVelocity().mVec128, _mm_mul_ps(linearComponentB, impulseMagnitude));
        // bodyB.internalGetDeltaAngularVelocity().mVec128 = _mm_add_ps(bodyB.internalGetDeltaAngularVelocity().mVec128, _mm_mul_ps(c.m_angularComponentB.mVec128, impulseMagnitude));
        // return deltaImpulse.m_floats[0] / c.m_jacDiagABInv;
        delta_impulse / self.jac_diag_ab_inv
    }

    // static btScalar gResolveSplitPenetrationImpulse_sse2(btSolverBody& bodyA, btSolverBody& bodyB, const btSolverConstraint& c)
    fn resolve_split_penetration_impulse(&mut self, velocities: &mut VelocityPair, inv_mass: f32) -> f32 {
        // if (!c.m_rhsPenetration)
        //     return 0.f;
        if self.rhs_penetration == 0. {
            return 0.;
        }

        // dbg!(self.rhs_penetration / 50.);
        // gNumSplitImpulseRecoveries++;

        // __m128 cpAppliedImp = _mm_set1_ps(c.m_appliedPushImpulse);
        // __m128 lowerLimit1 = _mm_set1_ps(c.m_lowerLimit);
        // __m128 upperLimit1 = _mm_set1_ps(c.m_upperLimit);
        // __m128 deltaImpulse = _mm_sub_ps(_mm_set1_ps(c.m_rhsPenetration), _mm_mul_ps(_mm_set1_ps(c.m_appliedPushImpulse), _mm_set1_ps(c.m_cfm)));
        let mut delta_impulse = self.rhs_penetration - 0.;

        // __m128 deltaVel1Dotn = _mm_add_ps(btSimdDot3(c.m_contactNormal1.mVec128, bodyA.internalGetPushVelocity().mVec128), btSimdDot3(c.m_relpos1CrossNormal.mVec128, bodyA.internalGetTurnVelocity().mVec128));
        let delta_vel_1_dot_n =
            self.contact_normal.dot(velocities.linear) + self.rel_pos_cross_normal.dot(velocities.angular);
        // __m128 deltaVel2Dotn = _mm_add_ps(btSimdDot3(c.m_contactNormal2.mVec128, bodyB.internalGetPushVelocity().mVec128), btSimdDot3(c.m_relpos2CrossNormal.mVec128, bodyB.internalGetTurnVelocity().mVec128));

        // deltaImpulse = _mm_sub_ps(deltaImpulse, _mm_mul_ps(deltaVel1Dotn, _mm_set1_ps(c.m_jacDiagABInv)));
        delta_impulse -= delta_vel_1_dot_n * self.jac_diag_ab_inv;
        // deltaImpulse = _mm_sub_ps(deltaImpulse, _mm_mul_ps(deltaVel2Dotn, _mm_set1_ps(c.m_jacDiagABInv)));

        // btSimdScalar sum = _mm_add_ps(cpAppliedImp, deltaImpulse);
        let sum = self.applied_push_impulse + delta_impulse;

        // btSimdScalar resultLowerLess, resultUpperLess;
        // resultLowerLess = _mm_cmplt_ps(sum, lowerLimit1);
        // resultUpperLess = _mm_cmplt_ps(sum, upperLimit1);
        // deltaImpulse = _mm_or_ps(_mm_and_ps(resultLowerLess, lowMinApplied), _mm_andnot_ps(resultLowerLess, deltaImpulse));
        delta_impulse = if sum < self.lower_limit {
            // __m128 lowMinApplied = _mm_sub_ps(lowerLimit1, cpAppliedImp);
            self.lower_limit - self.applied_push_impulse
        } else {
            delta_impulse
        };

        // c.m_appliedPushImpulse = _mm_or_ps(_mm_and_ps(resultLowerLess, lowerLimit1), _mm_andnot_ps(resultLowerLess, sum));
        self.applied_push_impulse = sum.max(self.lower_limit);

        // __m128 linearComponentA = _mm_mul_ps(c.m_contactNormal1.mVec128, bodyA.internalGetInvMass().mVec128);
        let linear_component_a = self.contact_normal * inv_mass;
        // __m128 linearComponentB = _mm_mul_ps(c.m_contactNormal2.mVec128, bodyB.internalGetInvMass().mVec128);

        // __m128 impulseMagnitude = deltaImpulse;
        let impulse_magnitude = Vec3A::splat(delta_impulse);

        // bodyA.internalGetPushVelocity().mVec128 = _mm_add_ps(bodyA.internalGetPushVelocity().mVec128, _mm_mul_ps(linearComponentA, impulseMagnitude));
        velocities.linear += linear_component_a * impulse_magnitude;
        // bodyA.internalGetTurnVelocity().mVec128 = _mm_add_ps(bodyA.internalGetTurnVelocity().mVec128, _mm_mul_ps(c.m_angularComponentA.mVec128, impulseMagnitude));
        velocities.angular += self.angular_component * impulse_magnitude;
        // bodyB.internalGetPushVelocity().mVec128 = _mm_add_ps(bodyB.internalGetPushVelocity().mVec128, _mm_mul_ps(linearComponentB, impulseMagnitude));
        // bodyB.internalGetTurnVelocity().mVec128 = _mm_add_ps(bodyB.internalGetTurnVelocity().mVec128, _mm_mul_ps(c.m_angularComponentB.mVec128, impulseMagnitude));

        // btSimdScalar deltaImp = deltaImpulse;
        // return deltaImp.m_floats[0] * (1. / c.m_jacDiagABInv);
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
        let residual = self.contact.resolve_single_constraint_row_lower_limit(deltas, inv_mass);
        let mut least_squares_residual = residual * residual;

        let total_impulse = self.contact.applied_impulse;
        if total_impulse > 0. {
            self.friction.lower_limit = -COEFF_FRICTION * total_impulse;
            self.friction.upper_limit = COEFF_FRICTION * total_impulse;

            let residual = self.friction.resolve_single_constraint_row_generic(deltas, inv_mass);
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
    contacts: Vec<Constraint>,
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
            contacts: Vec::new(),
            normal_sum: Vec3A::ZERO,
            depth_sum: 0.,
            count: 0,
            inv_mass,
            external_force_impulse,
        }
    }

    pub fn add_contacts(&mut self, contacts: ReArr<Contact, { Self::MAX_CONTACTS }>, ball: &Ball, dt: f32) {
        self.contacts.reserve(contacts.len());

        for contact in contacts {
            self.normal_sum += contact.triangle_normal;
            self.depth_sum += contact.local_position.length();
            self.count += 1;
            self.contacts.push(self.setup_contact_contraint(ball, contact, dt));
        }
    }

    // void btSequentialImpulseConstraintSolver::setupContactConstraint(btSolverConstraint& solverConstraint,
    //     int solverBodyIdA, int solverBodyIdB,
    //     btManifoldPoint& cp, const btContactSolverInfo& infoGlobal,
    //     btScalar& relaxation,
    //     const btVector3& rel_pos1, const btVector3& rel_pos2)
    fn setup_contact_contraint(&self, ball: &Ball, contact: Contact, dt: f32) -> Constraint {
        // btSolverBody* bodyA = &m_tmpSolverBodyPool[solverBodyIdA];
        // btSolverBody* bodyB = &m_tmpSolverBodyPool[solverBodyIdB];

        // this is the ball
        // btRigidBody* rb0 = bodyA->m_originalBody;
        // rb1 is a null pointer
        // btRigidBody* rb1 = bodyB->m_originalBody;

        // relaxation = infoGlobal.m_sor;
        // btScalar invTimeStep = btScalar(1) / infoGlobal.m_timeStep;
        let inv_time_step = 1. / dt;

        // btScalar cfm = infoGlobal.m_globalCfm; // 0
        // let mut cfm = 0.;
        // btScalar erp = infoGlobal.m_erp2; // 0.8

        // cfm *= invTimeStep;
        // cfm *= inv_time_step;

        // btVector3 torqueAxis0 = rel_pos1.cross(cp.m_normalWorldOnB);
        // dbg!(rel_pos1 / 50., normal_world_on_b);
        let torque_axis = contact.local_position.cross(contact.triangle_normal);
        // dbg!(torque_axis_0);
        // getAngularFactor will always return 1, 1, 1
        // solverConstraint.m_angularComponentA = rb0 ? rb0->getInvInertiaTensorWorld() * torqueAxis0 * rb0->getAngularFactor() : btVector3(0, 0, 0);
        // dbg!(self.inv_inertia);
        let angular_component = ball.inv_inertia * torque_axis;
        // dbg!(angular_component_a);
        // btVector3 torqueAxis1 = rel_pos2.cross(cp.m_normalWorldOnB);
        // let torque_axis_1 = rel_pos2.cross(normal_world_on_b);
        // solverConstraint.m_angularComponentB = rb1 ? rb1->getInvInertiaTensorWorld() * -torqueAxis1 * rb1->getAngularFactor() : btVector3(0, 0, 0);
        // let angular_component_b = Vec3A::ZERO;

        // dbg!(angular_component_a);

        // btVector3 vec;
        // btScalar denom0 = 0.f;
        // btScalar denom1 = 0.f;
        // if (rb0)
        // {
        //     vec = (solverConstraint.m_angularComponentA).cross(rel_pos1);
        //     denom0 = rb0->getInvMass() + cp.m_normalWorldOnB.dot(vec);
        // }
        // if (rb1)
        // {
        //     vec = (-solverConstraint.m_angularComponentB).cross(rel_pos2);
        //     denom1 = rb1->getInvMass() + cp.m_normalWorldOnB.dot(vec);
        // }
        let vec = angular_component.cross(contact.local_position);
        let denom = self.inv_mass + contact.triangle_normal.dot(vec);
        // let denom_1 = 0.;

        // dbg!(vec);
        // dbg!(denom_0);
        // btScalar denom = relaxation / (denom0 + denom1 + cfm);
        // let denom = Self::RELAXATION / denom_0;
        // solverConstraint.m_jacDiagABInv = denom;
        let jac_diag_ab_inv = Self::RELAXATION / denom;
        // dbg!(jac_diag_ab_inv);

        // if (rb0)
        // {
        //     solverConstraint.m_contactNormal1 = cp.m_normalWorldOnB;
        //     solverConstraint.m_relpos1CrossNormal = torqueAxis0;
        // }
        // else
        // {
        //     solverConstraint.m_contactNormal1.setZero();
        //     solverConstraint.m_relpos1CrossNormal.setZero();
        // }
        let rel_pos_cross_normal = torque_axis;
        // if (rb1)
        // {
        //     solverConstraint.m_contactNormal2 = -cp.m_normalWorldOnB;
        //     solverConstraint.m_relpos2CrossNormal = -torqueAxis1;
        // }
        // else
        // {
        //     solverConstraint.m_contactNormal2.setZero();
        //     solverConstraint.m_relpos2CrossNormal.setZero();
        // }
        // let contact_normal_2 = Vec3A::ZERO;
        // let rel_pos2_cross_normal = Vec3A::ZERO;

        // btScalar restitution = 0.f;
        // btScalar penetration = cp.getDistance() + infoGlobal.m_linearSlop; // m_linearSlop is 0
        // let penetration = contact.depth;

        // btVector3 vel1, vel2;

        // vel1 = rb0 ? rb0->getVelocityInLocalPoint(rel_pos1) : btVector3(0, 0, 0);
        let abs_vel = ball.get_velocity_in_local_point(contact.local_position);
        // vel2 = rb1 ? rb1->getVelocityInLocalPoint(rel_pos2) : btVector3(0, 0, 0);
        // let vel2 = Vec3A::ZERO;

        // dbg!(vel1 / 50.);

        // btVector3 vel = vel1 - vel2;
        // let vel = vel1 - vel2;
        // btScalar rel_vel = cp.m_normalWorldOnB.dot(vel);
        let rel_vel = contact.triangle_normal.dot(abs_vel);

        // solverConstraint.m_friction = cp.m_combinedFriction;

        // infoGlobal.m_restitutionVelocityThreshold = 0.2
        // restitution = restitutionCurve(rel_vel, cp.m_combinedRestitution, infoGlobal.m_restitutionVelocityThreshold);
        // if (restitution <= btScalar(0.))
        // if restitution <= 0. {
        //     // restitution = 0.f;
        //     restitution = 0.;
        // }
        let restitution = Game::restitution_curve(rel_vel);

        // none of the below is needed because applied_impulse is always 0 and it makes everything else 0
        // appliedImpulse is 0 and m_warmstartingFactor is 0.85
        // solverConstraint.m_appliedImpulse = cp.m_appliedImpulse * infoGlobal.m_warmstartingFactor;
        let applied_impulse = 0.;

        // if (rb0)
        //     bodyA->internalApplyImpulse(solverConstraint.m_contactNormal1 * bodyA->internalGetInvMass(), solverConstraint.m_angularComponentA, solverConstraint.m_appliedImpulse);
        // if (rb1)
        //     bodyB->internalApplyImpulse(-solverConstraint.m_contactNormal2 * bodyB->internalGetInvMass(), -solverConstraint.m_angularComponentB, -(btScalar)solverConstraint.m_appliedImpulse);

        // solverConstraint.m_appliedPushImpulse = 0.f;

        // m_externalForceImpulse is the effect of gravity, everything else is always 0
        // btVector3 externalForceImpulseA = bodyA->m_originalBody ? bodyA->m_externalForceImpulse : btVector3(0, 0, 0);
        // let external_force_impulse_a = external_force_impulse;
        // btVector3 externalTorqueImpulseA = bodyA->m_originalBody ? bodyA->m_externalTorqueImpulse : btVector3(0, 0, 0);
        // let external_torque_impulse_a = Vec3A::ZERO;
        // btVector3 externalForceImpulseB = bodyB->m_originalBody ? bodyB->m_externalForceImpulse : btVector3(0, 0, 0);
        // let external_force_impulse_b = Vec3A::ZERO;
        // btVector3 externalTorqueImpulseB = bodyB->m_originalBody ? bodyB->m_externalTorqueImpulse : btVector3(0, 0, 0);
        // let external_torque_impulse_b = Vec3A::ZERO;

        // btScalar vel1Dotn = solverConstraint.m_contactNormal1.dot(bodyA->m_linearVelocity + externalForceImpulseA) + solverConstraint.m_relpos1CrossNormal.dot(bodyA->m_angularVelocity + externalTorqueImpulseA);
        let rel_vel = contact.triangle_normal.dot(ball.velocity + self.external_force_impulse)
            + rel_pos_cross_normal.dot(ball.angular_velocity);

        // btScalar vel2Dotn = solverConstraint.m_contactNormal2.dot(bodyB->m_linearVelocity + externalForceImpulseB) + solverConstraint.m_relpos2CrossNormal.dot(bodyB->m_angularVelocity + externalTorqueImpulseB);
        // let vel_2_dot_n = contact_normal_2.dot(Vec3A::ZERO + external_force_impulse_b) + rel_pos_2_cross_normal.dot(Vec3A::ZERO + external_torque_impulse_b);
        // let vel_2_dot_n = 0.;
        // btScalar rel_vel = vel1Dotn + vel2Dotn; // vel2Dotn is always 0
        // let rel_vel = vel_dot_n + vel_2_dot_n;

        // btScalar velocityError = restitution - rel_vel;
        let velocity_error = restitution - rel_vel;
        // dbg!(velocity_error / 50., restitution / 50.);

        let positional_error = if contact.depth > 0. {
            0.
        } else {
            -contact.depth * Self::ERP2 * inv_time_step
        };

        // btScalar penetrationImpulse = positionalError * solverConstraint.m_jacDiagABInv; // m_jacDiagABInv is currently about 27.94
        let penetration_impulse = positional_error * jac_diag_ab_inv;
        // btScalar velocityImpulse = velocityError * solverConstraint.m_jacDiagABInv;
        let velocity_impulse = velocity_error * jac_diag_ab_inv;
        // dbg!(penetration_impulse / 50., velocity_impulse / 50.);

        //split position and velocity into rhs and m_rhsPenetration
        // solverConstraint.m_rhs = velocityImpulse;
        // solverConstraint.m_rhsPenetration = penetrationImpulse;
        // solverConstraint.m_cfm = cfm * solverConstraint.m_jacDiagABInv;
        // solverConstraint.m_lowerLimit = 0;
        // solverConstraint.m_upperLimit = 1e10f;
        Constraint {
            contact_normal: contact.triangle_normal,
            rel_pos_cross_normal,
            angular_component,
            rhs: velocity_impulse,
            rhs_penetration: penetration_impulse,
            lower_limit: 0.,
            upper_limit: 1e10,
            jac_diag_ab_inv,
            applied_impulse,
            applied_push_impulse: 0.,
        }
    }

    fn setup_special_contact_contraint(&self, ball: &Ball, normal_world_on_b: Vec3A, rel_pos: Vec3A) -> Constraint {
        let torque_axis = rel_pos.cross(normal_world_on_b);
        let angular_component = ball.inv_inertia * torque_axis;
        let vec = angular_component.cross(rel_pos);

        let denom = self.inv_mass + normal_world_on_b.dot(vec);
        let jac_diag_ab_inv = Self::RELAXATION / denom;

        let abs_vel = ball.get_velocity_in_local_point(rel_pos);
        let rel_vel = normal_world_on_b.dot(abs_vel);

        let restitution = Game::restitution_curve(rel_vel);

        let rel_vel =
            normal_world_on_b.dot(ball.velocity + self.external_force_impulse) + torque_axis.dot(ball.angular_velocity);

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

    // called by btSequentialImpulseConstraintSolver::addFrictionConstraint
    // void btSequentialImpulseConstraintSolver::setupFrictionConstraint(btSolverConstraint& solverConstraint, const btVector3& normalAxis, int solverBodyIdA, int solverBodyIdB, btManifoldPoint& cp, const btVector3& rel_pos1, const btVector3& rel_pos2, btCollisionObject* colObj0, btCollisionObject* colObj1, btScalar relaxation, const btContactSolverInfo& infoGlobal, btScalar desiredVelocity, btScalar cfmSlip)
    fn setup_friction_constraint(&self, ball: &Ball, normal_axis: Vec3A, rel_pos: Vec3A) -> Constraint {
        // btSolverBody& solverBodyA = m_tmpSolverBodyPool[solverBodyIdA];
        // btSolverBody& solverBodyB = m_tmpSolverBodyPool[solverBodyIdB];

        // // body 0 is the ball, bodyA is a null pointer
        // btRigidBody* body0 = m_tmpSolverBodyPool[solverBodyIdA].m_originalBody;
        // btRigidBody* bodyA = m_tmpSolverBodyPool[solverBodyIdB].m_originalBody;

        // solverConstraint.m_solverBodyIdA = solverBodyIdA;
        // solverConstraint.m_solverBodyIdB = solverBodyIdB;

        // solverConstraint.m_friction = cp.m_combinedFriction;
        // solverConstraint.m_originalContactPoint = 0;

        // solverConstraint.m_appliedImpulse = 0.f;
        let applied_impulse = 0.;
        // solverConstraint.m_appliedPushImpulse = 0.f;

        // if (body0)
        // {
        //     solverConstraint.m_contactNormal1 = normalAxis;
        //     btVector3 ftorqueAxis1 = rel_pos1.cross(solverConstraint.m_contactNormal1);
        //     solverConstraint.m_relpos1CrossNormal = ftorqueAxis1;
        //     solverConstraint.m_angularComponentA = body0->getInvInertiaTensorWorld() * ftorqueAxis1 * body0->getAngularFactor();
        // }
        // else
        // {
        //     solverConstraint.m_contactNormal1.setZero();
        //     solverConstraint.m_relpos1CrossNormal.setZero();
        //     solverConstraint.m_angularComponentA.setZero();
        // }

        let contact_normal = normal_axis;
        let rel_pos_cross_normal = rel_pos.cross(contact_normal);
        let angular_component = ball.inv_inertia * rel_pos_cross_normal;

        // if (bodyA)
        // {
        //     solverConstraint.m_contactNormal2 = -normalAxis;
        //     btVector3 ftorqueAxis1 = rel_pos2.cross(solverConstraint.m_contactNormal2);
        //     solverConstraint.m_relpos2CrossNormal = ftorqueAxis1;
        //     solverConstraint.m_angularComponentB = bodyA->getInvInertiaTensorWorld() * ftorqueAxis1 * bodyA->getAngularFactor();
        // }
        // else
        // {
        //     solverConstraint.m_contactNormal2.setZero();
        //     solverConstraint.m_relpos2CrossNormal.setZero();
        //     solverConstraint.m_angularComponentB.setZero();
        // }

        // let contact_normal_2 = Vec3A::ZERO;
        // let rel_pos2_cross_normal = Vec3A::ZERO;
        // let angular_component_b = Vec3A::ZERO;

        // btVector3 vec;
        // btScalar denom0 = 0.f;
        // btScalar denom1 = 0.f;
        // if (body0)
        // {
        //     vec = (solverConstraint.m_angularComponentA).cross(rel_pos1);
        //     denom0 = body0->getInvMass() + normalAxis.dot(vec);
        // }
        let vec = angular_component.cross(rel_pos);
        let denom: f32 = self.inv_mass + normal_axis.dot(vec);

        // if (bodyA)
        // {
        //     vec = (-solverConstraint.m_angularComponentB).cross(rel_pos2);
        //     denom1 = bodyA->getInvMass() + normalAxis.dot(vec);
        // }
        // let denom_1 = 0.;

        // btScalar denom = relaxation / (denom0 + denom1);
        // let denom = Self::RELAXATION / (denom_0 + denom_1);
        // solverConstraint.m_jacDiagABInv = denom;
        let jac_diag_ab_inv = Self::RELAXATION / denom;

        // btScalar rel_vel;
        // btScalar vel1Dotn = solverConstraint.m_contactNormal1.dot(body0 ? solverBodyA.m_linearVelocity + solverBodyA.m_externalForceImpulse : btVector3(0, 0, 0)) + solverConstraint.m_relpos1CrossNormal.dot(body0 ? solverBodyA.m_angularVelocity : btVector3(0, 0, 0));
        let rel_vel = contact_normal.dot(ball.velocity + self.external_force_impulse)
            + rel_pos_cross_normal.dot(ball.angular_velocity);
        // btScalar vel2Dotn = solverConstraint.m_contactNormal2.dot(bodyA ? solverBodyB.m_linearVelocity + solverBodyB.m_externalForceImpulse : btVector3(0, 0, 0)) + solverConstraint.m_relpos2CrossNormal.dot(bodyA ? solverBodyB.m_angularVelocity : btVector3(0, 0, 0));
        // let vel_2_dot_n = 0.;

        // rel_vel = vel1Dotn + vel2Dotn; // vel2Dotn is always 0
        // let rel_vel = vel_1_dot_n + vel_2_dot_n;

        // let desired_velocity = 0.;
        // btScalar velocityError = desiredVelocity - rel_vel;
        let velocity_error = -rel_vel;
        // btScalar velocityImpulse = velocityError * solverConstraint.m_jacDiagABInv;
        let velocity_impulse = velocity_error * jac_diag_ab_inv;

        // btScalar penetrationImpulse = btScalar(0);
        // let penetration_impulse = 0.;

        // solverConstraint.m_rhs = penetrationImpulse + velocityImpulse;
        // solverConstraint.m_rhsPenetration = 0.f;
        // solverConstraint.m_cfm = cfmSlip; // 0
        // solverConstraint.m_lowerLimit = -solverConstraint.m_friction;
        // solverConstraint.m_upperLimit = solverConstraint.m_friction;
        Constraint {
            contact_normal,
            rel_pos_cross_normal,
            angular_component,
            rhs: velocity_impulse,
            rhs_penetration: 0.,
            lower_limit: -COEFF_FRICTION,
            upper_limit: COEFF_FRICTION,
            jac_diag_ab_inv,
            applied_impulse,
            applied_push_impulse: 0.,
        }
    }

    // void btSequentialImpulseConstraintSolver::convertContactSpecial(btCollisionObject& obj, const btContactSolverInfo& infoGlobal)
    fn process_special_contact(&self, ball: &Ball) -> ConstraintPair {
        debug_assert!(self.count > 0);

        let count = f32::from(self.count);
        let average_normal = self.normal_sum / count;
        let average_distance = self.depth_sum / count;

        let rel_pos = average_normal * -average_distance;
        let vel: Vec3A = ball.get_velocity_in_local_point_no_delta(rel_pos, self.external_force_impulse);
        let rel_vel = average_normal.dot(vel);

        let contact_constraint = self.setup_special_contact_contraint(ball, average_normal, rel_pos);

        let mut lateral_friction_dir = vel - average_normal * rel_vel;
        let lat_rel_vel = lateral_friction_dir.length_squared();

        if lat_rel_vel > f32::EPSILON {
            lateral_friction_dir /= lat_rel_vel.sqrt();
        } else {
            lateral_friction_dir = Game::plane_space_1(average_normal);
        }

        let friction_constraint = self.setup_friction_constraint(ball, lateral_friction_dir, rel_pos);

        ConstraintPair {
            contact: contact_constraint,
            friction: friction_constraint,
        }
    }

    #[must_use]
    pub fn solve(mut self, ball: &Ball) -> (VelocityPair, Vec3A) {
        let mut average_constraint = self.process_special_contact(ball);

        let velocities = self.solve_split_impulse_iterations();

        let mut deltas = VelocityPair::ZERO;

        for _ in 0..Self::NUM_ITERATIONS {
            let least_squares_residual = average_constraint.solve_single_iteration(&mut deltas, self.inv_mass);

            if least_squares_residual <= f32::EPSILON {
                break;
            }
        }

        (deltas, velocities)
    }

    // void btSequentialImpulseConstraintSolver::solveGroupCacheFriendlySplitImpulseIterations(btCollisionObject** bodies, int numBodies, btPersistentManifold** manifoldPtr, int numManifolds, btTypedConstraint** constraints, int numConstraints, const btContactSolverInfo& infoGlobal)
    fn solve_split_impulse_iterations(&mut self) -> Vec3A {
        let mut velocities = VelocityPair::ZERO;
        let mut should_runs = [true; Self::MAX_CONTACTS];

        for _ in 0..Self::NUM_ITERATIONS {
            let mut any_run_next = false;

            for (should_run, contact) in should_runs.iter_mut().zip(&mut self.contacts) {
                if !*should_run {
                    continue;
                }

                let residual = contact.resolve_split_penetration_impulse(&mut velocities, self.inv_mass);

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
