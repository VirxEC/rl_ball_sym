//! All the data about the game to simulate it.

use super::{ball::Ball, geometry::Contact, tri_bvh::TriangleBvh};
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
    pub contact_normal: Vec3A,
    pub rel_pos_cross_normal: Vec3A,
    pub angular_component: Vec3A,
    pub rhs: f32,
    pub lower_limit: f32,
    pub upper_limit: f32,
    pub jac_diag_ab_inv: f32,
    pub applied_impulse: f32,
}

impl Constraint {
    // called by resolveSingleConstraintRowLowerLimit
    // static btScalar gResolveSingleConstraintRowLowerLimit_sse2(btSolverBody& bodyA, btSolverBody& bodyB, const btSolverConstraint& c)
    fn resolve_single_constraint_row_lower_limit(&mut self, deltas: &mut (Vec3A, Vec3A), inv_mass: f32) -> f32 {
        // m_cfm is 0
        // __m128 cpAppliedImp = _mm_set1_ps(c.m_appliedImpulse);
        // __m128 lowerLimit1 = _mm_set1_ps(c.m_lowerLimit);
        // __m128 upperLimit1 = _mm_set1_ps(c.m_upperLimit);
        // btSimdScalar deltaImpulse = _mm_sub_ps(_mm_set1_ps(c.m_rhs), _mm_mul_ps(_mm_set1_ps(c.m_appliedImpulse), _mm_set1_ps(c.m_cfm)));
        let mut delta_impulse: f32 = self.rhs - 0.;
        // __m128 deltaVel1Dotn = _mm_add_ps(btSimdDot3(c.m_contactNormal1.mVec128, bodyA.internalGetDeltaLinearVelocity().mVec128), btSimdDot3(c.m_relpos1CrossNormal.mVec128, bodyA.internalGetDeltaAngularVelocity().mVec128));
        let delta_vel_1_dot_n = self.contact_normal.dot(deltas.0) + self.rel_pos_cross_normal.dot(deltas.1);
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
        deltas.0 += linear_component_a * impulse_magnitude;
        // bodyA.internalGetDeltaAngularVelocity().mVec128 = _mm_add_ps(bodyA.internalGetDeltaAngularVelocity().mVec128, _mm_mul_ps(c.m_angularComponentA.mVec128, impulseMagnitude));
        deltas.1 += self.angular_component * impulse_magnitude;
        // bodyB.internalGetDeltaLinearVelocity().mVec128 = _mm_add_ps(bodyB.internalGetDeltaLinearVelocity().mVec128, _mm_mul_ps(linearComponentB, impulseMagnitude));
        // bodyB.internalGetDeltaAngularVelocity().mVec128 = _mm_add_ps(bodyB.internalGetDeltaAngularVelocity().mVec128, _mm_mul_ps(c.m_angularComponentB.mVec128, impulseMagnitude));
        // return deltaImpulse.m_floats[0] / c.m_jacDiagABInv;
        delta_impulse / self.jac_diag_ab_inv
    }

    // called by resolveSingleConstraintRowGeneric
    // static btScalar gResolveSingleConstraintRowGeneric_sse2(btSolverBody& bodyA, btSolverBody& bodyB, const btSolverConstraint& c)
    fn resolve_single_constraint_row_generic(&mut self, deltas: &mut (Vec3A, Vec3A), inv_mass: f32) -> f32 {
        // cfm is still 0
        // __m128 cpAppliedImp = _mm_set1_ps(c.m_appliedImpulse);
        let applied_impulse = self.applied_impulse;
        // __m128 lowerLimit1 = _mm_set1_ps(c.m_lowerLimit);
        // __m128 upperLimit1 = _mm_set1_ps(c.m_upperLimit);
        // btSimdScalar deltaImpulse = _mm_sub_ps(_mm_set1_ps(c.m_rhs), _mm_mul_ps(_mm_set1_ps(c.m_appliedImpulse), _mm_set1_ps(c.m_cfm)));
        let mut delta_impulse = self.rhs - 0.;
        // __m128 deltaVel1Dotn = _mm_add_ps(btSimdDot3(c.m_contactNormal1.mVec128, bodyA.internalGetDeltaLinearVelocity().mVec128), btSimdDot3(c.m_relpos1CrossNormal.mVec128, bodyA.internalGetDeltaAngularVelocity().mVec128));
        let delta_vel_1_dot_n = self.contact_normal.dot(deltas.0) + self.rel_pos_cross_normal.dot(deltas.1);
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
        // __m128 lowMinApplied = _mm_sub_ps(lowerLimit1, cpAppliedImp);
        let low_min_applied = self.lower_limit - applied_impulse;
        // deltaImpulse = _mm_or_ps(_mm_and_ps(resultLowerLess, lowMinApplied), _mm_andnot_ps(resultLowerLess, deltaImpulse));
        delta_impulse = if sum < self.lower_limit {
            low_min_applied
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
        deltas.0 += linear_component_a * impulse_magnitude;
        // bodyA.internalGetDeltaAngularVelocity().mVec128 = _mm_add_ps(bodyA.internalGetDeltaAngularVelocity().mVec128, _mm_mul_ps(c.m_angularComponentA.mVec128, impulseMagnitude));
        deltas.1 += self.angular_component * impulse_magnitude;
        // bodyB.internalGetDeltaLinearVelocity().mVec128 = _mm_add_ps(bodyB.internalGetDeltaLinearVelocity().mVec128, _mm_mul_ps(linearComponentB, impulseMagnitude));
        // bodyB.internalGetDeltaAngularVelocity().mVec128 = _mm_add_ps(bodyB.internalGetDeltaAngularVelocity().mVec128, _mm_mul_ps(c.m_angularComponentB.mVec128, impulseMagnitude));
        // return deltaImpulse.m_floats[0] / c.m_jacDiagABInv;
        delta_impulse / self.jac_diag_ab_inv
    }
}

#[derive(Clone, Debug)]
pub(crate) struct Constraints {
    contact: Vec<Constraint>,
    friction: Vec<Constraint>,
    inv_mass: f32,
}

impl Constraints {
    pub const MAX_CONTACTS: usize = 4;

    const NUM_ITERATIONS: usize = 10;
    const COEFF_FRICTION: f32 = 0.35;
    const RESTITUTION: f32 = 0.6;
    const RESTITUTION_VELOCITY_THRESHOLD: f32 = 0.2;
    const RELAXATION: f32 = 1.;

    #[inline]
    #[must_use]
    pub fn new(inv_mass: f32) -> Self {
        Self {
            contact: Vec::with_capacity(Self::MAX_CONTACTS),
            friction: Vec::with_capacity(Self::MAX_CONTACTS),
            inv_mass,
        }
    }

    pub fn create_constraint(&mut self, ball: &Ball, contact: &Contact, external_force_impulse: Vec3A) {
        let (contact_constraint, friction_constraint) = self.process_contacts(ball, contact, external_force_impulse);
        self.contact.push(contact_constraint);
        self.friction.push(friction_constraint);
    }

    // void btSequentialImpulseConstraintSolver::setupContactConstraint(btSolverConstraint& solverConstraint,
    //     int solverBodyIdA, int solverBodyIdB,
    //     btManifoldPoint& cp, const btContactSolverInfo& infoGlobal,
    //     btScalar& relaxation,
    //     const btVector3& rel_pos1, const btVector3& rel_pos2)
    fn setup_contact_contraint(
        &self,
        ball: &Ball,
        normal_world_on_b: Vec3A,
        rel_pos: Vec3A,
        external_force_impulse: Vec3A,
    ) -> Constraint {
        // btSolverBody* bodyA = &m_tmpSolverBodyPool[solverBodyIdA];
        // btSolverBody* bodyB = &m_tmpSolverBodyPool[solverBodyIdB];

        // this is the ball
        // btRigidBody* rb0 = bodyA->m_originalBody;
        // rb1 is a null pointer
        // btRigidBody* rb1 = bodyB->m_originalBody;

        // relaxation = infoGlobal.m_sor;
        // btScalar invTimeStep = btScalar(1) / infoGlobal.m_timeStep;
        // let inv_time_step = 1. / dt;

        // btScalar cfm = infoGlobal.m_globalCfm; // 0
        // let mut cfm = 0.;
        // btScalar erp = infoGlobal.m_erp2; // 0.8

        // cfm *= invTimeStep;
        // cfm *= inv_time_step;

        // btVector3 torqueAxis0 = rel_pos1.cross(cp.m_normalWorldOnB);
        // dbg!(rel_pos1 / 50., normal_world_on_b);
        let torque_axis = rel_pos.cross(normal_world_on_b);
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
        let vec = angular_component.cross(rel_pos);
        let denom = self.inv_mass + normal_world_on_b.dot(vec);
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
        let contact_normal = normal_world_on_b;
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
        let abs_vel = ball.get_velocity_in_local_point(rel_pos);
        // vel2 = rb1 ? rb1->getVelocityInLocalPoint(rel_pos2) : btVector3(0, 0, 0);
        // let vel2 = Vec3A::ZERO;

        // dbg!(vel1 / 50.);

        // btVector3 vel = vel1 - vel2;
        // let vel = vel1 - vel2;
        // btScalar rel_vel = cp.m_normalWorldOnB.dot(vel);
        let rel_vel = normal_world_on_b.dot(abs_vel);

        // solverConstraint.m_friction = cp.m_combinedFriction;

        // infoGlobal.m_restitutionVelocityThreshold = 0.2
        // restitution = restitutionCurve(rel_vel, cp.m_combinedRestitution, infoGlobal.m_restitutionVelocityThreshold);
        // if (restitution <= btScalar(0.))
        // if restitution <= 0. {
        //     // restitution = 0.f;
        //     restitution = 0.;
        // }
        let restitution = Game::restitution_curve(rel_vel, Self::RESTITUTION, Self::RESTITUTION_VELOCITY_THRESHOLD).max(0.);

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
        let rel_vel =
            contact_normal.dot(ball.velocity + external_force_impulse) + rel_pos_cross_normal.dot(ball.angular_velocity);

        // btScalar vel2Dotn = solverConstraint.m_contactNormal2.dot(bodyB->m_linearVelocity + externalForceImpulseB) + solverConstraint.m_relpos2CrossNormal.dot(bodyB->m_angularVelocity + externalTorqueImpulseB);
        // let vel_2_dot_n = contact_normal_2.dot(Vec3A::ZERO + external_force_impulse_b) + rel_pos_2_cross_normal.dot(Vec3A::ZERO + external_torque_impulse_b);
        // let vel_2_dot_n = 0.;
        // btScalar rel_vel = vel1Dotn + vel2Dotn; // vel2Dotn is always 0
        // let rel_vel = vel_dot_n + vel_2_dot_n;

        // btScalar positionalError = 0.f;
        // let mut positional_error = 0.;
        // btScalar velocityError = restitution - rel_vel;
        let velocity_error = restitution - rel_vel;
        // dbg!(velocity_error / 50., restitution / 50.);

        // if (penetration > 0)
        // if penetration > 0. {
        //     // positionalError = 0;
        //     positional_error = 0.;
        // } else {
        //     // positionalError = -penetration * erp * invTimeStep;
        //     positional_error = -penetration * Self::ERP2 * inv_time_step;
        // }

        // btScalar penetrationImpulse = positionalError * solverConstraint.m_jacDiagABInv; // m_jacDiagABInv is currently about 27.94
        // this should always be 0, in theory
        // let penetration_impulse = positional_error * jac_diag_ab_inv;
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
            contact_normal,
            rel_pos_cross_normal,
            angular_component,
            rhs: velocity_impulse,
            lower_limit: 0.,
            upper_limit: 1e10,
            jac_diag_ab_inv,
            applied_impulse,
        }
    }

    // called by btSequentialImpulseConstraintSolver::addFrictionConstraint
    // void btSequentialImpulseConstraintSolver::setupFrictionConstraint(btSolverConstraint& solverConstraint, const btVector3& normalAxis, int solverBodyIdA, int solverBodyIdB, btManifoldPoint& cp, const btVector3& rel_pos1, const btVector3& rel_pos2, btCollisionObject* colObj0, btCollisionObject* colObj1, btScalar relaxation, const btContactSolverInfo& infoGlobal, btScalar desiredVelocity, btScalar cfmSlip)
    fn setup_friction_constraint(
        &self,
        ball: &Ball,
        normal_axis: Vec3A,
        rel_pos: Vec3A,
        external_force_impulse: Vec3A,
    ) -> Constraint {
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
        // dbg!(jac_diag_ab_inv);

        // btScalar rel_vel;
        // btScalar vel1Dotn = solverConstraint.m_contactNormal1.dot(body0 ? solverBodyA.m_linearVelocity + solverBodyA.m_externalForceImpulse : btVector3(0, 0, 0)) + solverConstraint.m_relpos1CrossNormal.dot(body0 ? solverBodyA.m_angularVelocity : btVector3(0, 0, 0));
        let rel_vel =
            contact_normal.dot(ball.velocity + external_force_impulse) + rel_pos_cross_normal.dot(ball.angular_velocity);
        // btScalar vel2Dotn = solverConstraint.m_contactNormal2.dot(bodyA ? solverBodyB.m_linearVelocity + solverBodyB.m_externalForceImpulse : btVector3(0, 0, 0)) + solverConstraint.m_relpos2CrossNormal.dot(bodyA ? solverBodyB.m_angularVelocity : btVector3(0, 0, 0));
        // let vel_2_dot_n = 0.;

        // rel_vel = vel1Dotn + vel2Dotn; // vel2Dotn is always 0
        // let rel_vel = vel_1_dot_n + vel_2_dot_n;

        // let desired_velocity = 0.;
        // btScalar velocityError = desiredVelocity - rel_vel;
        let velocity_error = -rel_vel;
        // btScalar velocityImpulse = velocityError * solverConstraint.m_jacDiagABInv;
        let velocity_impulse = velocity_error * jac_diag_ab_inv;
        // dbg!(velocity_impulse / 50.);

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
            lower_limit: -Self::COEFF_FRICTION,
            upper_limit: Self::COEFF_FRICTION,
            jac_diag_ab_inv,
            applied_impulse,
        }
    }

    fn process_contacts(&self, ball: &Ball, contact: &Contact, external_force_impulse: Vec3A) -> (Constraint, Constraint) {
        // normalOnBInWorld = contact_point.direction
        // pointInWorld = contact_point.start
        // let point_in_world = contact.ray.start;
        // cp.getDistance() = depth = contact_point.depth
        // btVector3 pointA = pointInWorld + normalOnBInWorld * depth;
        // newPt.m_positionWorldOnA = pointA;
        // newPt.m_positionWorldOnB = pointInWorld;
        // newPt.m_normalWorldOnB = normalOnBInWorld;
        // let normal_world_on_b = contact.ray.direction;
        // dbg!(contact.start / 50.);
        // dbg!(contact.direction);
        // dbg!(contact.depth / 50.);
        // btPlaneSpace1(newPt.m_normalWorldOnB, newPt.m_lateralFrictionDir1, newPt.m_lateralFrictionDir2);
        // let (lateral_friction_dir_1, lateral_friction_dir_2) = Self::plane_space_1(normal_world_on_b);
        // dbg!(lateral_friction_dir_1, lateral_friction_dir_2);

        // adds contact point to manifold
        // this is where RocketSim does stuff like callbacks and extra forces for extra gamemodes

        // btVector3 rel_pos1;
        // btVector3 rel_pos2;
        // btScalar relaxation;

        // int frictionIndex = m_tmpSolverContactConstraintPool.size();
        // btSolverConstraint& solverConstraint = m_tmpSolverContactConstraintPool.expandNonInitializing();
        // solverConstraint.m_solverBodyIdA = solverBodyIdA;
        // solverConstraint.m_solverBodyIdB = solverBodyIdB;

        // solverConstraint.m_originalContactPoint = &cp;

        // const btVector3& pos1 = cp.getPositionWorldOnA();
        // let pos1 = contact.position;
        // const btVector3& pos2 = cp.getPositionWorldOnB();
        // let pos2 = point_in_world;

        // colObj0 is the ball
        // colObj1 is all default values
        // rel_pos1 = pos1 - colObj0->getWorldTransform().getOrigin();
        // let rel_pos1 = pos1 - self.location;
        let rel_pos = contact.local_position;
        // rel_pos2 = pos2 - colObj1->getWorldTransform().getOrigin();
        // let rel_pos2 = pos2 - Vec3A::ZERO;
        // dbg!(rel_pos1 / 50., rel_pos2 / 50.);

        // btVector3 vel1;
        // btVector3 vel2;

        // solverBodyA->getVelocityInLocalPointNoDelta(rel_pos1, vel1);
        let vel = ball.get_velocity_in_local_point_no_delta(rel_pos, external_force_impulse);

        // solverBodyB has m_originalBody as false, so vel2 is always 0, 0, 0
        // solverBodyB->getVelocityInLocalPointNoDelta(rel_pos2, vel2);
        // let vel2 = Vec3A::ZERO;

        // not sure at which point this changes or when it's set to this instead :/
        let cp_normal_world_on_b = contact.triangle_normal;
        // btVector3 vel = vel1 - vel2;
        // let vel = vel1 - vel2;
        // btScalar rel_vel = cp.m_normalWorldOnB.dot(vel);
        let rel_vel = cp_normal_world_on_b.dot(vel);
        // dbg!(rel_vel / 50.);

        // setupContactConstraint(solverConstraint, solverBodyIdA, solverBodyIdB, cp, infoGlobal, relaxation, rel_pos1, rel_pos2);
        let contact_constraint = self.setup_contact_contraint(ball, cp_normal_world_on_b, rel_pos, external_force_impulse);

        // cp.m_lateralFrictionDir1 = vel - cp.m_normalWorldOnB * rel_vel;
        let mut cp_lateral_friction_dir_1 = vel - cp_normal_world_on_b * rel_vel;
        // btScalar lat_rel_vel = cp.m_lateralFrictionDir1.length2();
        let lat_rel_vel = cp_lateral_friction_dir_1.length_squared();

        // if (lat_rel_vel > SIMD_EPSILON)
        if lat_rel_vel > f32::EPSILON {
            // cp.m_lateralFrictionDir1 *= 1.f / btSqrt(lat_rel_vel);
            cp_lateral_friction_dir_1 /= lat_rel_vel.sqrt();
            // addFrictionConstraint(cp.m_lateralFrictionDir1, solverBodyIdA, solverBodyIdB, frictionIndex, cp, rel_pos1, rel_pos2, colObj0, colObj1, relaxation, infoGlobal);
        } else {
            // btPlaneSpace1(cp.m_normalWorldOnB, cp.m_lateralFrictionDir1, cp.m_lateralFrictionDir2);
            cp_lateral_friction_dir_1 = Game::plane_space_1(cp_normal_world_on_b);
            // addFrictionConstraint(cp.m_lateralFrictionDir1, solverBodyIdA, solverBodyIdB, frictionIndex, cp, rel_pos1, rel_pos2, colObj0, colObj1, relaxation, infoGlobal);
        }

        let friction_constraint =
            self.setup_friction_constraint(ball, cp_lateral_friction_dir_1, rel_pos, external_force_impulse);

        // this method sets the current m_appliedImpulse back to 0
        // setFrictionConstraintImpulse(solverConstraint, solverBodyIdA, solverBodyIdB, cp, infoGlobal);

        (contact_constraint, friction_constraint)
    }

    fn solve_single_iteration(&mut self, deltas: &mut (Vec3A, Vec3A)) -> f32 {
        let mut least_squares_residual = 0f32;

        for contact_constraint in &mut self.contact {
            let residual = contact_constraint.resolve_single_constraint_row_lower_limit(deltas, self.inv_mass);
            least_squares_residual = least_squares_residual.max(residual * residual);
        }

        for (contact_constraint, friction_constraint) in self.contact.iter().zip(&mut self.friction) {
            let total_impulse = contact_constraint.applied_impulse;

            if total_impulse > 0. {
                friction_constraint.lower_limit = -Self::COEFF_FRICTION * total_impulse;
                friction_constraint.upper_limit = Self::COEFF_FRICTION * total_impulse;

                let residual = friction_constraint.resolve_single_constraint_row_generic(deltas, self.inv_mass);
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
