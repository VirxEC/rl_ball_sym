//! Tools for simulation a Rocket League ball.

use super::geometry::Ray;
use crate::{
    linear_algebra::math::Vec3AExt,
    simulation::{game::Game, geometry::Sphere},
};
use glam::Vec3A;
use std::f32::consts::FRAC_1_SQRT_2;

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

#[derive(Debug)]
struct Constraint {
    pub contact_normal_1: Vec3A,
    pub contact_normal_2: Vec3A,
    pub rel_pos1_cross_normal: Vec3A,
    pub rel_pos2_cross_normal: Vec3A,
    pub angular_component_a: Vec3A,
    pub rhs: f32,
    pub lower_limit: f32,
    pub upper_limit: f32,
    pub jac_diag_ab_inv: f32,
    pub applied_impulse: f32,
}

impl Ball {
    const FRICTION: f32 = 0.35;
    const RESTITUTION: f32 = 0.6;
    const DRAG: f32 = 0.03;
    const ERP2: f32 = 0.8;
    const RESTITUTION_VELOCITY_THRESHOLD: f32 = 0.2;
    const RELAXATION: f32 = 1.;
    const NUM_ITERATIONS: usize = 10;

    const V_MAX: f32 = 6000.;
    const W_MAX: f32 = 6.;

    const M: f32 = 30.;
    const INV_M: f32 = 1. / Self::M;

    const VELOCITY_THRESHOLD: f32 = 10.;
    const CONTACT_BREAKING_THRESHOLD: f32 = Sphere::get_contact_breaking_threshold();

    const STANDARD_RADIUS: f32 = 91.25;
    const HOOPS_RADIUS: f32 = 96.38307;
    const DROPSHOT_RADIUS: f32 = 100.2565;

    const SIMULATION_DT: f32 = 1. / 120.;
    const STANDARD_NUM_SLICES: usize = 720;

    const DEFAULT_INERTIA: f32 = 1. / (0.4 * Self::M);

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

    //  void btPlaneSpace1(const T& n, T& p, T& q)
    // {
    //     if (btFabs(n[2]) > SIMDSQRT12)
    //     {
    //         // choose p in y-z plane
    //         btScalar a = n[1] * n[1] + n[2] * n[2];
    //         btScalar k = btRecipSqrt(a);
    //         p[0] = 0;
    //         p[1] = -n[2] * k;
    //         p[2] = n[1] * k;
    //         // set q = n x p
    //         q[0] = a * k;
    //         q[1] = -n[0] * p[2];
    //         q[2] = n[0] * p[1];
    //     }
    //     else
    //     {
    //         // choose p in x-y plane
    //         btScalar a = n[0] * n[0] + n[1] * n[1];
    //         btScalar k = btRecipSqrt(a);
    //         p[0] = -n[1] * k;
    //         p[1] = n[0] * k;
    //         p[2] = 0;
    //         // set q = n x p
    //         q[0] = -n[2] * p[1];
    //         q[1] = n[2] * p[0];
    //         q[2] = a * k;
    //    }
    // }
    #[allow(clippy::many_single_char_names)]
    fn plane_space_1(n: Vec3A) -> (Vec3A, Vec3A) {
        if n.z.abs() > FRAC_1_SQRT_2 {
            // choose p in y-z plane
            // btScalar a = n[1] * n[1] + n[2] * n[2];
            let a = n.y * n.y + n.z * n.z;
            // btScalar k = btRecipSqrt(a);
            let k = 1. / a.sqrt();
            // p[0] = 0;
            // p[1] = -n[2] * k;
            // p[2] = n[1] * k;
            let p = Vec3A::new(0., -n.z * k, n.y * k);
            // set q = n x p
            // q[0] = a * k;
            // q[1] = -n[0] * p[2];
            // q[2] = n[0] * p[1];
            let q = Vec3A::new(a * k, -n.x * p.z, n.x * p.y);
            (p, q)
        } else {
            // choose p in x-y plane
            // btScalar a = n[0] * n[0] + n[1] * n[1];
            let a = n.x * n.x + n.y * n.y;
            // btScalar k = btRecipSqrt(a);
            let k = 1. / a.sqrt();
            // p[0] = -n[1] * k;
            // p[1] = n[0] * k;
            // p[2] = 0;
            let p = Vec3A::new(-n.y * k, n.x * k, 0.);
            // set q = n x p
            // q[0] = -n[2] * p[1];
            // q[1] = n[2] * p[0];
            // q[2] = a * k;
            let q = Vec3A::new(-n.z * p.y, n.z * p.x, a * k);
            (p, q)
        }
    }

    // btVector3 getVelocityInLocalPoint(const btVector3& rel_pos) const
    fn get_velocity_in_local_point(&self, rel_pos: Vec3A) -> Vec3A {
        // we also calculate lin/ang velocity for kinematic objects
        // return m_linearVelocity + m_angularVelocity.cross(rel_pos);
        self.velocity + self.angular_velocity.cross(rel_pos)
    }

    //  void getVelocityInLocalPointNoDelta(const btVector3& rel_pos, btVector3& velocity) const
    fn get_velocity_in_local_point_no_delta(&self, rel_pos: Vec3A, external_force_impulse: Vec3A) -> Vec3A {
        // velocity = m_linearVelocity + m_externalForceImpulse + (m_angularVelocity + m_externalTorqueImpulse).cross(rel_pos);
        self.velocity + external_force_impulse + self.angular_velocity.cross(rel_pos)
    }

    // btScalar btSequentialImpulseConstraintSolver::restitutionCurve(btScalar rel_vel, btScalar restitution, btScalar velocityThreshold)
    fn restitution_curve(rel_vel: f32, restitution: f32, velocity_threshold: f32) -> f32 {
        // if (btFabs(rel_vel) < velocityThreshold)
        if rel_vel.abs() < velocity_threshold {
            // return 0.;
            0.
        } else {
            // btScalar rest = restitution * -rel_vel;
            // return rest;
            restitution * -rel_vel
        }
    }

    //  void internalApplyImpulse(const btVector3& linearComponent, const btVector3& angularComponent, const btScalar impulseMagnitude)
    // {
    // 	if (m_originalBody)
    // 	{
    // 		m_deltaLinearVelocity += linearComponent * impulseMagnitude * m_linearFactor;
    // 		m_deltaAngularVelocity += angularComponent * (impulseMagnitude * m_angularFactor);
    // 	}
    // }

    // void btSequentialImpulseConstraintSolver::setupContactConstraint(btSolverConstraint& solverConstraint,
    //     int solverBodyIdA, int solverBodyIdB,
    //     btManifoldPoint& cp, const btContactSolverInfo& infoGlobal,
    //     btScalar& relaxation,
    //     const btVector3& rel_pos1, const btVector3& rel_pos2)
    fn setup_contact_contraint(
        &self,
        (contact, normal_world_on_b): (Ray, Vec3A),
        rel_pos1: Vec3A,
        rel_pos2: Vec3A,
        external_force_impulse: Vec3A,
        dt: f32,
    ) -> Constraint {
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
        let mut cfm = 0.;
        // btScalar erp = infoGlobal.m_erp2; // 0.8

        // cfm *= invTimeStep;
        cfm *= inv_time_step;

        // btVector3 torqueAxis0 = rel_pos1.cross(cp.m_normalWorldOnB);
        // dbg!(rel_pos1 / 50., normal_world_on_b);
        let torque_axis_0 = rel_pos1.cross(normal_world_on_b);
        // dbg!(torque_axis_0);
        // getAngularFactor will always return 1, 1, 1
        // solverConstraint.m_angularComponentA = rb0 ? rb0->getInvInertiaTensorWorld() * torqueAxis0 * rb0->getAngularFactor() : btVector3(0, 0, 0);
        // dbg!(self.inv_inertia);
        let angular_component_a = self.inv_inertia * torque_axis_0;
        // dbg!(angular_component_a);
        // btVector3 torqueAxis1 = rel_pos2.cross(cp.m_normalWorldOnB);
        let torque_axis_1 = rel_pos2.cross(normal_world_on_b);
        // solverConstraint.m_angularComponentB = rb1 ? rb1->getInvInertiaTensorWorld() * -torqueAxis1 * rb1->getAngularFactor() : btVector3(0, 0, 0);
        let angular_component_b = Vec3A::ZERO;

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
        let vec = angular_component_a.cross(rel_pos1);
        let denom_0 = Self::INV_M + normal_world_on_b.dot(vec);
        let denom_1 = 0.;

        // dbg!(vec);
        // dbg!(denom_0);
        // btScalar denom = relaxation / (denom0 + denom1 + cfm);
        let denom = Self::RELAXATION / (denom_0 + denom_1 + cfm);
        // solverConstraint.m_jacDiagABInv = denom;
        let jac_diag_ab_inv = denom;
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
        let contact_normal_1 = normal_world_on_b;
        let rel_pos1_cross_normal = torque_axis_0;
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
        let contact_normal_2 = Vec3A::ZERO;
        let rel_pos2_cross_normal = Vec3A::ZERO;

        // btScalar restitution = 0.f;
        // btScalar penetration = cp.getDistance() + infoGlobal.m_linearSlop; // m_linearSlop is 0
        let penetration = contact.depth;

        // btVector3 vel1, vel2;

        // vel1 = rb0 ? rb0->getVelocityInLocalPoint(rel_pos1) : btVector3(0, 0, 0);
        let vel1 = self.get_velocity_in_local_point(rel_pos1);
        // vel2 = rb1 ? rb1->getVelocityInLocalPoint(rel_pos2) : btVector3(0, 0, 0);
        let vel2 = Vec3A::ZERO;

        // dbg!(vel1 / 50.);

        // btVector3 vel = vel1 - vel2;
        let vel = vel1 - vel2;
        // btScalar rel_vel = cp.m_normalWorldOnB.dot(vel);
        let rel_vel = normal_world_on_b.dot(vel);

        // solverConstraint.m_friction = cp.m_combinedFriction;

        // infoGlobal.m_restitutionVelocityThreshold = 0.2
        // restitution = restitutionCurve(rel_vel, cp.m_combinedRestitution, infoGlobal.m_restitutionVelocityThreshold);
        let mut restitution = Self::restitution_curve(rel_vel, Self::RESTITUTION, Self::RESTITUTION_VELOCITY_THRESHOLD);
        // if (restitution <= btScalar(0.))
        if restitution <= 0. {
            // restitution = 0.f;
            restitution = 0.;
        }

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
        let external_force_impulse_a = external_force_impulse;
        // btVector3 externalTorqueImpulseA = bodyA->m_originalBody ? bodyA->m_externalTorqueImpulse : btVector3(0, 0, 0);
        let external_torque_impulse_a = Vec3A::ZERO;
        // btVector3 externalForceImpulseB = bodyB->m_originalBody ? bodyB->m_externalForceImpulse : btVector3(0, 0, 0);
        // let external_force_impulse_b = Vec3A::ZERO;
        // btVector3 externalTorqueImpulseB = bodyB->m_originalBody ? bodyB->m_externalTorqueImpulse : btVector3(0, 0, 0);
        // let external_torque_impulse_b = Vec3A::ZERO;

        // btScalar vel1Dotn = solverConstraint.m_contactNormal1.dot(bodyA->m_linearVelocity + externalForceImpulseA) + solverConstraint.m_relpos1CrossNormal.dot(bodyA->m_angularVelocity + externalTorqueImpulseA);
        let vel_1_dot_n = contact_normal_1.dot(self.velocity + external_force_impulse_a)
            + rel_pos1_cross_normal.dot(self.angular_velocity + external_torque_impulse_a);

        // btScalar vel2Dotn = solverConstraint.m_contactNormal2.dot(bodyB->m_linearVelocity + externalForceImpulseB) + solverConstraint.m_relpos2CrossNormal.dot(bodyB->m_angularVelocity + externalTorqueImpulseB);
        // let vel_2_dot_n = contact_normal_2.dot(Vec3A::ZERO + external_force_impulse_b) + rel_pos_2_cross_normal.dot(Vec3A::ZERO + external_torque_impulse_b);
        let vel_2_dot_n = 0.;
        // btScalar rel_vel = vel1Dotn + vel2Dotn; // vel2Dotn is always 0
        let rel_vel = vel_1_dot_n + vel_2_dot_n;

        // btScalar positionalError = 0.f;
        let mut positional_error = 0.;
        // btScalar velocityError = restitution - rel_vel;
        let velocity_error = restitution - rel_vel;
        // dbg!(velocity_error / 50., restitution / 50.);

        // if (penetration > 0)
        if penetration > 0. {
            // positionalError = 0;
            positional_error = 0.;
        } else {
            // positionalError = -penetration * erp * invTimeStep;
            positional_error = -penetration * Self::ERP2 * inv_time_step;
        }

        // btScalar penetrationImpulse = positionalError * solverConstraint.m_jacDiagABInv; // m_jacDiagABInv is currently about 27.94
        // this should always be 0, in theory
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
            contact_normal_1,
            contact_normal_2,
            rel_pos1_cross_normal,
            rel_pos2_cross_normal,
            angular_component_a,
            rhs: velocity_impulse,
            lower_limit: 0.,
            upper_limit: 1e10,
            jac_diag_ab_inv,
            applied_impulse,
        }
    }

    // called by btSequentialImpulseConstraintSolver::addFrictionConstraint
    // void btSequentialImpulseConstraintSolver::setupFrictionConstraint(btSolverConstraint& solverConstraint, const btVector3& normalAxis, int solverBodyIdA, int solverBodyIdB, btManifoldPoint& cp, const btVector3& rel_pos1, const btVector3& rel_pos2, btCollisionObject* colObj0, btCollisionObject* colObj1, btScalar relaxation, const btContactSolverInfo& infoGlobal, btScalar desiredVelocity, btScalar cfmSlip)
    fn setup_friction_constraint(&self, normal_axis: Vec3A, rel_pos1: Vec3A, external_force_impulse: Vec3A) -> Constraint {
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

        let contact_normal_1 = normal_axis;
        let rel_pos1_cross_normal = rel_pos1.cross(contact_normal_1);
        let angular_component_a = self.inv_inertia * rel_pos1_cross_normal;

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

        let contact_normal_2 = Vec3A::ZERO;
        let rel_pos2_cross_normal = Vec3A::ZERO;
        let angular_component_b = Vec3A::ZERO;

        // btVector3 vec;
        // btScalar denom0 = 0.f;
        // btScalar denom1 = 0.f;
        // if (body0)
        // {
        //     vec = (solverConstraint.m_angularComponentA).cross(rel_pos1);
        //     denom0 = body0->getInvMass() + normalAxis.dot(vec);
        // }
        let vec = angular_component_a.cross(rel_pos1);
        let denom_0 = Self::INV_M + normal_axis.dot(vec);

        // if (bodyA)
        // {
        //     vec = (-solverConstraint.m_angularComponentB).cross(rel_pos2);
        //     denom1 = bodyA->getInvMass() + normalAxis.dot(vec);
        // }
        let denom_1 = 0.;

        // btScalar denom = relaxation / (denom0 + denom1);
        let denom = Self::RELAXATION / (denom_0 + denom_1);
        // solverConstraint.m_jacDiagABInv = denom;
        let jac_diag_ab_inv = denom;
        // dbg!(jac_diag_ab_inv);

        // btScalar rel_vel;
        // btScalar vel1Dotn = solverConstraint.m_contactNormal1.dot(body0 ? solverBodyA.m_linearVelocity + solverBodyA.m_externalForceImpulse : btVector3(0, 0, 0)) + solverConstraint.m_relpos1CrossNormal.dot(body0 ? solverBodyA.m_angularVelocity : btVector3(0, 0, 0));
        let vel_1_dot_n =
            contact_normal_1.dot(self.velocity + external_force_impulse) + rel_pos1_cross_normal.dot(self.angular_velocity);
        // btScalar vel2Dotn = solverConstraint.m_contactNormal2.dot(bodyA ? solverBodyB.m_linearVelocity + solverBodyB.m_externalForceImpulse : btVector3(0, 0, 0)) + solverConstraint.m_relpos2CrossNormal.dot(bodyA ? solverBodyB.m_angularVelocity : btVector3(0, 0, 0));
        let vel_2_dot_n = 0.;

        // rel_vel = vel1Dotn + vel2Dotn; // vel2Dotn is always 0
        let rel_vel = vel_1_dot_n + vel_2_dot_n;

        let desired_velocity = 0.;
        // btScalar velocityError = desiredVelocity - rel_vel;
        let velocity_error = desired_velocity - rel_vel;
        // btScalar velocityImpulse = velocityError * solverConstraint.m_jacDiagABInv;
        let velocity_impulse = velocity_error * jac_diag_ab_inv;
        // dbg!(velocity_impulse / 50.);

        // btScalar penetrationImpulse = btScalar(0);
        let penetration_impulse = 0.;

        // solverConstraint.m_rhs = penetrationImpulse + velocityImpulse;
        // solverConstraint.m_rhsPenetration = 0.f;
        // solverConstraint.m_cfm = cfmSlip; // 0
        // solverConstraint.m_lowerLimit = -solverConstraint.m_friction;
        // solverConstraint.m_upperLimit = solverConstraint.m_friction;
        Constraint {
            contact_normal_1,
            contact_normal_2,
            rel_pos1_cross_normal,
            rel_pos2_cross_normal,
            angular_component_a,
            rhs: velocity_impulse,
            lower_limit: -Self::FRICTION,
            upper_limit: Self::FRICTION,
            jac_diag_ab_inv,
            applied_impulse,
        }
    }

    fn process_contacts(
        &self,
        (contact, triangle_normal): (Ray, Vec3A),
        gravity: Vec3A,
        dt: f32,
    ) -> (Constraint, Constraint) {
        // normalOnBInWorld = contact_point.direction
        // pointInWorld = contact_point.start
        let point_in_world = contact.start;
        // cp.getDistance() = depth = contact_point.depth
        // btVector3 pointA = pointInWorld + normalOnBInWorld * depth;
        // newPt.m_positionWorldOnA = pointA;
        // newPt.m_positionWorldOnB = pointInWorld;
        // newPt.m_normalWorldOnB = normalOnBInWorld;
        let normal_world_on_b = contact.direction;
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
        let pos1 = point_in_world + normal_world_on_b * contact.depth;
        // const btVector3& pos2 = cp.getPositionWorldOnB();
        let pos2 = point_in_world;

        // colObj0 is the ball
        // colObj1 is all default values
        // rel_pos1 = pos1 - colObj0->getWorldTransform().getOrigin();
        let rel_pos1 = pos1 - self.location;
        // rel_pos2 = pos2 - colObj1->getWorldTransform().getOrigin();
        let rel_pos2 = pos2 - Vec3A::ZERO;
        // dbg!(rel_pos1 / 50., rel_pos2 / 50.);

        // btVector3 vel1;
        // btVector3 vel2;
        let external_force_impulse = gravity * dt;

        // solverBodyA->getVelocityInLocalPointNoDelta(rel_pos1, vel1);
        let vel1 = self.get_velocity_in_local_point_no_delta(rel_pos1, external_force_impulse);

        // solverBodyB has m_originalBody as false, so vel2 is always 0, 0, 0
        // solverBodyB->getVelocityInLocalPointNoDelta(rel_pos2, vel2);
        let vel2 = Vec3A::ZERO;

        // not sure at which point this changes or when it's set to this instead :/
        let cp_normal_world_on_b = triangle_normal;
        // btVector3 vel = vel1 - vel2;
        let vel = vel1 - vel2;
        // btScalar rel_vel = cp.m_normalWorldOnB.dot(vel);
        let rel_vel = cp_normal_world_on_b.dot(vel);
        // dbg!(rel_vel / 50.);

        // setupContactConstraint(solverConstraint, solverBodyIdA, solverBodyIdB, cp, infoGlobal, relaxation, rel_pos1, rel_pos2);
        let contact_constraint = self.setup_contact_contraint(
            (contact, cp_normal_world_on_b),
            rel_pos1,
            rel_pos2,
            external_force_impulse,
            dt,
        );

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
            cp_lateral_friction_dir_1 = Self::plane_space_1(cp_normal_world_on_b).0;
            // addFrictionConstraint(cp.m_lateralFrictionDir1, solverBodyIdA, solverBodyIdB, frictionIndex, cp, rel_pos1, rel_pos2, colObj0, colObj1, relaxation, infoGlobal);
        }

        let friction_constraint =
            self.setup_friction_constraint(cp_lateral_friction_dir_1, rel_pos1, external_force_impulse);

        // this method sets the current m_appliedImpulse back to 0
        // setFrictionConstraintImpulse(solverConstraint, solverBodyIdA, solverBodyIdB, cp, infoGlobal);

        (contact_constraint, friction_constraint)
    }

    // called by resolveSingleConstraintRowLowerLimit
    // static btScalar gResolveSingleConstraintRowLowerLimit_sse2(btSolverBody& bodyA, btSolverBody& bodyB, const btSolverConstraint& c)
    fn resolve_single_constraint_row_lower_limit(constraint: &mut Constraint, deltas: &mut (Vec3A, Vec3A)) -> f32 {
        // m_cfm is 0
        // __m128 cpAppliedImp = _mm_set1_ps(c.m_appliedImpulse);
        // __m128 lowerLimit1 = _mm_set1_ps(c.m_lowerLimit);
        // __m128 upperLimit1 = _mm_set1_ps(c.m_upperLimit);
        // btSimdScalar deltaImpulse = _mm_sub_ps(_mm_set1_ps(c.m_rhs), _mm_mul_ps(_mm_set1_ps(c.m_appliedImpulse), _mm_set1_ps(c.m_cfm)));
        let mut delta_impulse = constraint.rhs - 0.;
        // __m128 deltaVel1Dotn = _mm_add_ps(btSimdDot3(c.m_contactNormal1.mVec128, bodyA.internalGetDeltaLinearVelocity().mVec128), btSimdDot3(c.m_relpos1CrossNormal.mVec128, bodyA.internalGetDeltaAngularVelocity().mVec128));
        let delta_vel_1_dot_n = constraint.contact_normal_1.dot(deltas.0) + constraint.rel_pos1_cross_normal.dot(deltas.1);
        // __m128 deltaVel2Dotn = _mm_add_ps(btSimdDot3(c.m_contactNormal2.mVec128, bodyB.internalGetDeltaLinearVelocity().mVec128), btSimdDot3(c.m_relpos2CrossNormal.mVec128, bodyB.internalGetDeltaAngularVelocity().mVec128));
        let delta_vel_2_dot_n = constraint.contact_normal_2.dot(deltas.0) + constraint.rel_pos2_cross_normal.dot(deltas.1);
        // deltaImpulse = _mm_sub_ps(deltaImpulse, _mm_mul_ps(deltaVel1Dotn, _mm_set1_ps(c.m_jacDiagABInv)));
        delta_impulse -= delta_vel_1_dot_n * constraint.jac_diag_ab_inv;
        // deltaImpulse = _mm_sub_ps(deltaImpulse, _mm_mul_ps(deltaVel2Dotn, _mm_set1_ps(c.m_jacDiagABInv)));
        delta_impulse -= delta_vel_2_dot_n * constraint.jac_diag_ab_inv;
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
        constraint.applied_impulse = if sum < constraint.lower_limit {
            constraint.lower_limit
        } else {
            sum
        };
        // __m128 linearComponentA = _mm_mul_ps(c.m_contactNormal1.mVec128, bodyA.internalGetInvMass().mVec128);
        let linear_component_a = constraint.contact_normal_1 * Self::INV_M;
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
    fn resolve_single_constraint_row_generic(constraint: &mut Constraint, deltas: &mut (Vec3A, Vec3A)) -> f32 {
        // cfm is still 0
        // __m128 cpAppliedImp = _mm_set1_ps(c.m_appliedImpulse);
        // __m128 lowerLimit1 = _mm_set1_ps(c.m_lowerLimit);
        // __m128 upperLimit1 = _mm_set1_ps(c.m_upperLimit);
        // btSimdScalar deltaImpulse = _mm_sub_ps(_mm_set1_ps(c.m_rhs), _mm_mul_ps(_mm_set1_ps(c.m_appliedImpulse), _mm_set1_ps(c.m_cfm)));
        let mut delta_impulse = constraint.rhs - 0.;
        // __m128 deltaVel1Dotn = _mm_add_ps(btSimdDot3(c.m_contactNormal1.mVec128, bodyA.internalGetDeltaLinearVelocity().mVec128), btSimdDot3(c.m_relpos1CrossNormal.mVec128, bodyA.internalGetDeltaAngularVelocity().mVec128));
        let delta_vel_1_dot_n = constraint.contact_normal_1.dot(deltas.0) + constraint.rel_pos1_cross_normal.dot(deltas.1);
        // __m128 deltaVel2Dotn = _mm_add_ps(btSimdDot3(c.m_contactNormal2.mVec128, bodyB.internalGetDeltaLinearVelocity().mVec128), btSimdDot3(c.m_relpos2CrossNormal.mVec128, bodyB.internalGetDeltaAngularVelocity().mVec128));
        let delta_vel_2_dot_n = constraint.contact_normal_2.dot(deltas.0) + constraint.rel_pos2_cross_normal.dot(deltas.1);
        // deltaImpulse = _mm_sub_ps(deltaImpulse, _mm_mul_ps(deltaVel1Dotn, _mm_set1_ps(c.m_jacDiagABInv)));
        delta_impulse -= delta_vel_1_dot_n * constraint.jac_diag_ab_inv;
        // deltaImpulse = _mm_sub_ps(deltaImpulse, _mm_mul_ps(deltaVel2Dotn, _mm_set1_ps(c.m_jacDiagABInv)));
        delta_impulse -= delta_vel_2_dot_n * constraint.jac_diag_ab_inv;
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
        constraint.applied_impulse = if sum < constraint.lower_limit {
            constraint.lower_limit
        } else {
            sum
        };
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
        let linear_component_a = constraint.contact_normal_1 * Self::INV_M;
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

    // btScalar btSequentialImpulseConstraintSolver::solveSingleIteration(int iteration, btCollisionObject** /*bodies */, int /*numBodies*/, btPersistentManifold** /*manifoldPtr*/, int /*numManifolds*/, btTypedConstraint** constraints, int numConstraints, const btContactSolverInfo& infoGlobal)
    fn solve_single_iteration(
        contact_constraints: &mut [Constraint],
        friction_constraints: &mut [Constraint],
        iteration: usize,
        deltas: &mut (Vec3A, Vec3A),
    ) -> f32 {
        // btScalar leastSquaresResidual = 0.f;
        let mut least_squares_residual = 0f32;

        // numNonContactPool is always 0
        // int numNonContactPool = m_tmpSolverNonContactConstraintPool.size();
        // int numConstraintPool = m_tmpSolverContactConstraintPool.size();
        // let num_constraints_pool = contact_constraints.len();
        // int numFrictionPool = m_tmpSolverContactFrictionConstraintPool.size();
        // let num_friction_pool = friction_constraints.len();

        // if (iteration < infoGlobal.m_numIterations)
        if iteration < Self::NUM_ITERATIONS {
            // solve the friction constraints after all contact constraints, don't interleave them
            // int numPoolConstraints = m_tmpSolverContactConstraintPool.size();
            // int j;

            // for (j = 0; j < numPoolConstraints; j++)
            // {
            // 	const btSolverConstraint& solveManifold = m_tmpSolverContactConstraintPool[m_orderTmpConstraintPool[j]];
            // 	btScalar residual = resolveSingleConstraintRowLowerLimit(m_tmpSolverBodyPool[solveManifold.m_solverBodyIdA], m_tmpSolverBodyPool[solveManifold.m_solverBodyIdB], solveManifold);
            // 	leastSquaresResidual = btMax(leastSquaresResidual, residual * residual);
            // }
            // dbg!(iteration);

            for contact_constraint in contact_constraints.iter_mut() {
                let residual = Self::resolve_single_constraint_row_lower_limit(contact_constraint, deltas);
                // dbg!(residual / 50.);
                least_squares_residual = least_squares_residual.max(residual * residual);
            }

            // solve all friction constraints

            // numFrictionPoolConstraints is the number of contact points
            // int numFrictionPoolConstraints = m_tmpSolverContactFrictionConstraintPool.size();
            // for (j = 0; j < numFrictionPoolConstraints; j++)
            // {
            // 	btSolverConstraint& solveManifold = m_tmpSolverContactFrictionConstraintPool[m_orderFrictionConstraintPool[j]];
            // 	btScalar totalImpulse = m_tmpSolverContactConstraintPool[solveManifold.m_frictionIndex].m_appliedImpulse;

            //  this only get called on the first round?
            // 	if (totalImpulse > btScalar(0))
            // 	{
            // 		solveManifold.m_lowerLimit = -(solveManifold.m_friction * totalImpulse);
            // 		solveManifold.m_upperLimit = solveManifold.m_friction * totalImpulse;

            // 		btScalar residual = resolveSingleConstraintRowGeneric(m_tmpSolverBodyPool[solveManifold.m_solverBodyIdA], m_tmpSolverBodyPool[solveManifold.m_solverBodyIdB], solveManifold);
            // 		leastSquaresResidual = btMax(leastSquaresResidual, residual * residual);
            // 	}
            // }
            for (friction_constraint, contact_constraint) in friction_constraints.iter_mut().zip(contact_constraints.iter())
            {
                let total_impulse = contact_constraint.applied_impulse;

                if total_impulse > 0. {
                    friction_constraint.lower_limit = -Self::FRICTION * total_impulse;
                    friction_constraint.upper_limit = Self::FRICTION * total_impulse;

                    let residual = Self::resolve_single_constraint_row_generic(friction_constraint, deltas);
                    // dbg!(residual / 50.);
                    least_squares_residual = least_squares_residual.max(residual * residual);
                }
            }
        }

        // return leastSquaresResidual;
        least_squares_residual
    }

    // void btSequentialImpulseConstraintSolver::writeBackContacts(int iBegin, int iEnd, const btContactSolverInfo& infoGlobal)
    // {
    //     for (int j = iBegin; j < iEnd; j++)
    //     {
    //         const btSolverConstraint& solveManifold = m_tmpSolverContactConstraintPool[j];
    //         btManifoldPoint* pt = (btManifoldPoint*)solveManifold.m_originalContactPoint;
    //         btAssert(pt);
    //         pt->m_appliedImpulse = solveManifold.m_appliedImpulse;
    //         pt->m_appliedImpulseLateral1 = m_tmpSolverContactFrictionConstraintPool[solveManifold.m_frictionIndex].m_appliedImpulse;
    //     }
    // }

    // void writebackVelocityAndTransform(btScalar timeStep, btScalar splitImpulseTurnErp)
    // {
    // 	(void)timeStep;
    // 	if (m_originalBody)
    // 	{
    // 		m_linearVelocity += m_deltaLinearVelocity;
    // 		m_angularVelocity += m_deltaAngularVelocity;
    // 	}
    // }

    // void btSequentialImpulseConstraintSolver::writeBackBodies(int iBegin, int iEnd, const btContactSolverInfo& infoGlobal)
    // {
    //     // this loop only runs once - it's for the ball
    //     for (int i = iBegin; i < iEnd; i++)
    //     {
    //         btRigidBody* body = m_tmpSolverBodyPool[i].m_originalBody;
    //         if (body)
    //         {
    //             // true
    //             if (infoGlobal.m_splitImpulse)
    //                 m_tmpSolverBodyPool[i].writebackVelocityAndTransform(infoGlobal.m_timeStep, infoGlobal.m_splitImpulseTurnErp);
    //             else
    //                 m_tmpSolverBodyPool[i].writebackVelocity();

    //             // m_linearVelocity = lin_vel;
    //             m_tmpSolverBodyPool[i].m_originalBody->setLinearVelocity(
    //                 m_tmpSolverBodyPool[i].m_linearVelocity +
    //                 m_tmpSolverBodyPool[i].m_externalForceImpulse);

    //             // m_angularVelocity = ang_vel;
    //             m_tmpSolverBodyPool[i].m_originalBody->setAngularVelocity(
    //                 m_tmpSolverBodyPool[i].m_angularVelocity +
    //                 m_tmpSolverBodyPool[i].m_externalTorqueImpulse);

    //             // true
    //             if (infoGlobal.m_splitImpulse)
    //                 // m_worldTransform = worldTrans;
    //                 m_tmpSolverBodyPool[i].m_originalBody->setWorldTransform(m_tmpSolverBodyPool[i].m_worldTransform);

    //             m_tmpSolverBodyPool[i].m_originalBody->setCompanionId(-1);
    //         }
    //     }
    // }

    #[must_use]
    fn solve_constraints(constraints: Vec<(Constraint, Constraint)>) -> (Vec3A, Vec3A) {
        // stuff is now setup, moves on

        // numConstraints = 0

        // btSequentialImpulseConstraintSolver::solveGroupCacheFriendlySplitImpulseIterations
        // this is only every actually needed if m_rhsPenetration is not 0
        // it should ALWAYS be 0, anything more and there's error which should happen between ball <-> triangles
        // gResolveSplitPenetrationImpulse_sse2 runs if m_rhsPenetration is not 0
        // solveGroupCacheFriendlySplitImpulseIterations(bodies, numBodies, manifoldPtr, numManifolds, constraints, numConstraints, infoGlobal);

        // infoGlobal.m_numIterations is 10
        // int maxIterations = m_maxOverrideNumSolverIterations > infoGlobal.m_numIterations ? m_maxOverrideNumSolverIterations : infoGlobal.m_numIterations;

        let (mut contact_constraints, mut friction_constraints): (Vec<_>, Vec<_>) = constraints.into_iter().unzip();

        let mut deltas = (Vec3A::ZERO, Vec3A::ZERO);

        // for (int iteration = 0; iteration < maxIterations; iteration++)
        for iteration in 0..Self::NUM_ITERATIONS {
            //  this gets smaller and smaller with each iteration
            // m_leastSquaresResidual = solveSingleIteration(iteration, bodies, numBodies, manifoldPtr, numManifolds, constraints, numConstraints, infoGlobal);
            let least_squares_residual =
                Self::solve_single_iteration(&mut contact_constraints, &mut friction_constraints, iteration, &mut deltas);

            // m_leastSquaresResidualThreshold is 0
            // if (m_leastSquaresResidual <= infoGlobal.m_leastSquaresResidualThreshold || (iteration >= (maxIterations - 1)))
            // SPEED NOTE: make 0. into f32::EPSILON
            if least_squares_residual <= 0. || iteration >= Self::NUM_ITERATIONS - 1 {
                // m_analyticsData.m_numSolverCalls++;
                // m_analyticsData.m_numIterationsUsed = iteration+1;
                // m_analyticsData.m_islandId = -2;
                // if (numBodies>0)
                //     m_analyticsData.m_islandId = bodies[0]->getCompanionId();
                // m_analyticsData.m_numBodies = numBodies;
                // m_analyticsData.m_numContactManifolds = numManifolds;
                // m_analyticsData.m_remainingLeastSquaresResidual = m_leastSquaresResidual;
                break;
            }
        }

        deltas
    }

    /// Simulate the ball for one game tick
    ///
    /// `dt` - The delta time (game tick length)
    pub fn step(&mut self, game: &Game, dt: f32) {
        self.time += dt;

        if self.velocity.length_squared() != 0. || self.angular_velocity.length_squared() != 0. {
            self.velocity *= (1. - Self::DRAG).powf(dt);

            let contacts = game.collision_mesh.collide(self.hitbox());
            if contacts.is_empty() {
                self.velocity += game.gravity * dt;
            } else {
                // dbg!(self.time);
                // dbg!(self.location / 50.);
                // dbg!(self.velocity / 50.);
                // dbg!(self.angular_velocity);

                let contraints = contacts
                    .into_iter()
                    .map(|(contact, triangle_normal)| self.process_contacts((contact, triangle_normal), game.gravity, dt))
                    .collect::<Vec<_>>();

                let (delta_linear_velocity, delta_angular_velocity) = Self::solve_constraints(contraints);
                // dbg!(delta_linear_velocity / 50., delta_angular_velocity);

                // m_tmpSolverContactConstraintPool is the number of contact points
                // writeBackContacts(0, m_tmpSolverContactConstraintPool.size(), infoGlobal);

                // m_tmpSolverBodyPool is 2 but only the first one (the ball) matters
                // writeBackBodies(0, m_tmpSolverBodyPool.size(), infoGlobal);

                self.velocity += delta_linear_velocity + game.gravity * dt;
                self.angular_velocity += delta_angular_velocity;
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
