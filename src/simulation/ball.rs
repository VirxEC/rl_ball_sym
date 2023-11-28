//! Tools for simulation a Rocket League ball.

use super::{
    game::{Constraint, Constraints, Game},
    geometry::{Ray, Sphere},
};
use crate::linear_algebra::math::Vec3AExt;
use colored::Colorize;
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
    const FRICTION: f32 = 0.35;
    const RESTITUTION: f32 = 0.6;
    const DRAG: f32 = 0.03;
    const RESTITUTION_VELOCITY_THRESHOLD: f32 = 0.2;
    const RELAXATION: f32 = 1.;

    const V_MAX: f32 = 6000.;
    const W_MAX: f32 = 6.;

    const M: f32 = 30.;
    const INV_M: f32 = 1. / Self::M;

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

    #[inline]
    fn get_velocity_in_local_point(&self, rel_pos: Vec3A) -> Vec3A {
        self.velocity + self.angular_velocity.cross(rel_pos)
    }

    #[inline]
    fn get_velocity_in_local_point_no_delta(&self, rel_pos: Vec3A, external_force_impulse: Vec3A) -> Vec3A {
        self.velocity + external_force_impulse + self.angular_velocity.cross(rel_pos)
    }

    // void btSequentialImpulseConstraintSolver::setupContactConstraint(btSolverConstraint& solverConstraint,
    //     int solverBodyIdA, int solverBodyIdB,
    //     btManifoldPoint& cp, const btContactSolverInfo& infoGlobal,
    //     btScalar& relaxation,
    //     const btVector3& rel_pos1, const btVector3& rel_pos2)
    fn setup_contact_contraint(
        &self,
        normal_world_on_b: Vec3A,
        rel_pos1: Vec3A,
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
        let torque_axis_0 = rel_pos1.cross(normal_world_on_b);
        // dbg!(torque_axis_0);
        // getAngularFactor will always return 1, 1, 1
        // solverConstraint.m_angularComponentA = rb0 ? rb0->getInvInertiaTensorWorld() * torqueAxis0 * rb0->getAngularFactor() : btVector3(0, 0, 0);
        // dbg!(self.inv_inertia);
        let angular_component_a = self.inv_inertia * torque_axis_0;
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
        let vec = angular_component_a.cross(rel_pos1);
        let denom_0 = Self::INV_M + normal_world_on_b.dot(vec);
        // let denom_1 = 0.;

        // dbg!(vec);
        // dbg!(denom_0);
        // btScalar denom = relaxation / (denom0 + denom1 + cfm);
        let denom = Self::RELAXATION / denom_0;
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
        // let contact_normal_2 = Vec3A::ZERO;
        // let rel_pos2_cross_normal = Vec3A::ZERO;

        // btScalar restitution = 0.f;
        // btScalar penetration = cp.getDistance() + infoGlobal.m_linearSlop; // m_linearSlop is 0
        // let penetration = contact.depth;

        // btVector3 vel1, vel2;

        // vel1 = rb0 ? rb0->getVelocityInLocalPoint(rel_pos1) : btVector3(0, 0, 0);
        let abs_vel = self.get_velocity_in_local_point(rel_pos1);
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
        let external_force_impulse_a = external_force_impulse;
        // btVector3 externalTorqueImpulseA = bodyA->m_originalBody ? bodyA->m_externalTorqueImpulse : btVector3(0, 0, 0);
        // let external_torque_impulse_a = Vec3A::ZERO;
        // btVector3 externalForceImpulseB = bodyB->m_originalBody ? bodyB->m_externalForceImpulse : btVector3(0, 0, 0);
        // let external_force_impulse_b = Vec3A::ZERO;
        // btVector3 externalTorqueImpulseB = bodyB->m_originalBody ? bodyB->m_externalTorqueImpulse : btVector3(0, 0, 0);
        // let external_torque_impulse_b = Vec3A::ZERO;

        // btScalar vel1Dotn = solverConstraint.m_contactNormal1.dot(bodyA->m_linearVelocity + externalForceImpulseA) + solverConstraint.m_relpos1CrossNormal.dot(bodyA->m_angularVelocity + externalTorqueImpulseA);
        let vel_1_dot_n = contact_normal_1.dot(self.velocity + external_force_impulse_a)
            + rel_pos1_cross_normal.dot(self.angular_velocity);

        // btScalar vel2Dotn = solverConstraint.m_contactNormal2.dot(bodyB->m_linearVelocity + externalForceImpulseB) + solverConstraint.m_relpos2CrossNormal.dot(bodyB->m_angularVelocity + externalTorqueImpulseB);
        // let vel_2_dot_n = contact_normal_2.dot(Vec3A::ZERO + external_force_impulse_b) + rel_pos_2_cross_normal.dot(Vec3A::ZERO + external_torque_impulse_b);
        let vel_2_dot_n = 0.;
        // btScalar rel_vel = vel1Dotn + vel2Dotn; // vel2Dotn is always 0
        let rel_vel = vel_1_dot_n + vel_2_dot_n;

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
            contact_normal_1,
            rel_pos1_cross_normal,
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
        // let penetration_impulse = 0.;

        // solverConstraint.m_rhs = penetrationImpulse + velocityImpulse;
        // solverConstraint.m_rhsPenetration = 0.f;
        // solverConstraint.m_cfm = cfmSlip; // 0
        // solverConstraint.m_lowerLimit = -solverConstraint.m_friction;
        // solverConstraint.m_upperLimit = solverConstraint.m_friction;
        Constraint {
            contact_normal_1,
            rel_pos1_cross_normal,
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
        external_force_impulse: Vec3A,
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
        // let pos2 = point_in_world;

        // colObj0 is the ball
        // colObj1 is all default values
        // rel_pos1 = pos1 - colObj0->getWorldTransform().getOrigin();
        let rel_pos1 = pos1 - self.location;
        // rel_pos2 = pos2 - colObj1->getWorldTransform().getOrigin();
        // let rel_pos2 = pos2 - Vec3A::ZERO;
        // dbg!(rel_pos1 / 50., rel_pos2 / 50.);

        // btVector3 vel1;
        // btVector3 vel2;

        // solverBodyA->getVelocityInLocalPointNoDelta(rel_pos1, vel1);
        let vel = self.get_velocity_in_local_point_no_delta(rel_pos1, external_force_impulse);

        // solverBodyB has m_originalBody as false, so vel2 is always 0, 0, 0
        // solverBodyB->getVelocityInLocalPointNoDelta(rel_pos2, vel2);
        // let vel2 = Vec3A::ZERO;

        // not sure at which point this changes or when it's set to this instead :/
        let cp_normal_world_on_b = triangle_normal;
        // btVector3 vel = vel1 - vel2;
        // let vel = vel1 - vel2;
        // btScalar rel_vel = cp.m_normalWorldOnB.dot(vel);
        let rel_vel = cp_normal_world_on_b.dot(vel);
        // dbg!(rel_vel / 50.);

        // setupContactConstraint(solverConstraint, solverBodyIdA, solverBodyIdB, cp, infoGlobal, relaxation, rel_pos1, rel_pos2);
        let contact_constraint = self.setup_contact_contraint(cp_normal_world_on_b, rel_pos1, external_force_impulse);

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
            self.setup_friction_constraint(cp_lateral_friction_dir_1, rel_pos1, external_force_impulse);

        // this method sets the current m_appliedImpulse back to 0
        // setFrictionConstraintImpulse(solverConstraint, solverBodyIdA, solverBodyIdB, cp, infoGlobal);

        (contact_constraint, friction_constraint)
    }

    /// Simulate the ball for one game tick
    ///
    /// `dt` - The delta time (game tick length)
    pub fn step(&mut self, game: &Game, dt: f32) {
        self.time += dt;

        if self.velocity.length_squared() != 0. || self.angular_velocity.length_squared() != 0. {
            println!("{}; {}; {}; {}", self.time, self.location, self.velocity, self.angular_velocity);
            self.velocity *= (1. - Self::DRAG).powf(dt);
            let external_force_impulse = game.gravity * dt;

            let contacts = game.triangle_collisions.collide(self.hitbox());
            if !contacts.is_empty() {
                println!("{}{}{}", "[CONTACTS: ".bright_green(), contacts.len(), "]".bright_green());
                // dbg!(self.location / 50.);
                // dbg!(self.velocity / 50.);
                // dbg!(self.angular_velocity);

                let mut constraints = Constraints::with_capacity(contacts.len(), Self::INV_M, Self::FRICTION);

                for contact in contacts {
                    constraints.push(self.process_contacts(contact, external_force_impulse));
                }

                let (delta_linear_velocity, delta_angular_velocity) = constraints.solve();

                self.velocity += delta_linear_velocity;
                self.angular_velocity += delta_angular_velocity;
            }

            self.velocity += external_force_impulse;
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
