use crate::linear_algebra::vector::Vec3;

enum BallShape {
    Sphere,
    Puck,
    Cube,
}

pub struct Ball {
    pub location: Vec3,
    pub velocity: Vec3,
    pub radius: f32,
    pub collision_radius: f32
}

impl Default for Ball {
    fn default() -> Self {
        Self {
            location: Vec3::default(),
            velocity: Vec3::default(),
            radius: 0.,
            collision_radius: 0.
        }
    }
}

impl Ball {
    const RESTITUTION: f32 = 0.6;
    const DRAG: f32 = -0.0305;
    const MU: f32 = 2.;
    
    const V_MAX: f32 = 4000.;
    const W_MAX: f32 = 6.;

    const M: f32 = 30.;
    const SOCCAR_RADIUS: f32 = 91.25;
    const HOOPS_RADIUS: f32 = 91.25;
    const DROPSHOT_RADIUS: f32 = 100.45;
    const SOCCAR_COLLISION_RADIUS: f32 = 93.15;
    const HOOPS_COLLISION_RADIUS: f32 = 93.15;
    const DROPSHOT_COLLISION_RADIUS: f32 = 103.6;

    pub fn initialize_soccar() -> Self {
        let mut ball = Ball::default();
        ball.radius = Ball::SOCCAR_RADIUS;
        ball.collision_radius = Ball::SOCCAR_COLLISION_RADIUS;

        ball
    }
}
