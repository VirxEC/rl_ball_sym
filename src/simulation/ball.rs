use crate::simulation::game::Game;
use crate::simulation::geometry::Sphere;
use vvec3::Vec3;

#[derive(Clone, Copy, Debug)]
pub struct Ball {
    pub time: f32,
    pub location: Vec3,
    pub velocity: Vec3,
    pub angular_velocity: Vec3,
    pub radius: f32,
    pub collision_radius: f32,
    pub moi: f32,
}

impl Default for Ball {
    fn default() -> Self {
        Self {
            time: 0.,
            location: Vec3::default(),
            velocity: Vec3::default(),
            angular_velocity: Vec3::default(),
            radius: 0.,
            collision_radius: 0.,
            moi: 0.,
        }
    }
}

#[derive(Clone)]
pub struct BallPrediction {
    pub num_slices: usize,
    pub slices: Vec<Ball>,
}

impl Default for BallPrediction {
    fn default() -> Self {
        Self {
            num_slices: 0,
            slices: Vec::new(),
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

    const INV_M: f32 = 1. / 30.;
    const RESTITUTION_M: f32 = -(1. + Ball::RESTITUTION) * Ball::M;

    const SIMULATION_DT: f32 = 1. / 120.;
    const STANDARD_NUM_SLICES: usize = 720;

    pub fn initialize_soccar() -> Self {
        let mut ball = Ball::default();
        ball.radius = Ball::SOCCAR_RADIUS;
        ball.collision_radius = Ball::SOCCAR_COLLISION_RADIUS;
        ball.initialize();

        ball
    }

    pub fn initialize_hoops() -> Self {
        let mut ball = Ball::default();
        ball.radius = Ball::HOOPS_RADIUS;
        ball.collision_radius = Ball::HOOPS_COLLISION_RADIUS;
        ball.initialize();

        ball
    }

    pub fn initialize_dropshot() -> Self {
        let mut ball = Ball::default();
        ball.radius = Ball::DROPSHOT_RADIUS;
        ball.collision_radius = Ball::DROPSHOT_COLLISION_RADIUS;
        ball.initialize();

        ball
    }

    pub fn initialize(&mut self) {
        self.location.z = 1.1 * self.collision_radius;
        self.calculate_moi();
    }

    pub fn calculate_moi(&mut self) {
        self.moi = 0.4 * Ball::M * self.radius * self.radius;
    }

    pub fn update(&mut self, time: f32, location: Vec3, velocity: Vec3, angular_velocity: Vec3) {
        self.time = time;
        self.location = location;
        self.velocity = velocity;
        self.angular_velocity = angular_velocity;
    }

    fn hitbox(&self) -> Sphere {
        Sphere {
            center: self.location.clone(),
            radius: self.collision_radius.clone(),
        }
    }

    pub fn step(game: &mut Game, dt: f32) {
        match game.collision_mesh.collide(&game.ball.hitbox()) {
            Some(contact) => {
                let p = contact.start;
                let n = contact.direction;

                let loc = p - game.ball.location;

                let m_reduced = 1. / (Ball::INV_M + loc.dot(&loc) / game.ball.moi);

                let v_perp = n * game.ball.velocity.dot(&n).min(0.);
                let v_para = game.ball.velocity - v_perp - loc.cross(&game.ball.angular_velocity);

                let ratio = v_perp.magnitude() / v_para.magnitude().max(0.0001);

                let j_perp = v_perp * Ball::RESTITUTION_M;
                let j_para = -(Ball::MU * ratio).min(1.) * m_reduced * v_para;

                let j = j_perp + j_para;

                game.ball.angular_velocity += loc.cross(&j) / game.ball.moi;
                game.ball.velocity += (j / Ball::M) + game.ball.velocity * (Ball::DRAG * dt);
                game.ball.location += game.ball.velocity * dt;

                let penetration = game.ball.collision_radius - (game.ball.location - p).dot(&n);
                if penetration > 0. {
                    game.ball.location += n * (1.001 * penetration);
                }
            }
            None => {
                game.ball.velocity += (game.ball.velocity * Ball::DRAG + game.gravity) * dt;
                game.ball.location += game.ball.velocity * dt;
            }
        }

        game.ball.angular_velocity *= (Ball::W_MAX / game.ball.angular_velocity.magnitude()).min(1.);
        game.ball.velocity *= (Ball::V_MAX / game.ball.velocity.magnitude()).min(1.);
        game.ball.time += dt;
    }

    pub fn get_ball_prediction_struct_for_time(game: &mut Game, time: &f32) -> BallPrediction {
        Ball::get_ball_prediction_struct_for_slices(game, (time / Ball::SIMULATION_DT).round() as usize)
    }

    pub fn get_ball_prediction_struct(game: &mut Game) -> BallPrediction {
        Ball::get_ball_prediction_struct_for_slices(game, Ball::STANDARD_NUM_SLICES)
    }

    pub fn get_ball_prediction_struct_for_slices(game: &mut Game, num_slices: usize) -> BallPrediction {
        let mut slices = Vec::with_capacity(num_slices);

        for _ in 0..Ball::STANDARD_NUM_SLICES {
            Ball::step(game, Ball::SIMULATION_DT);
            slices.push(game.ball.clone());
        }

        BallPrediction {
            slices,
            num_slices,
        }
    }
}
