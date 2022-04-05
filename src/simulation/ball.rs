use crate::simulation::game::Game;
use crate::simulation::geometry::Sphere;
use glam::Vec3A;

/// Represents the game's ball
#[derive(Clone, Copy, Debug, Default)]
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
    pub radius: f32,
    /// Size of the ball for collisions
    pub collision_radius: f32,
    /// Momemnt of inertia of the ball
    pub moi: f32,
}

/// Collection of Balls representing future predictions based on field geometry
pub type BallPrediction = Vec<Ball>;

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

    /// Sets the default values for a soccar ball
    pub fn initialize_soccar() -> Self {
        let mut ball = Ball {
            radius: Ball::SOCCAR_RADIUS,
            collision_radius: Ball::SOCCAR_COLLISION_RADIUS,
            ..Default::default()
        };

        ball.initialize();

        ball
    }

    /// Sets the default values for a soccer ball
    pub fn initialize_soccer() -> Self {
        Self::initialize_soccar()
    }

    /// Sets the default values for a hoops ball
    pub fn initialize_hoops() -> Self {
        let mut ball = Ball {
            radius: Ball::HOOPS_RADIUS,
            collision_radius: Ball::HOOPS_COLLISION_RADIUS,
            ..Default::default()
        };

        ball.initialize();

        ball
    }

    /// Sets the default values for a dropshot ball
    pub fn initialize_dropshot() -> Self {
        let mut ball = Ball {
            radius: Ball::DROPSHOT_RADIUS,
            collision_radius: Ball::DROPSHOT_COLLISION_RADIUS,
            ..Default::default()
        };

        ball.initialize();

        ball
    }

    /// Sets a value location and calculates the moi
    pub fn initialize(&mut self) {
        self.location.z = 1.1 * self.collision_radius;
        self.calculate_moi();
    }

    /// Calculates the moment of inertia of the ball
    pub fn calculate_moi(&mut self) {
        self.moi = 0.4 * Ball::M * self.radius * self.radius;
    }

    /// Updates the ball with everything that changes from game tick to game tick
    pub fn update(&mut self, time: f32, location: Vec3A, velocity: Vec3A, angular_velocity: Vec3A) {
        self.time = time;
        self.location = location;
        self.velocity = velocity;
        self.angular_velocity = angular_velocity;
    }

    /// Converts the ball into a sphere
    pub const fn hitbox(&self) -> Sphere {
        Sphere {
            center: self.location,
            radius: self.collision_radius,
        }
    }

    /// Simulate the ball for one game tick
    /// 
    /// dt is the delta time (game tick length)
    pub fn step(game: &mut Game, dt: f32) {
        match game.collision_mesh.collide(&game.ball.hitbox()) {
            Some(contact) => {
                let p = contact.start;
                let n = contact.direction;

                let loc = p - game.ball.location;

                let m_reduced = 1. / (Ball::INV_M + loc.length_squared() / game.ball.moi);

                let v_perp = n * game.ball.velocity.dot(n).min(0.);
                let v_para = game.ball.velocity - v_perp - loc.cross(game.ball.angular_velocity);

                let ratio = v_perp.length() / v_para.length().max(0.0001);

                let j_perp = v_perp * Ball::RESTITUTION_M;
                let j_para = -(Ball::MU * ratio).min(1.) * m_reduced * v_para;

                let j = j_perp + j_para;

                game.ball.angular_velocity += loc.cross(j) / game.ball.moi;
                game.ball.velocity += (j / Ball::M) + game.ball.velocity * (Ball::DRAG * dt);
                game.ball.location += game.ball.velocity * dt;

                let penetration = game.ball.collision_radius - (game.ball.location - p).dot(n);
                if penetration > 0. {
                    game.ball.location += n * (1.001 * penetration);
                }
            }
            None => {
                game.ball.velocity += (game.ball.velocity * Ball::DRAG + game.gravity) * dt;
                game.ball.location += game.ball.velocity * dt;
            }
        }

        game.ball.angular_velocity *= (Ball::W_MAX * game.ball.angular_velocity.length_recip()).min(1.);
        game.ball.velocity *= (Ball::V_MAX * game.ball.velocity.length_recip()).min(1.);
        game.ball.time += dt;
    }

    /// Simulate the ball for a given amount of time
    pub fn get_ball_prediction_struct_for_time(game: &mut Game, time: &f32) -> BallPrediction {
        Ball::get_ball_prediction_struct_for_slices(game, (time / Ball::SIMULATION_DT).round() as usize)
    }

    /// Simulate the ball for the stand amount of time
    pub fn get_ball_prediction_struct(game: &mut Game) -> BallPrediction {
        Ball::get_ball_prediction_struct_for_slices(game, Ball::STANDARD_NUM_SLICES)
    }

    /// Simulate the ball for a given amount of ticks
    pub fn get_ball_prediction_struct_for_slices(game: &mut Game, num_slices: usize) -> BallPrediction {
        let mut slices = Vec::with_capacity(num_slices);

        for _ in 0..num_slices {
            Ball::step(game, Ball::SIMULATION_DT);
            slices.push(game.ball);
        }

        slices
    }
}

#[cfg(test)]
mod test {
    use crate::load_soccar;

    use super::*;

    #[test]
    fn check_standard_num_slices() {
        let mut game = load_soccar();

        let prediction = Ball::get_ball_prediction_struct(&mut game);

        assert_eq!(prediction.len(), Ball::STANDARD_NUM_SLICES);
    }

    #[test]
    fn check_custom_num_slices() {
        const REQUESTED_SLICES: usize = 200;

        let mut game = load_soccar();

        let prediction = Ball::get_ball_prediction_struct_for_slices(&mut game, REQUESTED_SLICES);

        assert_eq!(prediction.len(), REQUESTED_SLICES);
    }

    #[test]
    fn check_num_slices_for_time() {
        const REQUESTED_TIME: f32 = 8.0;

        let mut game = load_soccar();

        let prediction = Ball::get_ball_prediction_struct_for_time(&mut game, &REQUESTED_TIME);

        let predicted_slices = (REQUESTED_TIME / Ball::SIMULATION_DT).round() as usize;

        assert_eq!(prediction.len(), predicted_slices);
    }
}
