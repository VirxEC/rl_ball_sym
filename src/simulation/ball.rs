use crate::linear_algebra::mat::{MatrixExt, EYE};
use crate::linear_algebra::math::{antisym, dot, lerp};
use crate::simulation::game::Game;
use crate::simulation::geometry::Sphere;
use glam::Vec3A;

use super::geometry::{Car, CarLike};

fn scale(dv: f32) -> f32 {
    let values = [[0., 0.65], [500., 0.65], [2300., 0.55], [4600., 0.30], [0., 0.], [0., 0.]];

    let input = dv.clamp(0., 4600.);

    for i in 0..5 {
        if values[i][0] <= input && input < values[i + 1][0] {
            let u = (input - values[i][0]) / (values[i + 1][0] - values[i][0]);
            return lerp(values[i][1], values[i + 1][1], u);
        }
    }

    -1.
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Ball {
    pub time: f32,
    pub location: Vec3A,
    pub velocity: Vec3A,
    pub angular_velocity: Vec3A,
    pub radius: f32,
    pub collision_radius: f32,
    pub moi: f32,
}

#[derive(Clone, Debug, Default)]
pub struct BallPrediction {
    pub num_slices: usize,
    pub slices: Vec<Ball>,
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
        let mut ball = Ball {
            radius: Ball::SOCCAR_RADIUS,
            collision_radius: Ball::SOCCAR_COLLISION_RADIUS,
            ..Default::default()
        };

        ball.initialize();

        ball
    }

    pub fn initialize_hoops() -> Self {
        let mut ball = Ball {
            radius: Ball::HOOPS_RADIUS,
            collision_radius: Ball::HOOPS_COLLISION_RADIUS,
            ..Default::default()
        };

        ball.initialize();

        ball
    }

    pub fn initialize_dropshot() -> Self {
        let mut ball = Ball {
            radius: Ball::DROPSHOT_RADIUS,
            collision_radius: Ball::DROPSHOT_COLLISION_RADIUS,
            ..Default::default()
        };

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

    pub fn update(&mut self, time: f32, location: Vec3A, velocity: Vec3A, angular_velocity: Vec3A) {
        self.time = time;
        self.location = location;
        self.velocity = velocity;
        self.angular_velocity = angular_velocity;
    }

    const fn hitbox(&self) -> Sphere {
        Sphere {
            center: self.location,
            radius: self.collision_radius,
        }
    }

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

    pub fn get_ball_prediction_struct_for_time(game: &mut Game, time: &f32) -> BallPrediction {
        Ball::get_ball_prediction_struct_for_slices(game, (time / Ball::SIMULATION_DT).round() as usize)
    }

    pub fn get_ball_prediction_struct(game: &mut Game) -> BallPrediction {
        Ball::get_ball_prediction_struct_for_slices(game, Ball::STANDARD_NUM_SLICES)
    }

    pub fn get_ball_prediction_struct_for_slices(game: &mut Game, num_slices: usize) -> BallPrediction {
        let mut slices = Vec::with_capacity(num_slices);

        for _ in 0..num_slices {
            Ball::step(game, Ball::SIMULATION_DT);
            slices.push(game.ball);
        }

        BallPrediction {
            num_slices: slices.len(),
            slices,
        }
    }

    pub fn step_car<T: CarLike>(game: &mut Game, car: T, dt: f32) {
        let p = car.get_hitbox().closest_point_on_obb(game.ball.location);

        if (p - game.ball.location).length() < game.ball.collision_radius {
            let cx = car.get_location();
            let cv = car.get_velocity();
            let cw = car.get_angular_velocity();
            let co = car.get_orientation();
            let ci = car.get_inv_i();

            let n1 = (p - game.ball.location).normalize_or_zero();

            let l_b = antisym(p - game.ball.location);
            let l_c = antisym(p - cx);

            let inv_i_c = co.dot(ci.dot(co.transpose()));

            let m = (((1. / Ball::M) + (1. / Car::M)) * EYE - (l_b.dot(l_b).denom(game.ball.moi)) - l_c.dot(inv_i_c.dot(l_c))).inverse();

            let delta_v = (cv - dot(l_c, cw)) - (game.ball.velocity - dot(l_b, game.ball.angular_velocity));

            // compute the impulse that is consistent with an inelastic collision
            let mut j1 = dot(m, delta_v);

            let j1_perp = j1.dot(n1).min(-1.) * n1;
            let j1_para = j1 - j1_perp;

            let ratio = j1_perp.length() / j1_para.length().max(0.001);

            // scale the parallel component of J1 such that the
            // Coulomb friction model is satisfied
            j1 = j1_perp + 1_f32.min(Ball::MU * ratio) * j1_para;

            let f = co.x_axis;
            let mut n2 = game.ball.location - cx;
            n2[2] *= 0.35;
            n2 = (n2 - 0.35 * n2.dot(f) * f).normalize_or_zero();

            let dv = (game.ball.velocity - cv).length().min(4600.);
            let j2 = Ball::M * dv * scale(dv) * n2;

            game.ball.angular_velocity += dot(l_b, j1) / game.ball.moi;
            game.ball.velocity += (j1 + j2) / Ball::M;
        }

        Ball::step(game, dt);
    }

    pub fn get_ball_car_prediction_struct_for_time<T: Copy + CarLike>(game: &mut Game, car: T, time: &f32) -> BallPrediction {
        Ball::get_ball_car_prediction_struct_for_slices(game, car, (time / Ball::SIMULATION_DT).round() as usize)
    }

    pub fn get_ball_car_prediction_struct<T: Copy + CarLike>(game: &mut Game, car: T) -> BallPrediction {
        Ball::get_ball_car_prediction_struct_for_slices(game, car, Ball::STANDARD_NUM_SLICES)
    }

    pub fn get_ball_car_prediction_struct_for_slices<T: Copy + CarLike>(game: &mut Game, car: T, num_slices: usize) -> BallPrediction {
        let mut slices = Vec::with_capacity(num_slices);

        for _ in 0..num_slices {
            Ball::step_car(game, car, Ball::SIMULATION_DT);
            slices.push(game.ball);
        }

        BallPrediction {
            num_slices: slices.len(),
            slices,
        }
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

        assert_eq!(prediction.num_slices, Ball::STANDARD_NUM_SLICES);
        assert_eq!(prediction.slices.len(), Ball::STANDARD_NUM_SLICES);
    }

    #[test]
    fn check_custom_num_slices() {
        const REQUESTED_SLICES: usize = 200;

        let mut game = load_soccar();

        let prediction = Ball::get_ball_prediction_struct_for_slices(&mut game, REQUESTED_SLICES);

        assert_eq!(prediction.num_slices, REQUESTED_SLICES);
        assert_eq!(prediction.slices.len(), REQUESTED_SLICES);
    }

    #[test]
    fn check_num_slices_for_time() {
        const REQUESTED_TIME: f32 = 8.0;

        let mut game = load_soccar();

        let prediction = Ball::get_ball_prediction_struct_for_time(&mut game, &REQUESTED_TIME);

        let predicted_slices = (REQUESTED_TIME / Ball::SIMULATION_DT).round() as usize;

        assert_eq!(prediction.num_slices, predicted_slices);
        assert_eq!(prediction.slices.len(), predicted_slices);
    }
}
