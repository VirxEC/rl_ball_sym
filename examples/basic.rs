use glam::{vec3a, Vec3A};
use rl_ball_sym::load_soccar;
use rl_ball_sym::simulation::ball::{Ball, BallPrediction};
use rl_ball_sym::simulation::game::Game;

use rand::Rng;

static mut GAME: Option<Game> = None;

pub fn main() {
    let mut rng = rand::thread_rng();
    let mut time = 0.;

    // simulate our get_output function being called for 2 seconds - you can pretty much just ignore this.
    for _ in 0..240 {
        get_output(vec3a(rng.gen_range(-4000.0..4000.), rng.gen_range(-5020.0..5020.), rng.gen_range(0.0..1944.)), vec3a(rng.gen_range(-2000.0..2000.), rng.gen_range(-2000.0..2000.), rng.gen_range(-2000.0..2000.)), vec3a(rng.gen_range(-3.0..3.), rng.gen_range(-3.0..3.), rng.gen_range(-3.0..3.)), time);
        time += 1. / 120.;
    }
}

fn get_output(ball_location: Vec3A, ball_velocity: Vec3A, ball_angular_velocity: Vec3A, time: f32) {
    let game: &mut Game;

    unsafe {
        // if game is uninitialized, initialize soccar
        if GAME.is_none() {
            GAME = Some(load_soccar());
        }

        // get mutable reference to GAME and unwrap
        game = GAME.as_mut().unwrap();
    }

    game.ball.update(time, ball_location, ball_velocity, ball_angular_velocity);

    // generate the ball prediction struct
    // this is a list of 720 slices
    // it goes 6 seconds into the future with 120 slices per second
    let ball_prediction: BallPrediction = Ball::get_ball_prediction_struct(game);
    assert_eq!(ball_prediction.num_slices, 720);
    assert_eq!(ball_prediction.slices[ball_prediction.num_slices - 1].time.round() as i32, game.ball.time.round() as i32);
}
