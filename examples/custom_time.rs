use rl_ball_sym::linear_algebra::vector::Vec3;
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
        get_output(
            Vec3 {
                x: rng.gen_range(-4000.0..4000.),
                y: rng.gen_range(-5020.0..5020.),
                z: rng.gen_range(0.0..1944.),
            },
            Vec3 {
                x: rng.gen_range(-2000.0..2000.),
                y: rng.gen_range(-2000.0..2000.),
                z: rng.gen_range(-2000.0..2000.),
            },
            Vec3 {
                x: rng.gen_range(-3.0..3.),
                y: rng.gen_range(-3.0..3.),
                z: rng.gen_range(-3.0..3.),
            },
            time,
        );
        time += 1. / 120.;
    }
}

fn get_output(ball_location: Vec3, ball_velocity: Vec3, ball_angular_velocity: Vec3, time: f32) {
    let game: &mut Game;

    unsafe {
        // if game is unintialized, initialize soccar
        if GAME.is_none() {
            GAME = Some(load_soccar());
        }

        // clone and unwrap GAME
        game = GAME.as_mut().unwrap();
    }

    // set the ball information in game
    game.ball.time = time;
    game.ball.location = ball_location;
    game.ball.velocity = ball_velocity;
    game.ball.angular_velocity = ball_angular_velocity;

    // generate the ball prediction struct for 12 seconds into the future
    // it generates 120 slices per second
    let prediction_time = 12.;
    let ball_prediction: BallPrediction = Ball::get_ball_prediction_struct_for_time(game, &prediction_time);
    assert_eq!(ball_prediction.num_slices, (120. * prediction_time).round() as usize);
    assert_eq!(ball_prediction.slices[ball_prediction.num_slices - 1].time.round() as i32, game.ball.time.round() as i32);
}
