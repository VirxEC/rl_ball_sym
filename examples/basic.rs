use rl_ball_sym::linear_algebra::vector::Vec3;
use rl_ball_sym::load_soccar;
use rl_ball_sym::simulation::ball::BallPrediction;
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
                z: rng.gen_range(100.0..1944.),
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
    let mut game: &mut Game;

    unsafe {
        // if game is uninitialized, initialize soccar
        if GAME.is_none() {
            GAME = Some(load_soccar());
        }

        // get mutable reference to GAME and unwrap
        game = GAME.as_mut().unwrap();
    }

    // set the ball information in game
    game.ball.time = time;
    game.ball.location = ball_location;
    game.ball.velocity = ball_velocity;
    game.ball.angular_velocity = ball_angular_velocity;

    // generate the ball prediction struct
    // this is a list of 720 slices
    // it goes 6 seconds into the future with 120 slices per second
    let ball_prediction: BallPrediction = game.ball.get_ball_prediction_struct(&game);
    assert_eq!(ball_prediction.num_slices, 720);
    assert_eq!(ball_prediction.slices[ball_prediction.num_slices - 1].time.round() as i32, (game.ball.time.round() + 6.) as i32);
}
