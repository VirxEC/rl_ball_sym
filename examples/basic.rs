use glam::Vec3A;
use rand::Rng;
use rl_ball_sym::{
    load_soccar,
    simulation::{
        ball::{Ball, BallPrediction},
        game::Game,
    },
};

static mut GAME: Option<Game> = None;
static mut BALL: Option<Ball> = None;

fn main() {
    let mut rng = rand::thread_rng();
    let mut time = 0.;

    // simulate our get_output function being called for 2 seconds - you can pretty much just ignore this.
    for _ in 0..240 {
        get_output(
            Vec3A::new(rng.gen_range(-4000.0..4000.), rng.gen_range(-5020.0..5020.), rng.gen_range(0.0..1944.)),
            Vec3A::new(rng.gen_range(-2000.0..2000.), rng.gen_range(-2000.0..2000.), rng.gen_range(-2000.0..2000.)),
            Vec3A::new(rng.gen_range(-3.0..3.), rng.gen_range(-3.0..3.), rng.gen_range(-3.0..3.)),
            time,
        );
        time += 1. / 120.;
    }
}

fn get_output(ball_location: Vec3A, ball_velocity: Vec3A, ball_angular_velocity: Vec3A, time: f32) {
    let game: &Game;
    let mut ball: Ball;

    unsafe {
        // if game is unintialized, initialize soccar
        if GAME.is_none() {
            let (game, ball) = load_soccar();
            GAME = Some(game);
            BALL = Some(ball);
        }

        // get a reference to GAME and unwrap
        game = GAME.as_ref().unwrap();
        // Ball implments the copy trait
        ball = BALL.unwrap();
    }

    ball.update(time, ball_location, ball_velocity, ball_angular_velocity);

    // generate the ball prediction struct
    // this is a list of 720 slices
    // it goes 6 seconds into the future with 120 slices per second
    let ball_prediction: BallPrediction = ball.get_ball_prediction_struct(game);
    assert_eq!(ball_prediction.len(), 720);

    // game.ball is modified, it doesn't stay the same!
    assert_eq!((ball_prediction[ball_prediction.len() - 1].time * 1000.).round() as i32, (ball.time * 1000.).round() as i32);
}
