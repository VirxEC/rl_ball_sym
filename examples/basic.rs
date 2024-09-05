use rand::Rng;
use rl_ball_sym::{load_standard, Ball, Game, Predictions, Vec3A};
use std::sync::{LazyLock, RwLock};

// We only need to initialize everything once
static GAME: RwLock<LazyLock<(Game, Ball)>> = RwLock::new(LazyLock::new(load_standard));

fn main() {
    let mut rng = rand::thread_rng();
    let mut time = 0.;

    // Generate random inputs for our get_output function - you just ignore this
    for _ in 0..240 {
        get_output(
            Vec3A::new(
                rng.gen_range(-4000.0..4000.),
                rng.gen_range(-5020.0..5020.),
                rng.gen_range(0.0..1944.),
            ),
            Vec3A::new(
                rng.gen_range(-2000.0..2000.),
                rng.gen_range(-2000.0..2000.),
                rng.gen_range(-2000.0..2000.),
            ),
            Vec3A::new(rng.gen_range(-3.0..3.), rng.gen_range(-3.0..3.), rng.gen_range(-3.0..3.)),
            time,
        );
        time += 1. / 120.;
    }
}

// Run the simulation
fn get_output(ball_location: Vec3A, ball_velocity: Vec3A, ball_angular_velocity: Vec3A, time: f32) {
    let reader = GAME.read().expect("game lock poisoned");

    // Game is too complex and can't implement Copy, so the type of the variable game is &Game
    // Since we have to reference Game to avoid cloning it, we need to hold reader until the end of the function
    // Ball does implement Copy though, so we copy the basic ball data we got when we called load_standard
    let (game, mut ball) = (&reader.0, reader.1);

    ball.update(time, ball_location, ball_velocity, ball_angular_velocity);

    // generate the ball prediction struct
    // this is a list of 720 slices
    // it goes 6 seconds into the future with 120 slices per second
    let ball_prediction: Predictions = ball.get_ball_prediction_struct(game);
    assert_eq!(ball_prediction.len(), 720);

    // ball is not modified, it stays the same!
    assert_eq!(ball.time, time);
}
