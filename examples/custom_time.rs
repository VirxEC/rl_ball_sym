use rand::Rng;
use rl_ball_sym::{load_soccar, Ball, BallPrediction, Game, Vec3A};
use std::sync::RwLock;

// RwLock's can only have one reader, but they can have multiple writers enabling safe parallel access to the data once it's been initilized
static GAME: RwLock<Option<(Game, Ball)>> = RwLock::new(None);

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
    let read_lock = {
        let read_lock = GAME.read().expect("game lock poisoned");

        if read_lock.as_ref().is_some() {
            // return the read lock with the game data
            read_lock
        } else {
            // drop the read lock so we can grab the write lock
            drop(read_lock);
            // grab the write lock
            let mut write_lock = GAME.write().expect("game lock poisoned");
            // load a standard soccer match
            *write_lock = Some(load_soccar());
            // drop the write lock so we can read the game data
            drop(write_lock);
            // grab the read lock again
            GAME.read().expect("game lock poisoned")
        }
    };

    // Game is too complex and can't implement Copy, so the type of the variable game is &Game
    // Since we have to reference Game to avoid cloning it, we need to hold read_lock until the end of the function
    // Ball does implement Copy though, so we copy the basic ball data we got when we called load_soccar
    let (game, mut ball) = read_lock.as_ref().unwrap();

    // set the ball information from the 'game" (the random number generator in this case)
    ball.update(time, ball_location, ball_velocity, ball_angular_velocity);

    // generate the ball prediction struct for 12 seconds into the future
    // it generates 120 slices per second
    let prediction_time = 12.;
    let ball_prediction: BallPrediction = ball.get_ball_prediction_struct_for_time(game, prediction_time);
    assert_eq!(ball_prediction.len(), (120. * prediction_time).ceil() as usize);

    // ball is not modified, it stays the same!
    assert_eq!(ball.time, time);
}
