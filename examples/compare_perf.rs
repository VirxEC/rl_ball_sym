use glam::Vec3A;
use rocketsim_rs::{
    init,
    math::Vec3,
    sim::{Arena, ArenaConfig, BallState, GameMode},
};
use std::time::Instant;

const SECONDS: u32 = 25;
const STEPS: u32 = 120 * 30;
const NUM_ITERS: u32 = 2000;

fn main() {
    init(None, true);

    let mut arena = Arena::new(GameMode::Soccar, ArenaConfig::default(), 120);

    let mut times = Vec::new();
    for _ in 0..NUM_ITERS {
        arena.pin_mut().set_ball(BallState {
            pos: Vec3::new(0., 0., 200.),
            vel: Vec3::new(2000., 2000., 2000.),
            ..Default::default()
        });

        let start_time = Instant::now();

        arena.pin_mut().step(STEPS);

        times.push(start_time.elapsed());
    }

    let best_elapsed = times[NUM_ITERS as usize / 1000];
    println!("RocketSim simulates {SECONDS} seconds in {best_elapsed:?}");

    let (game, mut ball) = rl_ball_sym::load_standard();

    let mut times = Vec::new();
    for _ in 0..NUM_ITERS {
        ball.update(0., Vec3A::new(0., 0., 200.), Vec3A::new(2000., 2000., 2000.), Vec3A::ZERO);

        let start_time = Instant::now();

        for _ in 0..STEPS {
            ball.step(&game, 1. / 120.);
        }

        times.push(start_time.elapsed());
    }

    let best_elapsed = times[NUM_ITERS as usize / 1000];
    println!("RLBallSym simulates {SECONDS} seconds in {best_elapsed:?}");
}
