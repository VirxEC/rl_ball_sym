use glam::Vec3A;
use rocketsim_rs::{
    init,
    math::Vec3,
    sim::{Arena, ArenaConfig, BallState, GameMode},
};
use std::time::Instant;

const STEPS: u32 = 120 * 60 * 60 * 5;

fn main() {
    init(None);

    let mut arena = Arena::new(
        GameMode::Soccar,
        ArenaConfig {
            no_ball_rot: false,
            ..Default::default()
        },
        120,
    );

    arena.pin_mut().set_ball(BallState {
        pos: Vec3::new(0., 0., 200.),
        vel: Vec3::new(2000., 2000., 2000.),
        ..Default::default()
    });

    let start_time = Instant::now();

    arena.pin_mut().step(STEPS);

    let elapsed = start_time.elapsed();

    println!("RocketSim simulated 5 hours in {elapsed:?}");

    let (game, mut ball) = rl_ball_sym::load_standard();
    ball.location = Vec3A::new(0., 0., 200.);
    ball.velocity = Vec3A::new(2000., 2000., 2000.);

    let start_time = Instant::now();

    for _ in 0..STEPS {
        ball.step(&game, 1. / 120.);
    }

    let elapsed = start_time.elapsed();

    println!("RLBallSym simulated 5 hours in {elapsed:?}");
}
