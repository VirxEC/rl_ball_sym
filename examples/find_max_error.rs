use byteorder::{LittleEndian, WriteBytesExt};
use rand::Rng;
use rl_ball_sym::load_standard;
use rocketsim_rs::{
    math::Vec3,
    sim::{Arena, BallState},
};
use std::{
    fs,
    io::{self, Write},
};

fn write_ball(file: &mut fs::File, ball: BallState, time: f32) -> io::Result<()> {
    file.write_f32::<LittleEndian>(time)?;
    file.write_f32::<LittleEndian>(ball.pos.x)?;
    file.write_f32::<LittleEndian>(ball.pos.y)?;
    file.write_f32::<LittleEndian>(ball.pos.z)?;
    file.write_f32::<LittleEndian>(ball.vel.x)?;
    file.write_f32::<LittleEndian>(ball.vel.y)?;
    file.write_f32::<LittleEndian>(ball.vel.z)?;
    file.write_f32::<LittleEndian>(ball.ang_vel.x)?;
    file.write_f32::<LittleEndian>(ball.ang_vel.y)?;
    file.write_f32::<LittleEndian>(ball.ang_vel.z)?;
    Ok(())
}

const TRIES: usize = 1_000_000;
const NUM_TICKS: i32 = 120;

fn main() -> io::Result<()> {
    rocketsim_rs::init(None, true);

    let mut arena = Arena::default_standard();
    let (game, mut ball) = load_standard();

    let mut rng = rand::rng();

    let mut max_error = 0.;
    let mut error_values = [Vec3::ZERO; 3];

    'outer: for i in 0..TRIES {
        let location = Vec3::new(
            rng.random_range(0.0..2900.),
            rng.random_range(0.0..3900.),
            rng.random_range(100.0..1900.),
        );
        let velocity = Vec3::new(
            rng.random_range(0.0..2000.),
            rng.random_range(0.0..2000.),
            rng.random_range(-2000.0..2000.),
        );
        let angular_velocity = Vec3::new(
            rng.random_range(-3.0..3.),
            rng.random_range(-3.0..3.),
            rng.random_range(-3.0..3.),
        );

        let mut rocketsim_ball = arena.pin_mut().get_ball();
        rocketsim_ball.pos = location;
        rocketsim_ball.vel = velocity;
        rocketsim_ball.ang_vel = angular_velocity;
        arena.pin_mut().set_ball(rocketsim_ball);

        ball.update(
            0.,
            location.into(),
            velocity.into(),
            angular_velocity.into(),
        );
        for _ in 0..NUM_TICKS {
            let last_ball = arena.pin_mut().get_ball();
            ball.step(&game, 1. / 120.);
            arena.pin_mut().step(1);

            let rocketsim_ball = arena.pin_mut().get_ball();

            let pos_error = ball.location.distance(rocketsim_ball.pos.into());
            let vel_error = ball.velocity.distance(rocketsim_ball.vel.into());
            let error = pos_error / 2. + vel_error / 2.;
            if error > max_error && error < 100. {
                max_error = error;
                error_values[0] = last_ball.pos;
                error_values[1] = last_ball.vel;
                error_values[2] = last_ball.ang_vel;

                if (max_error > 30. && i > TRIES / 2) || max_error > 80. {
                    println!(
                        "Breaking early with an error of {max_error}; Location error: {pos_error}, Velocity error: {vel_error}"
                    );
                    break 'outer;
                }

                break;
            }

            ball.location = rocketsim_ball.pos.into();
            ball.velocity = rocketsim_ball.vel.into();
            ball.angular_velocity = rocketsim_ball.ang_vel.into();
        }

        if i % 1000 == 0 {
            print!("Max error in {i} / {TRIES}: {max_error:.0}\r");
            io::stdout().flush()?;
        }
    }

    println!("\n");
    println!(
        "Location: ({}, {}, {})",
        error_values[0].x, error_values[0].y, error_values[0].z
    );
    println!(
        "Velocity: ({}, {}, {})",
        error_values[1].x, error_values[1].y, error_values[1].z
    );
    println!(
        "Angular velocity: ({}, {}, {})",
        error_values[2].x, error_values[2].y, error_values[2].z
    );

    let mut ball = arena.pin_mut().get_ball();
    ball.pos = error_values[0];
    ball.vel = error_values[1];
    ball.ang_vel = error_values[2];
    arena.pin_mut().set_ball(ball);

    let mut file = fs::File::create("examples/ball.dump")?;
    file.write_u16::<LittleEndian>(2)?;
    write_ball(&mut file, ball, 0.)?;
    arena.pin_mut().step(1);
    let ball = arena.pin_mut().get_ball();
    write_ball(&mut file, ball, 1. / 120.)?;

    Ok(())
}
