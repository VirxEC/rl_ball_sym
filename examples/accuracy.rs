use std::{fs, io};

use byteorder::{LittleEndian, ReadBytesExt};
use rl_ball_sym::Ball;

#[cfg(feature = "compression")]
use rl_ball_sym::compressed::load_standard;
#[cfg(all(feature = "uncompressed", not(feature = "compression")))]
use rl_ball_sym::load_standard;

fn read_balls(file_name: &str, mut ball: Ball) -> io::Result<Vec<Ball>> {
    let mut file = fs::File::open(file_name)?;
    let num_balls = file.read_u16::<LittleEndian>()? as usize;

    let mut balls = Vec::with_capacity(num_balls);

    for _ in 0..num_balls {
        ball.time = file.read_f32::<LittleEndian>()?;
        ball.location.x = file.read_f32::<LittleEndian>()?;
        ball.location.y = file.read_f32::<LittleEndian>()?;
        ball.location.z = file.read_f32::<LittleEndian>()?;
        ball.velocity.x = file.read_f32::<LittleEndian>()?;
        ball.velocity.y = file.read_f32::<LittleEndian>()?;
        ball.velocity.z = file.read_f32::<LittleEndian>()?;
        ball.angular_velocity.x = file.read_f32::<LittleEndian>()?;
        ball.angular_velocity.y = file.read_f32::<LittleEndian>()?;
        ball.angular_velocity.z = file.read_f32::<LittleEndian>()?;

        balls.push(ball);
    }

    Ok(balls)
}

fn main() -> io::Result<()> {
    let (game, mut ball) = load_standard();
    ball.location.z = 1800.;
    ball.velocity.x = 1000.;
    ball.velocity.y = 1000.;
    ball.velocity.z = 650.;

    let cballs = read_balls("examples/ball.dump", ball)?;

    assert_eq!(ball.time, cballs[0].time);
    assert_eq!(ball.location, cballs[0].location);
    assert_eq!(ball.velocity, cballs[0].velocity);
    assert_eq!(ball.angular_velocity, cballs[0].angular_velocity);

    for cball in cballs[1..].iter() {
        ball.step(&game, 1. / 120.);
        let vel_diff = ball.velocity - cball.velocity;

        if vel_diff.length_squared() > 0. {
            println!("First difference at time {}:", ball.time);
            println!("{}", ball.location - cball.location);
            println!("{}, {}: {vel_diff}", ball.velocity, cball.velocity);
            println!("{}", ball.angular_velocity - cball.angular_velocity);
            break;
        }
    }

    let last_cball = cballs.last().unwrap();
    let last_ball = *ball.get_ball_prediction_struct_for_time(&game, last_cball.time - ball.time).last().unwrap();

    println!("Advanced to time {}, aimed for {}:", last_ball.time, last_cball.time);
    println!("{}", last_ball.location - last_cball.location);
    println!("{}, {}: {}", last_ball.velocity, last_cball.velocity, last_ball.velocity - last_cball.velocity);
    println!("{}", last_ball.angular_velocity - last_cball.angular_velocity);

    Ok(())
}
