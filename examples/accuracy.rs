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

#[derive(serde::Serialize)]
struct BallDump {
    rocketsim: Vec<Ball>,
    rl_ball_sym: Vec<Ball>,
}

fn main() -> io::Result<()> {
    let (game, mut ball) = load_standard();
    ball.location.z = 1800.;
    ball.velocity.x = 1000.;
    ball.velocity.y = 1000.;
    ball.velocity.z = 650.;

    let rocketsim = read_balls("examples/ball.dump", ball)?;

    assert_eq!(ball.time, rocketsim[0].time);
    assert_eq!(ball.location, rocketsim[0].location);
    assert_eq!(ball.velocity, rocketsim[0].velocity);
    assert_eq!(ball.angular_velocity, rocketsim[0].angular_velocity);

    let mut rl_ball_sym = Vec::with_capacity(rocketsim.len());
    rl_ball_sym.push(ball);

    for _ in 1..rocketsim.len() {
        ball.step(&game, 1. / 120.);
        rl_ball_sym.push(ball);
    }

    fs::write(
        "analysis/accuracy.json",
        serde_json::to_string(&BallDump { rocketsim, rl_ball_sym }).unwrap(),
    )?;

    Ok(())
}
