use byteorder::{LittleEndian, ReadBytesExt};
use colored::Colorize;
use rl_ball_sym::{load_standard, Ball};
use std::{fs, io};

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

    let rocketsim = read_balls("examples/ball.dump", ball)?;

    let mut rl_ball_sym = Vec::with_capacity(rocketsim.len());
    rl_ball_sym.push(rocketsim[0]);

    ball = rocketsim[0];
    for cball in rocketsim.iter().copied().take(3) {
        print_error(ball, cball);

        println!("{}", "[START OF TICK]".green());
        ball = cball;
        ball.step(&game, 1. / 120.);
        rl_ball_sym.push(ball);
        println!("{}", "[END OF TICK]".red());
    }

    fs::write(
        "analysis/accuracy.json",
        serde_json::to_string(&BallDump { rocketsim, rl_ball_sym }).unwrap(),
    )?;

    Ok(())
}

fn print_error(ball: Ball, cball: Ball) {
    println!("{} | {}", ball.velocity / 50., cball.velocity / 50.);
    println!(
        "Error: {}, {}, {}",
        ball.location.distance(cball.location),
        ball.velocity.distance(cball.velocity),
        ball.angular_velocity.distance(cball.angular_velocity)
    );
    println!("Vel error: {}", ball.velocity - cball.velocity);
}
