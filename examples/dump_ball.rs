use std::{fs, io};

use byteorder::{LittleEndian, WriteBytesExt};
#[cfg(feature = "compression")]
use rl_ball_sym::compressed::load_standard;
#[cfg(all(feature = "uncompressed", not(feature = "compression")))]
use rl_ball_sym::load_standard;
use rl_ball_sym::Ball;

fn write_ball(file: &mut fs::File, ball: Ball) -> io::Result<()> {
    file.write_f32::<LittleEndian>(ball.time)?;
    file.write_f32::<LittleEndian>(ball.location.x)?;
    file.write_f32::<LittleEndian>(ball.location.y)?;
    file.write_f32::<LittleEndian>(ball.location.z)?;
    file.write_f32::<LittleEndian>(ball.velocity.x)?;
    file.write_f32::<LittleEndian>(ball.velocity.y)?;
    file.write_f32::<LittleEndian>(ball.velocity.z)?;
    file.write_f32::<LittleEndian>(ball.angular_velocity.x)?;
    file.write_f32::<LittleEndian>(ball.angular_velocity.y)?;
    file.write_f32::<LittleEndian>(ball.angular_velocity.z)?;

    Ok(())
}

fn main() -> io::Result<()> {
    let (game, mut ball) = load_standard();
    ball.location.z = 1800.;
    ball.velocity.x = 1000.;
    ball.velocity.y = 1000.;
    ball.velocity.z = 650.;

    let predictions = ball.get_ball_prediction_struct_for_time(&game, 12.);

    let mut file = fs::File::create("examples/ball.dump")?;
    file.write_u16::<LittleEndian>(1 + predictions.len() as u16)?;
    write_ball(&mut file, ball)?;

    for ball_slice in predictions {
        write_ball(&mut file, ball_slice)?;
    }

    Ok(())
}
