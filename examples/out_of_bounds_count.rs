use rand::Rng;
use rl_ball_sym::{glam::Vec3A, load_standard};
use std::io::{Write, stdout};

fn main() {
    let (game, mut ball) = load_standard();

    let mut rng = rand::rng();

    let num_iters = 100_000_000;
    let mut count = 0;

    for i in 0..num_iters {
        let location = Vec3A::new(
            rng.random_range(-2900.0..2900.),
            rng.random_range(-3900.0..3900.),
            rng.random_range(100.0..1900.),
        );
        let velocity = Vec3A::new(
            rng.random_range(-2000.0..2000.),
            rng.random_range(-2000.0..2000.),
            rng.random_range(-2000.0..2000.),
        );
        let angular_velocity = Vec3A::new(
            rng.random_range(-3.0..3.),
            rng.random_range(-3.0..3.),
            rng.random_range(-3.0..3.),
        );

        ball.update(0., location, velocity, angular_velocity);

        for _ in 0..120 * 6 {
            ball.step(&game, 1. / 120.);
            if ball.location.z > 2_100.
                || ball.location.z < 0.
                || ball.location.y.abs() > 6_000.
                || ball.location.x.abs() > 4_100.
            {
                // println!(
                //     "\nOut of bounds at tick {}! Location: {}, {}, {}",
                //     i, ball.location.x, ball.location.y, ball.location.z);
                count += 1;
                break;
            }
        }

        if i % 1000 == 0 {
            print!(
                "\rNumber of out of bounds: {count}/{i} ({:.6}%)",
                count as f32 / i as f32 * 100.
            );
            stdout().flush().unwrap();
        }
    }

    println!();
}
