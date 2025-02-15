use colored::Colorize;
use rand::Rng;
use rl_ball_sym::{glam::Vec3A, load_standard};

fn main() {
    let (game, mut ball) = load_standard();

    let mut rng = rand::rng();

    'outer: for _ in 0..1_000_000 {
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

        let slices = 120 * 6;
        let mut balls = Vec::with_capacity(slices);

        for _ in 0..slices {
            balls.push(ball);
            ball.step(&game, 1. / 120.);
            if ball.location.z > 2_100.
                || ball.location.z < 0.
                || ball.location.y.abs() > 6_000.
                || ball.location.x.abs() > 4_100.
            {
                println!("{}", "[START]".bright_red());
                for ball in balls {
                    println!(
                        "time: {}, location: {}, velocity: {}, angular_velocity: {}",
                        ball.time, ball.location, ball.velocity, ball.angular_velocity
                    );
                }

                break 'outer;
            }
        }
    }
}
