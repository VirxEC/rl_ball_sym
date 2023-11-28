use colored::Colorize;
use glam::Vec3A;
use rand::Rng;

#[cfg(feature = "compression")]
use rl_ball_sym::compressed::load_standard;
#[cfg(all(feature = "uncompressed", not(feature = "compression")))]
use rl_ball_sym::load_standard;

fn main() {
    let (game, mut ball) = load_standard();

    let mut rng = rand::thread_rng();

    'outer: for _ in 0..1_000 {
        println!("{}", "[START]".bright_red());
        let location = Vec3A::new(
            rng.gen_range(0f32..3200.),
            rng.gen_range(0f32..3200.),
            rng.gen_range(200f32..1800.),
        );
        let velocity = Vec3A::new(
            rng.gen_range(0f32..2000.),
            rng.gen_range(0f32..2000.),
            rng.gen_range(0f32..2000.),
        );
        let angular_velocity = Vec3A::new(rng.gen_range(0f32..1.), rng.gen_range(0f32..1.), rng.gen_range(0f32..1.));

        ball.update(0., location, velocity, angular_velocity);

        for _ in 0..120 * 2 {
            ball.step(&game, 1. / 120.);
            if ball.location.z > 2_100.
                || ball.location.z < 0.
                || ball.location.y.abs() > 6_000.
                || ball.location.x.abs() > 4_100.
            {
                println!("Current slice: {ball:?}");

                break 'outer;
            }
        }
    }
}
