use std::hint::black_box;

use rl_ball_sym::Vec3A;

#[cfg(feature = "compression")]
use rl_ball_sym::compressed::load_standard;
#[cfg(all(feature = "uncompressed", not(feature = "compression")))]
use rl_ball_sym::load_standard;

fn main() {
    // load a standard standard match
    let (game, mut ball) = load_standard();

    // we only need to call load_standard once!
    for _ in 0..6000 {
        ball.update(0., Vec3A::new(0., 0., 200.), Vec3A::new(0., 0., -0.1), Vec3A::new(0., 0., 0.));
        black_box(ball.get_ball_prediction_struct_for_time(&game, 60.));
    }
}
