use rl_ball_sym::{Vec3A, load_standard};
use std::hint::black_box;

fn main() {
    // load a standard standard match
    let (game, mut ball) = load_standard();

    // we only need to call load_standard once!
    for _ in 0..100000 {
        ball.update(
            0.,
            Vec3A::new(0., 0., 200.),
            Vec3A::new(600., 1550., -1200.),
            Vec3A::new(0., 0., 0.),
        );
        let predictions = ball.get_ball_prediction_struct_for_time(&game, 8.);
        black_box(predictions);
    }
}
