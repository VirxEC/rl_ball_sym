use rl_ball_sym::{Predictions, Vec3A, load_standard};

fn main() {
    // load a standard standard match
    let (game, mut ball) = load_standard();

    // the current state of the ball in the game
    ball.update(
        0.,
        Vec3A::new(0., 0., 200.),
        Vec3A::new(0., 0., -0.1),
        Vec3A::new(0., 0., 0.),
    );

    // generate the ball prediction struct for 12 seconds into the future
    // it generates 120 slices per second
    let prediction_time = 12.;
    let ball_prediction: Predictions =
        ball.get_ball_prediction_struct_for_time(&game, prediction_time);
    assert_eq!(
        ball_prediction.len(),
        (120. * prediction_time).ceil() as usize
    );

    // ball is not modified, it stays the same!
    assert_eq!(ball.time, 0.);
}
