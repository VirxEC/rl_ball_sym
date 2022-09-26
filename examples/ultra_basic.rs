use rl_ball_sym::{load_soccar, BallPrediction, Vec3A};

fn main() {
    // load a standard soccer match
    let (game, mut ball) = load_soccar();

    // the current state of the ball in the game
    ball.update(0., Vec3A::new(0., 0., 200.), Vec3A::new(0., 0., -0.1), Vec3A::new(0., 0., 0.));

    // generate the ball prediction struct
    // this is a list of 720 slices
    // it goes 6 seconds into the future with 120 slices per second
    let ball_prediction: BallPrediction = ball.get_ball_prediction_struct(&game);
    assert_eq!(ball_prediction.len(), 720);

    // ball is not modified, it stays the same!
    assert_eq!(ball.time, 0.);
}
