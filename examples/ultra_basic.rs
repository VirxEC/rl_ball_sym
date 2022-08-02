use glam::Vec3A;
use rl_ball_sym::{load_soccar, simulation::ball::BallPrediction};

fn main() {
    let (game, mut ball) = load_soccar();

    // generate random values
    ball.update(0., Vec3A::new(0., 0., 200.), Vec3A::new(0., 0., 0.), Vec3A::new(0., 0., 0.));

    // generate the ball prediction struct
    // this is a list of 720 slices
    // it goes 6 seconds into the future with 120 slices per second
    let ball_prediction: BallPrediction = ball.get_ball_prediction_struct(&game);
    assert_eq!(ball_prediction.len(), 720);

    // game.ball is modified, it doesn't stay the same!
    assert_eq!((ball_prediction[ball_prediction.len() - 1].time * 1000.).round() as i32, (ball.time * 1000.).round() as i32);
}
