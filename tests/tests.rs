use rl_ball_sym::{glam::Vec3A, load_dropshot, load_hoops, load_soccar, load_soccar_throwback};

#[test]
fn init() {
    let (game, mut ball) = load_soccar();
    ball.location.z = 1900.;

    let ball_prediction_struct = ball.get_ball_prediction_struct(&game);
    dbg!(ball_prediction_struct.len());
}

#[test]
fn predict_custom_soccar() {
    let (game, mut ball) = load_soccar();
    let time = 8.;
    ball.velocity.z = -100.;

    let ball_prediction = ball.get_ball_prediction_struct_for_time(&game, time);
    assert_eq!(ball_prediction.len(), 960);
}

#[test]
fn predict_soccar() {
    let (game, mut ball) = load_soccar();
    ball.velocity.z = -100.;

    let ball_prediction = ball.get_ball_prediction_struct(&game);
    assert_eq!(ball_prediction.len(), 720);
}

#[test]
fn predict_hoops() {
    let (game, mut ball) = load_hoops();
    ball.velocity.z = -100.;

    let ball_prediction = ball.get_ball_prediction_struct(&game);
    assert_eq!(ball_prediction.len(), 720);
}

#[test]
fn predict_dropshot() {
    let (game, mut ball) = load_dropshot();
    ball.velocity.z = -100.;

    let ball_prediction = ball.get_ball_prediction_struct(&game);
    assert_eq!(ball_prediction.len(), 720);
}

#[test]
fn predict_throwback_soccar() {
    let (game, mut ball) = load_soccar_throwback();
    ball.velocity.z = -100.;

    let ball_prediction = ball.get_ball_prediction_struct(&game);
    assert_eq!(ball_prediction.len(), 720);
}

#[test]
fn check_for_nans_ball() {
    let (game, mut ball) = load_soccar();

    ball.update(0., Vec3A::new(0., 0., 100.), Vec3A::new(0., 0., f32::EPSILON), Vec3A::ZERO);

    let ball_prediction = ball.get_ball_prediction_struct(&game);

    for slice in ball_prediction {
        assert!(slice.location.is_finite());
        assert!(slice.velocity.is_finite());
        assert!(slice.angular_velocity.is_finite());
    }
}
