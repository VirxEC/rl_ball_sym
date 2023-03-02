#[cfg(feature = "compression")]
use rl_ball_sym::compressed::{load_dropshot, load_hoops, load_standard, load_standard_throwback};
use rl_ball_sym::glam::Vec3A;
#[cfg(all(feature = "uncompressed", not(feature = "compression")))]
use rl_ball_sym::{load_dropshot, load_hoops, load_standard, load_standard_throwback};

#[test]
fn init() {
    let (game, mut ball) = load_standard();
    ball.location.z = 1900.;

    let ball_prediction_struct = ball.get_ball_prediction_struct(&game);
    dbg!(ball_prediction_struct.len());
}

#[test]
fn predict_custom_standard() {
    let (game, mut ball) = load_standard();
    let time = 8.;
    ball.velocity.z = -100.;

    let ball_prediction = ball.get_ball_prediction_struct_for_time(&game, time);
    assert_eq!(ball_prediction.len(), 960);
}

#[test]
fn predict_standard() {
    let (game, mut ball) = load_standard();
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
fn predict_throwback_standard() {
    let (game, mut ball) = load_standard_throwback();
    ball.velocity.z = -100.;

    let ball_prediction = ball.get_ball_prediction_struct(&game);
    assert_eq!(ball_prediction.len(), 720);
}

#[test]
fn check_for_nans_ball() {
    let (game, mut ball) = load_standard();

    ball.update(0., Vec3A::new(0., 0., 100.), Vec3A::new(0., 0., f32::EPSILON), Vec3A::ZERO);

    let ball_prediction = ball.get_ball_prediction_struct(&game);

    for slice in ball_prediction {
        assert!(slice.location.is_finite());
        assert!(slice.velocity.is_finite());
        assert!(slice.angular_velocity.is_finite());
    }
}
