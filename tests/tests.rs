use rand::random;
use rl_ball_sym::{glam::Vec3A, load_dropshot, load_hoops, load_standard, load_standard_throwback};

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

    for slice in ball_prediction {
        assert!(slice.location.z > 0.);
    }
}

#[test]
fn predict_standard() {
    let (game, mut ball) = load_standard();
    ball.velocity.z = -100.;

    let ball_prediction = ball.get_ball_prediction_struct(&game);
    assert_eq!(ball_prediction.len(), 720);

    for slice in ball_prediction {
        assert!(slice.location.z > 0.);
    }
}

#[test]
fn predict_hoops() {
    let (game, mut ball) = load_hoops();
    ball.velocity.z = -100.;

    let ball_prediction = ball.get_ball_prediction_struct(&game);
    assert_eq!(ball_prediction.len(), 720);

    for slice in ball_prediction {
        assert!(slice.location.z > 0.);
    }
}

#[test]
fn predict_dropshot() {
    let (game, mut ball) = load_dropshot();
    ball.velocity.z = -100.;

    let ball_prediction = ball.get_ball_prediction_struct(&game);
    assert_eq!(ball_prediction.len(), 720);

    for slice in ball_prediction {
        assert!(slice.location.z > 0.);
    }
}

#[test]
fn predict_throwback_standard() {
    let (game, mut ball) = load_standard_throwback();
    ball.velocity.z = -100.;

    let ball_prediction = ball.get_ball_prediction_struct(&game);
    assert_eq!(ball_prediction.len(), 720);

    for slice in ball_prediction {
        assert!(slice.location.z > 0.);
    }
}

#[inline]
fn rand_vec() -> Vec3A {
    Vec3A::new(
        f32::from(random::<u8>()),
        f32::from(random::<u8>()),
        f32::from(random::<u8>()),
    )
}

#[test]
fn check_for_nans_ball() {
    let (game, mut ball) = load_standard();

    for _ in 0..100 {
        ball.update(0., rand_vec(), rand_vec(), rand_vec());

        let ball_prediction = ball.get_ball_prediction_struct(&game);

        for slice in ball_prediction {
            assert!(slice.location.is_finite());
            assert!(slice.velocity.is_finite());
            assert!(slice.angular_velocity.is_finite());
        }
    }
}
