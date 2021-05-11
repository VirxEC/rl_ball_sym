use rl_ball_sym::load_soccar;
#[macro_use]
extern crate json;
use std::time::Instant;
use std::fs;


#[test]
fn build() {
    let game = load_soccar(0, 0);

    // test all the default values to make sure they're proper

    assert_eq!(game.index, 0 as u8);
    assert_eq!(game.team, 0 as u8);

    assert_eq!(game.field.field_mesh.ids.len(), 24084);
    assert_eq!(game.field.field_mesh.vertices.len(), 13152);

    assert_eq!(game.gravity.x as i64, 0);
    assert_eq!(game.gravity.y as i64, 0);
    assert_eq!(game.gravity.z as i64, -650);

    dbg!(game.field.collision_mesh.root.box_);

    assert_eq!(game.field.collision_mesh.num_leaves, 8028 as u64);

    assert_eq!(game.ball.time as i64, 0);
    assert_eq!(game.ball.location.x as i64, 0);
    assert_eq!(game.ball.location.y as i64, 0);
    assert_eq!(game.ball.location.z as i64, 102);
    assert_eq!(game.ball.velocity.x as i64, 0);
    assert_eq!(game.ball.velocity.y as i64, 0);
    assert_eq!(game.ball.velocity.z as i64, 0);
    assert_eq!(game.ball.angular_velocity.x as i64, 0);
    assert_eq!(game.ball.angular_velocity.y as i64, 0);
    assert_eq!(game.ball.angular_velocity.z as i64, 0);
    assert_eq!(game.ball.radius as i64, 91);
    assert_eq!(game.ball.collision_radius as i64, 93);
}

#[test]
fn predict() {
    let game = load_soccar(0, 0);

    assert_eq!(game.ball.time as i64, 0);
    assert_eq!(game.ball.location.x as i64, 0);
    assert_eq!(game.ball.location.y as i64, 0);
    assert_eq!(game.ball.location.z as i64, 102);
    assert_eq!(game.ball.velocity.x as i64, 0);
    assert_eq!(game.ball.velocity.y as i64, 0);
    assert_eq!(game.ball.velocity.z as i64, 0);
    assert_eq!(game.ball.angular_velocity.x as i64, 0);
    assert_eq!(game.ball.angular_velocity.y as i64, 0);
    assert_eq!(game.ball.angular_velocity.z as i64, 0);
    assert_eq!(game.ball.radius as i64, 91);
    assert_eq!(game.ball.collision_radius as i64, 93);

    let mut ball = game.ball.clone();
    ball.location.z = 300.;
    ball.velocity.z = 600.;
    ball.velocity.x = 1000.;

    let start = Instant::now();
    let ball_prediction = ball.get_ball_prediction_struct(&game);
    println!("Ran ball prediction in {}ms", start.elapsed().as_millis());
    let last_slice = &ball_prediction.slices[ball_prediction.num_slices - 1];

    assert_eq!(ball_prediction.num_slices, 720);
    println!("{:?}", last_slice);
    assert!(last_slice.location.z > 0.);

    let mut json_obj =  json::JsonValue::new_array();
    for ball in ball_prediction.slices {
        json_obj.push(object! {
            time: ball.time,
            location: object! {
                x: ball.location.x,
                y: ball.location.y,
                z: ball.location.z
            },
            velocity: object! {
                x: ball.velocity.x,
                y: ball.velocity.y,
                z: ball.velocity.z
            }
        }).unwrap();
    }
    fs::write("ball_prediction.json", json_obj.dump()).expect("Unable to write file");
}

#[test]
fn time_limit() {
    let game = load_soccar(0, 0);

    let start = Instant::now();
    let ball_prediction = game.ball.get_ball_prediction_struct(&game);
    let elapsed = start.elapsed().as_millis();
    println!("Ran ball prediction in {}ms", &elapsed);

    assert_eq!(ball_prediction.num_slices, 720);
    assert!(elapsed < 2);
}
