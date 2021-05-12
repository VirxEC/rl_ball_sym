use rl_ball_sym::{linear_algebra::vector::Vec3};
use rl_ball_sym::simulation::ball::Ball;
use rl_ball_sym::simulation::game::Game;
use rl_ball_sym::{load_dropshot, load_hoops, load_soccar, load_soccar_throwback};
use std::fs;
use std::time::Instant;

#[macro_use]
extern crate json;

static mut GAME: Option<Game> = None;

#[test]
fn init() {
    let mut game: Game;
    unsafe {
        GAME = Some(load_soccar());
        game = GAME.clone().unwrap();
    }

    game.ball.location.z = 1900.;
    let ball_prediction_struct = game.ball.get_ball_prediction_struct(&game);
    dbg!(ball_prediction_struct.num_slices);
}

#[test]
fn gamemode_soccar() {
    let game = load_soccar();

    // test all the default values to make sure they're proper

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
fn gamemode_hoops() {
    let game = load_hoops();

    // test all the default values to make sure they're proper

    assert_eq!(game.field.field_mesh.ids.len(), 47196);
    assert_eq!(game.field.field_mesh.vertices.len(), 25224);

    assert_eq!(game.gravity.x as i64, 0);
    assert_eq!(game.gravity.y as i64, 0);
    assert_eq!(game.gravity.z as i64, -650);

    dbg!(game.field.collision_mesh.root.box_);

    assert_eq!(game.field.collision_mesh.num_leaves, 15732 as u64);

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
fn gamemode_dropshot() {
    let game = load_dropshot();

    // test all the default values to make sure they're proper

    assert_eq!(game.field.field_mesh.ids.len(), 10848);
    assert_eq!(game.field.field_mesh.vertices.len(), 5766);

    assert_eq!(game.gravity.x as i64, 0);
    assert_eq!(game.gravity.y as i64, 0);
    assert_eq!(game.gravity.z as i64, -650);

    dbg!(game.field.collision_mesh.root.box_);

    assert_eq!(game.field.collision_mesh.num_leaves, 3616 as u64);

    assert_eq!(game.ball.time as i64, 0);
    assert_eq!(game.ball.location.x as i64, 0);
    assert_eq!(game.ball.location.y as i64, 0);
    assert_eq!(game.ball.location.z as i64, 113);
    assert_eq!(game.ball.velocity.x as i64, 0);
    assert_eq!(game.ball.velocity.y as i64, 0);
    assert_eq!(game.ball.velocity.z as i64, 0);
    assert_eq!(game.ball.angular_velocity.x as i64, 0);
    assert_eq!(game.ball.angular_velocity.y as i64, 0);
    assert_eq!(game.ball.angular_velocity.z as i64, 0);
    assert_eq!(game.ball.radius as i64, 100);
    assert_eq!(game.ball.collision_radius as i64, 103);
}

#[test]
fn gamemode_soccar_throwback() {
    let game = load_soccar_throwback();

    // test all the default values to make sure they're proper

    assert_eq!(game.field.field_mesh.ids.len(), 28044);
    assert_eq!(game.field.field_mesh.vertices.len(), 84096);

    assert_eq!(game.gravity.x as i64, 0);
    assert_eq!(game.gravity.y as i64, 0);
    assert_eq!(game.gravity.z as i64, -650);

    dbg!(game.field.collision_mesh.root.box_);
    dbg!(game.field.collision_mesh.root.left.unwrap().box_);
    dbg!(game.field.collision_mesh.root.right.unwrap().box_);

    assert_eq!(game.field.collision_mesh.num_leaves, 9348);

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
fn fast_start_soccar() {
    let runs = 200;
    let mut times = Vec::with_capacity(runs);

    for _ in 0..runs {
        let start = Instant::now();
        load_soccar();
        times.push(start.elapsed().as_secs_f32());
    }

    let elapsed: f32 = times.iter().sum::<f32>() / (runs as f32);
    let elapsed_ms = elapsed * 1000.;
    println!("Loaded soccar gamemode in an average of {} seconds ({}ms)", elapsed, &elapsed_ms);
    assert!(elapsed_ms < 6.);
}

#[test]
fn fast_start_hoops() {
    let runs = 200;
    let mut times = Vec::with_capacity(runs);

    for _ in 0..runs {
        let start = Instant::now();
        load_hoops();
        times.push(start.elapsed().as_secs_f32());
    }

    let elapsed: f32 = times.iter().sum::<f32>() / (runs as f32);
    let elapsed_ms = elapsed * 1000.;
    println!("Loaded hoops gamemode in an average of {} seconds ({}ms)", elapsed, &elapsed_ms);
    assert!(elapsed_ms < 11.);
}

#[test]
fn fast_start_dropshot() {
    let runs = 200;
    let mut times = Vec::with_capacity(runs);

    for _ in 0..runs {
        let start = Instant::now();
        load_dropshot();
        times.push(start.elapsed().as_secs_f32());
    }

    let elapsed: f32 = times.iter().sum::<f32>() / (runs as f32);
    let elapsed_ms = elapsed * 1000.;
    println!("Loaded dropshot gamemode in an average of {} seconds ({}ms)", elapsed, &elapsed_ms);
    assert!(elapsed_ms < 4.);
}

#[test]
fn fast_start_soccar_throwback() {
    let runs = 200;
    let mut times = Vec::with_capacity(runs);

    for _ in 0..runs {
        let start = Instant::now();
        load_dropshot();
        times.push(start.elapsed().as_secs_f32());
    }

    let elapsed: f32 = times.iter().sum::<f32>() / (runs as f32);
    let elapsed_ms = elapsed * 1000.;
    println!("Loaded soccar gamemode (throwback stadium) in an average of {} seconds ({}ms)", elapsed, &elapsed_ms);
    assert!(elapsed_ms < 4.);
}

#[test]
fn basic_predict() {
    let game = load_soccar();

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

    let ball = Ball {
        time: 0.098145,
        location: Vec3 {
            x: -2294.524658,
            y: 1684.135986,
            z: 317.176727,
        },
        velocity: Vec3 {
            x: 1273.753662,
            y: -39.792305,
            z: 763.282715,
        },
        angular_velocity: Vec3 {
            x: 2.3894,
            y: -0.8755,
            z: 3.8078,
        },
        radius: game.ball.radius,
        collision_radius: game.ball.collision_radius,
        moi: game.ball.moi,
    };

    let start = Instant::now();
    let time = 60.; // 1 minute, lol
    let ball_prediction = ball.get_ball_prediction_struct_for_time(&game, time);
    println!("Ran ball prediction in {}", start.elapsed().as_secs_f32());
    let last_slice = &ball_prediction.slices[ball_prediction.num_slices - 1];

    assert_eq!(ball_prediction.num_slices, time as usize * 120);
    println!("{:?}", last_slice);
    assert!(last_slice.location.z > 0.);

    let mut json_obj = json::JsonValue::new_array();
    for ball in ball_prediction.slices {
        json_obj
            .push(object! {
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
                },
                angular_velocity: object! {
                    x: ball.angular_velocity.x,
                    y: ball.angular_velocity.y,
                    z: ball.angular_velocity.z
                }
            })
            .unwrap();
    }
    fs::write("ball_prediction.json", json_obj.dump()).expect("Unable to write file");
}

#[test]
fn fast_predict_soccar() {
    let game = load_soccar();
    let runs = 200;
    let mut times: Vec<f32> = Vec::with_capacity(runs);
    println!("Testing for average ball prediction struct generation time - running function {} times.", &runs);

    for _ in 0..runs {
        let start = Instant::now();
        game.ball.get_ball_prediction_struct(&game);
        times.push(start.elapsed().as_secs_f32());
    }

    let elapsed: f32 = times.iter().sum::<f32>() / (runs as f32);
    let elapsed_ms = elapsed * 1000.;
    println!("Ran ball prediction on soccar map in an average of {} seconds ({}ms)", &elapsed, elapsed_ms);

    let ball_prediction = game.ball.get_ball_prediction_struct(&game);
    assert_eq!(ball_prediction.num_slices, 720);
    assert!(elapsed_ms < 1.);
}

#[test]
fn fast_predict_hoops() {
    let game = load_hoops();
    let runs = 200;
    let mut times: Vec<f32> = Vec::with_capacity(runs);
    println!("Testing for average ball prediction struct generation time - running function {} times.", &runs);

    for _ in 0..runs {
        let start = Instant::now();
        game.ball.get_ball_prediction_struct(&game);
        times.push(start.elapsed().as_secs_f32());
    }

    let elapsed: f32 = times.iter().sum::<f32>() / (runs as f32);
    let elapsed_ms = elapsed * 1000.;
    println!("Ran ball prediction on hoops map in an average of {} seconds ({}ms)", &elapsed, elapsed_ms);

    let ball_prediction = game.ball.get_ball_prediction_struct(&game);
    assert_eq!(ball_prediction.num_slices, 720);
    assert!(elapsed_ms < 1.);
}

#[test]
fn fast_predict_dropshot() {
    let game = load_dropshot();
    let runs = 200;
    let mut times: Vec<f32> = Vec::with_capacity(runs);
    println!("Testing for average ball prediction struct generation time - running function {} times.", &runs);

    for _ in 0..runs {
        let start = Instant::now();
        game.ball.get_ball_prediction_struct(&game);
        times.push(start.elapsed().as_secs_f32());
    }

    let elapsed: f32 = times.iter().sum::<f32>() / (runs as f32);
    let elapsed_ms = elapsed * 1000.;
    println!("Ran ball prediction on dropshot map in an average of {} seconds ({}ms)", &elapsed, elapsed_ms);

    let ball_prediction = game.ball.get_ball_prediction_struct(&game);
    assert_eq!(ball_prediction.num_slices, 720);
    assert!(elapsed_ms < 1.);
}

#[test]
fn fast_predict_soccar_throwback() {
    let game = load_soccar_throwback();
    let runs = 50;
    let mut times: Vec<f32> = Vec::with_capacity(runs);
    println!("Testing for average ball prediction struct generation time - running function {} times.", &runs);

    for _ in 0..runs {
        let start = Instant::now();
        game.ball.get_ball_prediction_struct(&game);
        times.push(start.elapsed().as_secs_f32());
    }

    let elapsed: f32 = times.iter().sum::<f32>() / (runs as f32);
    let elapsed_ms = elapsed * 1000.;
    println!("Ran ball prediction on throwback stadium in an average of {} seconds ({}ms)", &elapsed, elapsed_ms);

    let ball_prediction = game.ball.get_ball_prediction_struct(&game);
    assert_eq!(ball_prediction.num_slices, 720);
    assert!(elapsed_ms < 1.);
}
