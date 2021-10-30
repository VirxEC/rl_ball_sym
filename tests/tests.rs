use rand::Rng;
use rl_ball_sym::simulation::ball::Ball;
use rl_ball_sym::simulation::game::Game;
use rl_ball_sym::simulation::geometry::Aabb;
use rl_ball_sym::simulation::morton::Morton;
use rl_ball_sym::{load_dropshot, load_hoops, load_soccar, load_soccar_throwback};
use std::time::Instant;
use vvec3::Vec3;

static mut GAME_0: Option<Game> = None;
static mut GAME_1: Option<Game> = None;

#[test]
fn init() {
    let mut game: &mut Game;
    unsafe {
        GAME_0 = Some(load_soccar());
        game = GAME_0.as_mut().unwrap();
    }

    game.ball.location.z = 1900.;
    let ball_prediction_struct = Ball::get_ball_prediction_struct(&mut game);
    dbg!(ball_prediction_struct.num_slices);
}

#[test]
fn morton() {
    let global_box = Aabb {
        min: Vec3::new(-4096., -5120., 0.),
        max: Vec3::new(4096., 5120., 2044.),
    };

    let morton = Morton::from(&global_box);

    let box_ = Aabb {
        min: Vec3::new(-4095., -5119., 1.),
        max: Vec3::new(-4094., -5118., 2.),
    };

    // let code = morton.get_code(&box_);
    let c = (box_.min + box_.max) / 2.;

    let u = (c - morton.offset) * morton.scale;
    dbg!(&u);

    let code_x = Morton::expand3(u.x as u32);
    dbg!(code_x);
    dbg!(format!("{:b}", u.x as u32));
    dbg!(format!("{:b}", code_x));
    assert_eq!(format!("{:b}", code_x), "1000001001001001001001"); // 001000001001001001001001

    let code_y = Morton::expand3(u.y as u32) << 1;
    dbg!(code_y);
    dbg!(format!("{:b}", u.y as u32));
    dbg!(format!("{:b}", code_y));
    assert_eq!(format!("{:b}", code_y), "10000000010010000000010"); // 010000000010010000000010

    let code_z = Morton::expand3(u.z as u32) << 2;
    dbg!(code_z);
    dbg!(format!("{:b}", u.z as u32));
    dbg!(format!("{:b}", code_z));
    assert_eq!(format!("{:b}", code_z), "100100000000000000000000000100");

    let code = code_z | code_y | code_x;
    dbg!(&code); // 610317903
    assert_eq!(format!("{:b}", code as u32), "100100011000001011011001001111");

    // assert!(false);
}

#[test]
fn gamemode_soccar() {
    let game = load_soccar();

    // test all the default values to make sure they're proper

    assert_eq!(game.gravity.x as i64, 0);
    assert_eq!(game.gravity.y as i64, 0);
    assert_eq!(game.gravity.z as i64, -650);

    dbg!(game.collision_mesh.root.box_);

    assert_eq!(game.collision_mesh.num_leaves, 8028 as u64);

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

    assert_eq!(game.gravity.x as i64, 0);
    assert_eq!(game.gravity.y as i64, 0);
    assert_eq!(game.gravity.z as i64, -650);

    dbg!(game.collision_mesh.root.box_);

    assert_eq!(game.collision_mesh.num_leaves, 15732 as u64);

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

    assert_eq!(game.gravity.x as i64, 0);
    assert_eq!(game.gravity.y as i64, 0);
    assert_eq!(game.gravity.z as i64, -650);

    dbg!(game.collision_mesh.root.box_);

    assert_eq!(game.collision_mesh.num_leaves, 3616 as u64);

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
fn gamemode_throwback_soccar() {
    let game = load_soccar_throwback();

    // test all the default values to make sure they're proper

    assert_eq!(game.gravity.x as i64, 0);
    assert_eq!(game.gravity.y as i64, 0);
    assert_eq!(game.gravity.z as i64, -650);

    dbg!(&game.collision_mesh.root.box_);
    dbg!(&game.collision_mesh.root.left.as_deref().unwrap().box_);
    dbg!(&game.collision_mesh.root.right.as_deref().unwrap().box_);

    assert_eq!(game.collision_mesh.num_leaves, 9272);

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
fn fast_init() {
    let runs = 200;
    let mut times = Vec::new();

    for _ in 0..runs {
        let mut game: &mut Game;

        let start = Instant::now();
        unsafe {
            if GAME_1.is_none() {
                GAME_1 = Some(load_soccar());
            }

            game = GAME_1.as_mut().unwrap();
        }

        game.ball.location.z = 1900.;
        Ball::get_ball_prediction_struct(&mut game);

        times.push(start.elapsed().as_secs_f32());
    }

    let elapsed: f32 = times.iter().sum::<f32>() / (runs as f32);
    let elapsed_ms = elapsed * 1000.;
    println!("Ran test ticks in an average of {} seconds ({}ms)", elapsed, &elapsed_ms);
    assert!(elapsed_ms < 1.);
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
    assert!(elapsed_ms < 4.);
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
    assert!(elapsed_ms < 6.);
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
    assert!(elapsed_ms < 3.);
}

#[test]
fn fast_start_throwback_soccar() {
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
    assert!(elapsed_ms < 3.);
}

#[test]
fn basic_predict() {
    let mut game = load_soccar();

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

    game.ball.update(0.098145, Vec3::new(-2294.524658, 1684.135986, 317.176727), Vec3::new(1273.753662, -39.792305, 763.282715), Vec3::new(2.3894, -0.8755, 3.8078));

    let start = Instant::now();
    let time = 60.; // 1 minute, lol
    let ball_prediction = Ball::get_ball_prediction_struct_for_time(&mut game, &time);
    println!("Ran ball prediction in {}", start.elapsed().as_secs_f32());
    assert_eq!(ball_prediction.num_slices, time as usize * 120);

    let iters = 2000;
    let time = 10.; // 10 seconds
    let num_slices = time as usize * 120 * iters;
    let mut rng = rand::thread_rng();

    let mut x_locs = Vec::with_capacity(num_slices);
    let mut y_locs = Vec::with_capacity(num_slices);
    let mut z_locs = Vec::with_capacity(num_slices);

    dbg!(game.collision_mesh.global_box);

    for _ in 0..iters {
        game.ball.update(0., Vec3::new(rng.gen_range(-3900.0..3900.), rng.gen_range(-5000.0..5000.), rng.gen_range(100.0..1900.)), Vec3::new(rng.gen_range(-2000.0..2000.), rng.gen_range(-2000.0..2000.), rng.gen_range(-2000.0..2000.)), Vec3::new(rng.gen_range(-3.0..3.), rng.gen_range(-3.0..3.), rng.gen_range(-3.0..3.)));

        let ball_prediction = Ball::get_ball_prediction_struct(&mut game);

        for slice in ball_prediction.slices {
            if slice.location.y.abs() > 5120. + slice.radius {
                break;
            }

            x_locs.push(slice.location.x as isize);
            y_locs.push(slice.location.y as isize);
            z_locs.push(slice.location.z as isize);
        }
    }

    dbg!(*x_locs.iter().min().unwrap());
    dbg!(*x_locs.iter().max().unwrap());

    dbg!(*y_locs.iter().min().unwrap());
    dbg!(*y_locs.iter().max().unwrap());

    dbg!(*z_locs.iter().min().unwrap());
    dbg!(*z_locs.iter().max().unwrap());

    assert!(*z_locs.iter().min().unwrap() > game.collision_mesh.global_box.min.z as isize);
    assert!(*z_locs.iter().max().unwrap() < game.collision_mesh.global_box.max.z as isize);

    assert!(*y_locs.iter().min().unwrap() > game.collision_mesh.global_box.min.y as isize);
    assert!(*y_locs.iter().max().unwrap() < game.collision_mesh.global_box.max.y as isize);

    assert!(*x_locs.iter().min().unwrap() > game.collision_mesh.global_box.min.x as isize);
    assert!(*x_locs.iter().max().unwrap() < game.collision_mesh.global_box.max.x as isize);
}

#[test]
fn fast_predict_soccar() {
    let mut game = load_soccar();
    let runs = 200;
    let mut times: Vec<f32> = Vec::with_capacity(runs);
    println!("Testing for average ball prediction struct generation time - running function {} times.", &runs);

    for _ in 0..runs {
        let start = Instant::now();
        Ball::get_ball_prediction_struct(&mut game);
        times.push(start.elapsed().as_secs_f32());
    }

    let elapsed: f32 = times.iter().sum::<f32>() / (runs as f32);
    let elapsed_ms = elapsed * 1000.;
    println!("Ran ball prediction on soccar map in an average of {} seconds ({}ms)", &elapsed, elapsed_ms);

    let ball_prediction = Ball::get_ball_prediction_struct(&mut game);
    assert_eq!(ball_prediction.num_slices, 720);
    assert!(elapsed_ms < 1.);
}

#[test]
fn fast_predict_hoops() {
    let mut game = load_hoops();
    let runs = 200;
    let mut times: Vec<f32> = Vec::with_capacity(runs);
    println!("Testing for average ball prediction struct generation time - running function {} times.", &runs);

    for _ in 0..runs {
        let start = Instant::now();
        Ball::get_ball_prediction_struct(&mut game);
        times.push(start.elapsed().as_secs_f32());
    }

    let elapsed: f32 = times.iter().sum::<f32>() / (runs as f32);
    let elapsed_ms = elapsed * 1000.;
    println!("Ran ball prediction on hoops map in an average of {} seconds ({}ms)", &elapsed, elapsed_ms);

    let ball_prediction = Ball::get_ball_prediction_struct(&mut game);
    assert_eq!(ball_prediction.num_slices, 720);
    assert!(elapsed_ms < 1.);
}

#[test]
fn fast_predict_dropshot() {
    let mut game = load_dropshot();
    let runs = 200;
    let mut times: Vec<f32> = Vec::with_capacity(runs);
    println!("Testing for average ball prediction struct generation time - running function {} times.", &runs);

    for _ in 0..runs {
        let start = Instant::now();
        Ball::get_ball_prediction_struct(&mut game);
        times.push(start.elapsed().as_secs_f32());
    }

    let elapsed: f32 = times.iter().sum::<f32>() / (runs as f32);
    let elapsed_ms = elapsed * 1000.;
    println!("Ran ball prediction on dropshot map in an average of {} seconds ({}ms)", &elapsed, elapsed_ms);

    let ball_prediction = Ball::get_ball_prediction_struct(&mut game);
    assert_eq!(ball_prediction.num_slices, 720);
    assert!(elapsed_ms < 1.);
}

#[test]
fn fast_predict_throwback_soccar() {
    let mut game = load_soccar_throwback();

    let runs = 50;
    let mut times: Vec<f32> = Vec::with_capacity(runs);
    println!("Testing for average ball prediction struct generation time - running function {} times.", &runs);

    for _ in 0..runs {
        let start = Instant::now();
        Ball::get_ball_prediction_struct(&mut game);
        times.push(start.elapsed().as_secs_f32());
    }

    let elapsed: f32 = times.iter().sum::<f32>() / (runs as f32);
    let elapsed_ms = elapsed * 1000.;
    println!("Ran ball prediction on throwback stadium in an average of {} seconds ({}ms)", &elapsed, elapsed_ms);

    let ball_prediction = Ball::get_ball_prediction_struct(&mut game);
    assert_eq!(ball_prediction.num_slices, 720);
    assert!(elapsed_ms < 1.);
}
