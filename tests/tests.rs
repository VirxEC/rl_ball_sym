use rand::Rng;
use rl_ball_sym::linear_algebra::vector::Vec3;
use rl_ball_sym::simulation::ball::Ball;
use rl_ball_sym::simulation::bvh::BvhNode;
use rl_ball_sym::simulation::game::Game;
use rl_ball_sym::simulation::geometry::Aabb;
use rl_ball_sym::simulation::morton::Morton;
use rl_ball_sym::{load_dropshot, load_hoops, load_soccar, load_soccar_throwback};
use std::fs;
use std::time::Instant;

#[macro_use]
extern crate json;

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
    let ball_prediction_struct = game.ball.get_ball_prediction_struct(&game);
    dbg!(ball_prediction_struct.num_slices);
}

#[test]
fn morton() {
    let global_box = Aabb {
        min: Vec3 {
            x: -4096.,
            y: -5120.,
            z: 0.,
        },
        max: Vec3 {
            x: 4096.,
            y: 5120.,
            z: 2044.,
        },
    };

    let morton = Morton::from(&global_box);

    let box_ = Aabb {
        min: Vec3 {
            x: -4095.,
            y: -5119.,
            z: 1.,
        },
        max: Vec3 {
            x: -4094.,
            y: -5118.,
            z: 2.,
        },
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
fn gamemode_throwback_soccar() {
    let game = load_soccar_throwback();

    // test all the default values to make sure they're proper

    assert_eq!(game.field.field_mesh.ids.len(), 28044);
    assert_eq!(game.field.field_mesh.vertices.len(), 84096);

    assert_eq!(game.gravity.x as i64, 0);
    assert_eq!(game.gravity.y as i64, 0);
    assert_eq!(game.gravity.z as i64, -650);

    dbg!(&game.field.collision_mesh.root.box_);
    dbg!(&game.field.collision_mesh.root.left.as_deref().unwrap().box_);
    dbg!(&game.field.collision_mesh.root.right.as_deref().unwrap().box_);

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

    let mut collision_leaves = Vec::with_capacity(game.field.collision_mesh.num_leaves as usize);

    // Allocate traversal stack from thread-local memory,
    // and push NULL to indicate that there are no postponed nodes.
    let bvh_default = BvhNode::default();
    let mut stack: Vec<&BvhNode> = vec![&bvh_default; 32];
    let mut stack_ptr = 1;

    // Traverse nodes starting from the root.
    let mut node = &*game.field.collision_mesh.root;
    loop {
        // Check each child node for overlap.
        let left = node.left.as_deref();
        let right = node.right.as_deref();
        let mut traverse_left = false;
        let mut traverse_right = false;

        if left.is_some() {
            let left = left.unwrap();
            if left.is_terminal {
                collision_leaves.push(left);
            } else {
                // traverse when a query overlaps with an internal node
                traverse_left = true;
            }
        }

        if right.is_some() {
            let right = right.unwrap();
            if right.is_terminal {
                collision_leaves.push(right);
            } else {
                // traverse when a query overlaps with an internal node
                traverse_right = true;
            }
        }

        if !(traverse_left || traverse_right) {
            // pop
            stack_ptr -= 1;

            node = &stack[stack_ptr];
        } else if traverse_left {
            node = left.unwrap();

            if traverse_right {
                // push
                stack[stack_ptr] = right.unwrap();
                stack_ptr += 1;
            }
        } else {
            node = right.unwrap();
        }

        if stack_ptr == 0 {
            break;
        }
    }

    let mut json_obj = json::JsonValue::new_array();
    for leaf in &collision_leaves {
        json_obj
            .push(object! {
                morton: leaf.morton,
                box_: object! {
                    min: object! {
                        x: leaf.box_.min.x,
                        y: leaf.box_.min.y,
                        z: leaf.box_.min.z
                    },
                    max: object! {
                        x: leaf.box_.max.x,
                        y: leaf.box_.max.y,
                        z: leaf.box_.max.z
                    },
                },
            })
            .unwrap();
    }
    fs::write("throwback_leaves.json", json_obj.dump()).expect("Unable to write file");
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
        game.ball.get_ball_prediction_struct(&game);

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
    assert!(elapsed_ms < 5.);
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
    assert!(elapsed_ms < 10.);
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
    let ball_prediction = ball.get_ball_prediction_struct_for_time(&game, &time);
    println!("Ran ball prediction in {}", start.elapsed().as_secs_f32());
    assert_eq!(ball_prediction.num_slices, time as usize * 120);

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

    let iters = 2000;
    let time = 10.; // 10 seconds
    let num_slices = time as usize * 120 * iters;
    let mut rng = rand::thread_rng();

    let mut x_locs = Vec::with_capacity(num_slices);
    let mut y_locs = Vec::with_capacity(num_slices);
    let mut z_locs = Vec::with_capacity(num_slices);

    dbg!(game.field.collision_mesh.global_box);

    for _ in 0..iters {
        let ball = Ball {
            time: 0.,
            location: Vec3 {
                x: rng.gen_range(-3900.0..3900.),
                y: rng.gen_range(-5000.0..5000.),
                z: rng.gen_range(100.0..1900.),
            },
            velocity: Vec3 {
                x: rng.gen_range(-2000.0..2000.),
                y: rng.gen_range(-2000.0..2000.),
                z: rng.gen_range(-2000.0..2000.),
            },
            angular_velocity: Vec3 {
                x: rng.gen_range(-3.0..3.),
                y: rng.gen_range(-3.0..3.),
                z: rng.gen_range(-3.0..3.),
            },
            radius: game.ball.radius,
            collision_radius: game.ball.collision_radius,
            moi: game.ball.moi,
        };

        let ball_prediction = ball.get_ball_prediction_struct_for_time(&game, &time);
        for slice in ball_prediction.slices {
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

    assert!(*z_locs.iter().min().unwrap() > game.field.collision_mesh.global_box.min.z as isize);
    assert!(*z_locs.iter().max().unwrap() < game.field.collision_mesh.global_box.max.z as isize);

    assert!(*y_locs.iter().min().unwrap() > game.field.collision_mesh.global_box.min.y as isize);
    assert!(*y_locs.iter().max().unwrap() < game.field.collision_mesh.global_box.max.y as isize);

    assert!(*x_locs.iter().min().unwrap() > game.field.collision_mesh.global_box.min.x as isize);
    assert!(*x_locs.iter().max().unwrap() < game.field.collision_mesh.global_box.max.x as isize);
}

#[test]
fn fast_predict_soccar() {
    let game = load_soccar();
    let runs = 120000;
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
fn fast_predict_throwback_soccar() {
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
