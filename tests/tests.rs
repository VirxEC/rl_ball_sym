use rand::Rng;
use rl_ball_sym::{
    glam::Vec3A,
    load_dropshot, load_hoops, load_soccar, load_soccar_throwback,
    simulation::{bvh::BvhNode, geometry::Aabb, morton::Morton},
};

#[test]
fn init() {
    let (game, mut ball) = load_soccar();
    ball.location.z = 1900.;

    let ball_prediction_struct = ball.get_ball_prediction_struct(&game);
    dbg!(ball_prediction_struct.len());
}

#[test]
fn morton() {
    let global_box = Aabb::from(Vec3A::new(-4096., -5120., 0.), Vec3A::new(4096., 5120., 2044.));

    let morton = Morton::from(&global_box);

    let box_ = Aabb::from(Vec3A::new(-4095., -5119., 1.), Vec3A::new(-4094., -5118., 2.));

    // let code = morton.get_code(&box_);
    let c = (box_.min() + box_.max()) / 2.;

    let u = (c - morton.offset()) * morton.scale();
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
}

#[test]
fn gamemode_soccar() {
    let (game, ball) = load_soccar();

    // test all the default values to make sure they're proper

    assert_eq!(game.gravity.x as i64, 0);
    assert_eq!(game.gravity.y as i64, 0);
    assert_eq!(game.gravity.z as i64, -650);

    dbg!(game.collision_mesh.root.box_());

    assert_eq!(game.collision_mesh.num_leaves, 8028u64);

    assert_eq!(ball.time as i64, 0);
    assert_eq!(ball.location.x as i64, 0);
    assert_eq!(ball.location.y as i64, 0);
    assert_eq!(ball.location.z as i64, 102);
    assert_eq!(ball.velocity.x as i64, 0);
    assert_eq!(ball.velocity.y as i64, 0);
    assert_eq!(ball.velocity.z as i64, 0);
    assert_eq!(ball.angular_velocity.x as i64, 0);
    assert_eq!(ball.angular_velocity.y as i64, 0);
    assert_eq!(ball.angular_velocity.z as i64, 0);
    assert_eq!(ball.radius as i64, 91);
    assert_eq!(ball.collision_radius as i64, 93);
}

#[test]
fn gamemode_hoops() {
    let (game, ball) = load_hoops();

    // test all the default values to make sure they're proper

    assert_eq!(game.gravity.x as i64, 0);
    assert_eq!(game.gravity.y as i64, 0);
    assert_eq!(game.gravity.z as i64, -650);

    dbg!(game.collision_mesh.root.box_());

    assert_eq!(game.collision_mesh.num_leaves, 15732u64);

    assert_eq!(ball.time as i64, 0);
    assert_eq!(ball.location.x as i64, 0);
    assert_eq!(ball.location.y as i64, 0);
    assert_eq!(ball.location.z as i64, 102);
    assert_eq!(ball.velocity.x as i64, 0);
    assert_eq!(ball.velocity.y as i64, 0);
    assert_eq!(ball.velocity.z as i64, 0);
    assert_eq!(ball.angular_velocity.x as i64, 0);
    assert_eq!(ball.angular_velocity.y as i64, 0);
    assert_eq!(ball.angular_velocity.z as i64, 0);
    assert_eq!(ball.radius as i64, 91);
    assert_eq!(ball.collision_radius as i64, 93);
}

#[test]
fn gamemode_dropshot() {
    let (game, ball) = load_dropshot();

    // test all the default values to make sure they're proper

    assert_eq!(game.gravity.x as i64, 0);
    assert_eq!(game.gravity.y as i64, 0);
    assert_eq!(game.gravity.z as i64, -650);

    dbg!(game.collision_mesh.root.box_());

    assert_eq!(game.collision_mesh.num_leaves, 3616u64);

    assert_eq!(ball.time as i64, 0);
    assert_eq!(ball.location.x as i64, 0);
    assert_eq!(ball.location.y as i64, 0);
    assert_eq!(ball.location.z as i64, 113);
    assert_eq!(ball.velocity.x as i64, 0);
    assert_eq!(ball.velocity.y as i64, 0);
    assert_eq!(ball.velocity.z as i64, 0);
    assert_eq!(ball.angular_velocity.x as i64, 0);
    assert_eq!(ball.angular_velocity.y as i64, 0);
    assert_eq!(ball.angular_velocity.z as i64, 0);
    assert_eq!(ball.radius as i64, 100);
    assert_eq!(ball.collision_radius as i64, 103);
}

#[test]
fn gamemode_throwback_soccar() {
    let (game, ball) = load_soccar_throwback();

    // test all the default values to make sure they're proper

    assert_eq!(game.gravity.x as i64, 0);
    assert_eq!(game.gravity.y as i64, 0);
    assert_eq!(game.gravity.z as i64, -650);

    dbg!(&game.collision_mesh.root.box_());
    if let BvhNode::Branch(branch) = &game.collision_mesh.root {
        dbg!(branch.left.box_());
        dbg!(branch.right.box_());
    }

    assert_eq!(game.collision_mesh.num_leaves, 9272);

    assert_eq!(ball.time as i64, 0);
    assert_eq!(ball.location.x as i64, 0);
    assert_eq!(ball.location.y as i64, 0);
    assert_eq!(ball.location.z as i64, 102);
    assert_eq!(ball.velocity.x as i64, 0);
    assert_eq!(ball.velocity.y as i64, 0);
    assert_eq!(ball.velocity.z as i64, 0);
    assert_eq!(ball.angular_velocity.x as i64, 0);
    assert_eq!(ball.angular_velocity.y as i64, 0);
    assert_eq!(ball.angular_velocity.z as i64, 0);
    assert_eq!(ball.radius as i64, 91);
    assert_eq!(ball.collision_radius as i64, 93);
}

#[test]
fn basic_predict_soccar() {
    let (game, mut ball) = load_soccar();

    assert_eq!(ball.time as i64, 0);
    assert_eq!(ball.location.x as i64, 0);
    assert_eq!(ball.location.y as i64, 0);
    assert_eq!(ball.location.z as i64, 102);
    assert_eq!(ball.velocity.x as i64, 0);
    assert_eq!(ball.velocity.y as i64, 0);
    assert_eq!(ball.velocity.z as i64, 0);
    assert_eq!(ball.angular_velocity.x as i64, 0);
    assert_eq!(ball.angular_velocity.y as i64, 0);
    assert_eq!(ball.angular_velocity.z as i64, 0);
    assert_eq!(ball.radius as i64, 91);
    assert_eq!(ball.collision_radius as i64, 93);

    ball.update(
        0.098145,
        Vec3A::new(-2294.5247, 1684.136, 317.17673),
        Vec3A::new(1273.7537, -39.792305, 763.2827),
        Vec3A::new(2.3894, -0.8755, 3.8078),
    );

    let time = 60.; // 1 minute, lol
    let ball_prediction = ball.get_ball_prediction_struct_for_time(&game, &time);
    assert_eq!(ball_prediction.len(), time as usize * 120);

    let iters = 20000;
    let time = 10.; // 10 seconds
    let num_slices = time as usize * 120 * iters;
    let mut rng = rand::thread_rng();

    let mut x_locs = Vec::with_capacity(num_slices);
    let mut y_locs = Vec::with_capacity(num_slices);
    let mut z_locs = Vec::with_capacity(num_slices);

    dbg!(game.collision_mesh.global_box);

    for _ in 0..iters {
        ball.update(
            0.,
            Vec3A::new(rng.gen_range(-3200.0..3200.), rng.gen_range(-4500.0..4500.), rng.gen_range(100.0..1900.)),
            Vec3A::new(rng.gen_range(-2000.0..2000.), rng.gen_range(-2000.0..2000.), rng.gen_range(-2000.0..2000.)),
            Vec3A::new(rng.gen_range(-3.0..3.), rng.gen_range(-3.0..3.), rng.gen_range(-3.0..3.)),
        );

        let ball_prediction = ball.get_ball_prediction_struct(&game);

        for slice in ball_prediction {
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

    assert!(*z_locs.iter().min().unwrap() > game.collision_mesh.global_box.min().z as isize);
    assert!(*z_locs.iter().max().unwrap() < game.collision_mesh.global_box.max().z as isize);

    assert!(*y_locs.iter().min().unwrap() > game.collision_mesh.global_box.min().y as isize);
    assert!(*y_locs.iter().max().unwrap() < game.collision_mesh.global_box.max().y as isize);

    assert!(*x_locs.iter().min().unwrap() > game.collision_mesh.global_box.min().x as isize);
    assert!(*x_locs.iter().max().unwrap() < game.collision_mesh.global_box.max().x as isize);
}

#[test]
fn basic_predict_throwback() {
    let (game, mut ball) = load_soccar_throwback();

    assert_eq!(ball.time as i64, 0);
    assert_eq!(ball.location.x as i64, 0);
    assert_eq!(ball.location.y as i64, 0);
    assert_eq!(ball.location.z as i64, 102);
    assert_eq!(ball.velocity.x as i64, 0);
    assert_eq!(ball.velocity.y as i64, 0);
    assert_eq!(ball.velocity.z as i64, 0);
    assert_eq!(ball.angular_velocity.x as i64, 0);
    assert_eq!(ball.angular_velocity.y as i64, 0);
    assert_eq!(ball.angular_velocity.z as i64, 0);
    assert_eq!(ball.radius as i64, 91);
    assert_eq!(ball.collision_radius as i64, 93);

    ball.update(
        0.098145,
        Vec3A::new(-2294.5247, 1684.136, 317.17673),
        Vec3A::new(1273.7537, -39.792305, 763.2827),
        Vec3A::new(2.3894, -0.8755, 3.8078),
    );

    let time = 60.; // 1 minute, lol
    let ball_prediction = ball.get_ball_prediction_struct_for_time(&game, &time);
    assert_eq!(ball_prediction.len(), time as usize * 120);

    let iters = 200;
    let time = 10.; // 10 seconds
    let num_slices = time as usize * 120 * iters;
    let mut rng = rand::thread_rng();

    let mut x_locs = Vec::with_capacity(num_slices);
    let mut y_locs = Vec::with_capacity(num_slices);
    let mut z_locs = Vec::with_capacity(num_slices);

    dbg!(game.collision_mesh.global_box);

    for _ in 0..iters {
        ball.update(
            0.,
            Vec3A::new(rng.gen_range(-3900.0..3900.), rng.gen_range(-5000.0..5000.), rng.gen_range(100.0..1900.)),
            Vec3A::new(rng.gen_range(-2000.0..2000.), rng.gen_range(-2000.0..2000.), rng.gen_range(-2000.0..2000.)),
            Vec3A::new(rng.gen_range(-3.0..3.), rng.gen_range(-3.0..3.), rng.gen_range(-3.0..3.)),
        );

        let ball_prediction = ball.get_ball_prediction_struct(&game);

        for slice in ball_prediction {
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

    assert!(*z_locs.iter().min().unwrap() > game.collision_mesh.global_box.min().z as isize);
    assert!(*z_locs.iter().max().unwrap() < game.collision_mesh.global_box.max().z as isize);

    assert!(*y_locs.iter().min().unwrap() > game.collision_mesh.global_box.min().y as isize);
    assert!(*y_locs.iter().max().unwrap() < game.collision_mesh.global_box.max().y as isize);

    assert!(*x_locs.iter().min().unwrap() > game.collision_mesh.global_box.min().x as isize);
    assert!(*x_locs.iter().max().unwrap() < game.collision_mesh.global_box.max().x as isize);
}

#[test]
fn predict_custom_soccar() {
    let (game, mut ball) = load_soccar();
    let time = 8.;

    let ball_prediction = ball.get_ball_prediction_struct_for_time(&game, &time);
    assert_eq!(ball_prediction.len(), 960);
}

#[test]
fn predict_soccar() {
    let (game, mut ball) = load_soccar();

    let ball_prediction = ball.get_ball_prediction_struct(&game);
    assert_eq!(ball_prediction.len(), 720);
}

#[test]
fn predict_hoops() {
    let (game, mut ball) = load_hoops();

    let ball_prediction = ball.get_ball_prediction_struct(&game);
    assert_eq!(ball_prediction.len(), 720);
}

#[test]
fn predict_dropshot() {
    let (game, mut ball) = load_dropshot();

    let ball_prediction = ball.get_ball_prediction_struct(&game);
    assert_eq!(ball_prediction.len(), 720);
}

#[test]
fn predict_throwback_soccar() {
    let (game, mut ball) = load_soccar_throwback();

    let ball_prediction = ball.get_ball_prediction_struct(&game);
    assert_eq!(ball_prediction.len(), 720);
}

#[test]
fn check_for_nans() {
    let (game, mut ball) = load_soccar();

    ball.update(0., Vec3A::new(0., 0., 100.), Vec3A::ZERO, Vec3A::ZERO);

    let ball_prediction = ball.get_ball_prediction_struct(&game);

    for slice in ball_prediction {
        assert!(slice.location.is_finite());
        assert!(slice.velocity.is_finite());
        assert!(slice.angular_velocity.is_finite());
    }
}

#[test]
fn check_for_nans_ball() {
    let (game, mut ball) = load_soccar();

    ball.update(0., Vec3A::new(0., 0., 100.), Vec3A::ZERO, Vec3A::ZERO);

    let ball_prediction = ball.get_ball_prediction_struct(&game);

    for slice in ball_prediction {
        assert!(slice.location.is_finite());
        assert!(slice.velocity.is_finite());
        assert!(slice.angular_velocity.is_finite());
    }
}
