use std::sync::Mutex;

use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion};
use glam::Vec3A;
use lazy_static::lazy_static;
use rl_ball_sym::{
    load_dropshot, load_hoops, load_soccar, load_soccar_throwback,
    simulation::{ball::Ball, game::Game, geometry::Car},
};

lazy_static! {
    static ref GAME: Mutex<Game> = Mutex::new(load_soccar());
}

// This bench seems unnecssary, it seems to only test how long it takes to lock a mutex.
fn init_benchmark(c: &mut Criterion) {
    c.bench_function("init", |b| {
        b.iter(|| {
            let mut game = GAME.lock().unwrap();
            game.ball.location.z = 1900.;
            Ball::get_ball_prediction_struct(&mut game);
        })
    });
}

fn load_soccar_benchmark(c: &mut Criterion) {
    c.bench_function("load_soccar", |b| b.iter(load_soccar));
}

fn load_hoops_benchmark(c: &mut Criterion) {
    c.bench_function("load_hoops", |b| b.iter(load_hoops));
}

fn load_dropshot_benchmark(c: &mut Criterion) {
    c.bench_function("load_dropshot", |b| b.iter(load_dropshot));
}

fn load_soccar_throwback_benchmark(c: &mut Criterion) {
    c.bench_function("load_soccar_throwback", |b| b.iter(load_soccar_throwback));
}

fn get_ball_prediction_struct_with_time_benchmark(c: &mut Criterion) {
    let mut game = load_soccar();
    let time = 8.;

    c.bench_with_input(BenchmarkId::new("get_ball_prediction_struct_for_time", time), &time, |b, time| b.iter(|| Ball::get_ball_prediction_struct_for_time(black_box(&mut game), time)));
}

fn get_ball_prediction_struct_benchmark(c: &mut Criterion) {
    let mut game = load_soccar();
    c.bench_function("get_ball_prediction/soccar", |b| b.iter(|| Ball::get_ball_prediction_struct(black_box(&mut game))));
}

fn get_ball_prediction_struct_hoops_benchmark(c: &mut Criterion) {
    let mut game = load_hoops();

    c.bench_function("get_ball_prediction/hoops", |b| b.iter(|| Ball::get_ball_prediction_struct(black_box(&mut game))));
}

fn get_ball_prediction_struct_dropshot(c: &mut Criterion) {
    let mut game = load_dropshot();

    c.bench_function("get_ball_prediction/dropshot", |b| b.iter(|| Ball::get_ball_prediction_struct(black_box(&mut game))));
}

fn get_ball_prediction_struct_throwback(c: &mut Criterion) {
    let mut game = load_soccar_throwback();

    c.bench_function("get_ball_prediction/throwback", |b| b.iter(|| Ball::get_ball_prediction_struct(black_box(&mut game))));
}

fn get_ball_car_prediction_struct_benchmark(c: &mut Criterion) {
    let mut game = load_soccar();

    let location = Vec3A::new(0., 0., 17.01);
    let orientation = Car::new_orientation(Vec3A::new(1., 0., 0.), Vec3A::new(0., 1., 0.), Vec3A::new(0., 0., 1.));
    let hitbox = Car::new_hitbox(location, orientation, Vec3A::new(118., 84.2, 36.16), Vec3A::new(13.86, 0., 20.75));

    let mut car = Car::new(hitbox);
    car.update(location, Vec3A::default(), Vec3A::default(), orientation);

    c.bench_function("get_ball_car_prediction/soccar", |b| b.iter(|| Ball::get_ball_car_prediction_struct(black_box(&mut game), black_box(car))));
}

criterion_group!(init, init_benchmark, load_soccar_benchmark, load_hoops_benchmark, load_dropshot_benchmark, load_soccar_throwback_benchmark,);
criterion_group!(ball_prediction, get_ball_prediction_struct_with_time_benchmark, get_ball_prediction_struct_benchmark, get_ball_prediction_struct_hoops_benchmark, get_ball_prediction_struct_dropshot, get_ball_prediction_struct_throwback);
criterion_group!(ball_car_prediction, get_ball_car_prediction_struct_benchmark);
criterion_main!(init, ball_prediction, ball_car_prediction);
