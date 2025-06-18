use criterion::{BenchmarkId, Criterion, criterion_group, criterion_main};
use glam::Vec3A;
use rl_ball_sym::{load_dropshot, load_hoops, load_standard, load_standard_throwback};
use std::hint::black_box;

fn load_standard_benchmark(c: &mut Criterion) {
    c.bench_function("load_standard", |b| b.iter(load_standard));
}

fn load_hoops_benchmark(c: &mut Criterion) {
    c.bench_function("load_hoops", |b| b.iter(load_hoops));
}

fn load_dropshot_benchmark(c: &mut Criterion) {
    c.bench_function("load_dropshot", |b| b.iter(load_dropshot));
}

fn load_standard_throwback_benchmark(c: &mut Criterion) {
    c.bench_function("load_standard_throwback", |b| {
        b.iter(load_standard_throwback);
    });
}

const BALL_VEL: Vec3A = Vec3A::new(600., 1550., 0.);

fn get_ball_prediction_struct_with_time_benchmark(c: &mut Criterion) {
    let (game, mut ball) = load_standard();
    let time = 8.;
    ball.velocity = BALL_VEL;

    c.bench_with_input(
        BenchmarkId::new("get_ball_prediction_struct_for_time", time),
        &time,
        |b, time| b.iter(|| ball.get_ball_prediction_struct_for_time(black_box(&game), *time)),
    );
}

fn get_ball_prediction_struct_benchmark(c: &mut Criterion) {
    let (game, mut ball) = load_standard();
    ball.velocity = BALL_VEL;

    c.bench_function("get_ball_prediction/standard", |b| {
        b.iter(|| ball.get_ball_prediction_struct(black_box(&game)));
    });
}

fn get_ball_prediction_struct_hoops_benchmark(c: &mut Criterion) {
    let (game, mut ball) = load_hoops();
    ball.velocity = BALL_VEL;

    c.bench_function("get_ball_prediction/hoops", |b| {
        b.iter(|| ball.get_ball_prediction_struct(black_box(&game)));
    });
}

fn get_ball_prediction_struct_dropshot(c: &mut Criterion) {
    let (game, mut ball) = load_dropshot();
    ball.velocity = BALL_VEL;

    c.bench_function("get_ball_prediction/dropshot", |b| {
        b.iter(|| ball.get_ball_prediction_struct(black_box(&game)));
    });
}

fn get_ball_prediction_struct_throwback(c: &mut Criterion) {
    let (game, mut ball) = load_standard_throwback();
    ball.velocity = BALL_VEL;

    c.bench_function("get_ball_prediction/throwback", |b| {
        b.iter(|| ball.get_ball_prediction_struct(black_box(&game)));
    });
}

criterion_group!(
    init,
    load_standard_benchmark,
    load_hoops_benchmark,
    load_dropshot_benchmark,
    load_standard_throwback_benchmark,
);
criterion_group!(
    ball_prediction,
    get_ball_prediction_struct_with_time_benchmark,
    get_ball_prediction_struct_benchmark,
    get_ball_prediction_struct_hoops_benchmark,
    get_ball_prediction_struct_dropshot,
    get_ball_prediction_struct_throwback
);

criterion_main!(init, ball_prediction);
