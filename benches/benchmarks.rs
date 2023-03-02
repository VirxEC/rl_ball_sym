use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion};
#[cfg(feature = "compression")]
use rl_ball_sym::compressed;
use rl_ball_sym::{load_dropshot, load_hoops, load_standard, load_standard_throwback};

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
    c.bench_function("load_standard_throwback", |b| b.iter(load_standard_throwback));
}

#[cfg(feature = "compression")]
fn compressed_load_standard_benchmark(c: &mut Criterion) {
    c.bench_function("compressed_load_standard", |b| b.iter(compressed::load_standard));
}

#[cfg(feature = "compression")]
fn compressed_load_hoops_benchmark(c: &mut Criterion) {
    c.bench_function("compressed_load_hoops", |b| b.iter(compressed::load_hoops));
}

#[cfg(feature = "compression")]
fn compressed_load_dropshot_benchmark(c: &mut Criterion) {
    c.bench_function("compressed_load_dropshot", |b| b.iter(compressed::load_dropshot));
}

#[cfg(feature = "compression")]
fn compressed_load_standard_throwback_benchmark(c: &mut Criterion) {
    c.bench_function("compressed_load_standard_throwback", |b| b.iter(compressed::load_standard_throwback));
}

fn get_ball_prediction_struct_with_time_benchmark(c: &mut Criterion) {
    let (game, mut ball) = load_standard();
    let time = 8.;
    ball.velocity.z = f32::EPSILON;

    c.bench_with_input(BenchmarkId::new("get_ball_prediction_struct_for_time", time), &time, |b, time| {
        b.iter(|| ball.get_ball_prediction_struct_for_time(black_box(&game), *time))
    });
}

fn get_ball_prediction_struct_benchmark(c: &mut Criterion) {
    let (game, mut ball) = load_standard();
    ball.velocity.z = f32::EPSILON;

    c.bench_function("get_ball_prediction/standard", |b| b.iter(|| ball.get_ball_prediction_struct(black_box(&game))));
}

fn get_ball_prediction_struct_hoops_benchmark(c: &mut Criterion) {
    let (game, mut ball) = load_hoops();
    ball.velocity.z = f32::EPSILON;

    c.bench_function("get_ball_prediction/hoops", |b| b.iter(|| ball.get_ball_prediction_struct(black_box(&game))));
}

fn get_ball_prediction_struct_dropshot(c: &mut Criterion) {
    let (game, mut ball) = load_dropshot();
    ball.velocity.z = f32::EPSILON;

    c.bench_function("get_ball_prediction/dropshot", |b| b.iter(|| ball.get_ball_prediction_struct(black_box(&game))));
}

fn get_ball_prediction_struct_throwback(c: &mut Criterion) {
    let (game, mut ball) = load_standard_throwback();
    ball.velocity.z = f32::EPSILON;

    c.bench_function("get_ball_prediction/throwback", |b| b.iter(|| ball.get_ball_prediction_struct(black_box(&game))));
}

criterion_group!(
    init,
    load_standard_benchmark,
    load_hoops_benchmark,
    load_dropshot_benchmark,
    load_standard_throwback_benchmark,
);
#[cfg(feature = "compression")]
criterion_group!(
    compressed_init,
    compressed_load_standard_benchmark,
    compressed_load_hoops_benchmark,
    compressed_load_dropshot_benchmark,
    compressed_load_standard_throwback_benchmark,
);
criterion_group!(
    ball_prediction,
    get_ball_prediction_struct_with_time_benchmark,
    get_ball_prediction_struct_benchmark,
    get_ball_prediction_struct_hoops_benchmark,
    get_ball_prediction_struct_dropshot,
    get_ball_prediction_struct_throwback
);

#[cfg(not(feature = "compression"))]
criterion_main!(init, ball_prediction);

#[cfg(feature = "compression")]
criterion_main!(init, compressed_init, ball_prediction);
