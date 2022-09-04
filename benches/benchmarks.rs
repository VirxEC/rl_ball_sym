use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion};
use rl_ball_sym::{load_dropshot, load_hoops, load_soccar, load_soccar_throwback};

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
    let (game, ball) = load_soccar();
    let time = 8.;

    c.bench_with_input(BenchmarkId::new("get_ball_prediction_struct_for_time", time), &time, |b, time| {
        b.iter(|| ball.get_ball_prediction_struct_for_time(black_box(&game), time))
    });
}

fn get_ball_prediction_struct_benchmark(c: &mut Criterion) {
    let (game, ball) = load_soccar();

    c.bench_function("get_ball_prediction/soccar", |b| b.iter(|| ball.get_ball_prediction_struct(black_box(&game))));
}

fn get_ball_prediction_struct_hoops_benchmark(c: &mut Criterion) {
    let (game, ball) = load_hoops();

    c.bench_function("get_ball_prediction/hoops", |b| b.iter(|| ball.get_ball_prediction_struct(black_box(&game))));
}

fn get_ball_prediction_struct_dropshot(c: &mut Criterion) {
    let (game, ball) = load_dropshot();

    c.bench_function("get_ball_prediction/dropshot", |b| b.iter(|| ball.get_ball_prediction_struct(black_box(&game))));
}

fn get_ball_prediction_struct_throwback(c: &mut Criterion) {
    let (game, ball) = load_soccar_throwback();

    c.bench_function("get_ball_prediction/throwback", |b| b.iter(|| ball.get_ball_prediction_struct(black_box(&game))));
}

criterion_group!(init, load_soccar_benchmark, load_hoops_benchmark, load_dropshot_benchmark, load_soccar_throwback_benchmark,);
criterion_group!(
    ball_prediction,
    get_ball_prediction_struct_with_time_benchmark,
    get_ball_prediction_struct_benchmark,
    get_ball_prediction_struct_hoops_benchmark,
    get_ball_prediction_struct_dropshot,
    get_ball_prediction_struct_throwback
);
criterion_main!(init, ball_prediction);
