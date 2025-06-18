use rl_ball_sym::load_hoops;
use std::hint::black_box;

fn main() {
    for _ in 0..500 {
        black_box(load_hoops());
    }
}
