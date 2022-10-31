# rl_ball_sym

[![unsafe forbidden](https://img.shields.io/badge/unsafe-forbidden-success.svg)](https://github.com/rust-secure-code/safety-dance/)

Rust implementation of ball path prediction for Rocket League; Inspired by Samuel (Chip) P. Mish's C++ utils called [RLUtilities](https://github.com/samuelpmish/RLUtilities).

This crate also contains fixes to discovered errors stemming from the original repo.

## Running

Make sure you have Rust/Cargo installed, then just run `cargo test --release` in the terminal.

## Example implementations

Check out the examples folder! If you want to run them and don't know how:

```bat
cargo run --example example_name
```

For example, to run the example `basic.rs`:

```bat
cargo run --example basic
```

## Performance numbers

Numbers are from a system running Ubuntu 22.04.1 with a Ryzen 9 5900X with 3600MHz CL18 RAM.

Numbers _will_ vary depending on your system.

+ `load_soccer`: Loads 8028 triangles, executes in around `910µs`
+ `load_hoops`: Loads 15732 triangles, executes in around `1.82ms`
+ `load_dropshot`: Loads 3616 triangles, executes in around `410µs`
+ `load_soccer_throwback`: Loads 9272 triangles, executes in around `1.16ms`
+ `get_ball_prediction_struct_for_time`: Soccer + 8 seconds, executes in around `140µs`
+ `get_ball_prediction`: Soccer + 6 seconds, executes in around `110µs`
+ `get_ball_prediction`: Hoops + 6 seconds, executes in around `200µs`
+ `get_ball_prediction`: Dropshot + 6 seconds, executes in around `90µs`
+ `get_ball_prediction`: Soccer + Throwback Stadium + 6 seconds, executes in around `110µs`

## Features

 - `uncompressed`: Default feature. Enables the loading of uncompressed binary field data, which is faster but increases the size of the final binary.
 - `compression`: Nightly only - Minimize the size of the produced binaries by compressing the binary field data at compile time. Will slightly slow down `load_x()` functions. 
 - `stable-compression`: Does the same as `compression` using the same crate, but available for use in stable Rust.
 - `fast-math`: Enables the `fast-math` feature in the [`glam`](https://crates.io/crates/glam) crate.
