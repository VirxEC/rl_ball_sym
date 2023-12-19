# rl_ball_sym

[![unsafe forbidden](https://img.shields.io/badge/unsafe-forbidden-success.svg)](https://github.com/rust-secure-code/safety-dance/)

[![forthebadge](https://forthebadge.com/images/badges/made-with-rust.svg)](https://forthebadge.com)

Rust implementation of Rocket League's ball physics;
Inspired by Samuel P. Mish's C++ utils called [RLUtilities](https://github.com/samuelpmish/RLUtilities)
and other miscellaneous performance improvements.

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

Numbers are with default features from a system running Ubuntu 23.10 with a Ryzen 9 5900X with 3600MHz CL18 RAM.

Numbers _will_ vary depending on your system.

+ `load_standard`: Loads 8028 triangles, executes in around `640µs`
+ `load_hoops`: Loads 15732 triangles, executes in around `1.30ms`
+ `load_dropshot`: Loads 3616 triangles, executes in around `300µs`
+ `load_standard_throwback`: Loads 9272 triangles, executes in around `805µs`
+ `get_ball_prediction_struct_for_time`: standard + 8 seconds, executes in around `130µs`
+ `get_ball_prediction`: standard + 6 seconds, executes in around `90µs`
+ `get_ball_prediction`: Hoops + 6 seconds, executes in around `120µs`
+ `get_ball_prediction`: Dropshot + 6 seconds, executes in around `110µs`
+ `get_ball_prediction`: standard + Throwback Stadium + 6 seconds, executes in around `100µs`

## Features

+ `standard`: Enable loading the standard map
+ `hoops`: Enable loading the hoops map
+ `dropshot`: Enable loading the dropshot map
+ `throwback`: Enable loading the throwback map (with standard game rules)
+ `compression`: Minimize the size of the produced binaries by compressing the binary field data at compile time. Will slightly slow down `load_x()` functions.
