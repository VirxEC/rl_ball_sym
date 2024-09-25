# rl_ball_sym

[![unsafe forbidden](https://img.shields.io/badge/unsafe-forbidden-success.svg)](https://github.com/rust-secure-code/safety-dance/)

Rust implementation of Rocket League's ball physics;
Inspired by Samuel P. Mish's C++ utils called [RLUtilities](https://github.com/samuelpmish/RLUtilities)
with accuracy improvements from [RocketSim](https://github.com/ZealanL/RocketSim)
and other miscellaneous performance improvements.

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

Numbers are from a system running Ubuntu 23.10 with a Ryzen 9 5900X and 3600MHz CL18 RAM.

Numbers _will_ vary depending on your system. Only default features are enabled.

+ `load_standard`: Loads 8028 triangles, executes in around `655µs`
+ `load_hoops`: Loads 15732 triangles, executes in around `1.35ms`
+ `load_dropshot`: Loads 3616 triangles, executes in around `300µs`
+ `load_standard_throwback`: Loads 9272 triangles, executes in around `815µs`
+ `get_ball_prediction_struct_for_time`: standard + 8 seconds, executes in around `180µs`
+ `get_ball_prediction`: standard + 6 seconds, executes in around `145µs`
+ `get_ball_prediction`: Hoops + 6 seconds, executes in around `140µs`
+ `get_ball_prediction`: Dropshot + 6 seconds, executes in around `115µs`
+ `get_ball_prediction`: standard + Throwback Stadium + 6 seconds, executes in around `145µs`

## Features

+ `standard`: Enable loading the standard map
+ `hoops`: Enable loading the hoops map
+ `dropshot`: Enable loading the dropshot map
+ `throwback`: Enable loading the throwback map (with standard game rules)
+ `compression`: Minimize the size of the produced binaries by compressing the binary field data at compile time. Will slightly slow down `load_x()` functions.
