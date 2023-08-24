#![forbid(unsafe_code)]
#![warn(missing_docs, clippy::pedantic)]

//! `rl_ball_sym` is a Rust implementation of a simulation of the Rocket League ball inside it's field.
//! It loads the real geometry from the game and simulates the ball's movement in nanoseconds.
//!
//! ## Example: `ultra_basic`
//!
//! ```
//! use rl_ball_sym::{load_standard, Predictions, Vec3A};
//!
//! // load a standard standard match
//! let (game, mut ball) = load_standard();
//!
//! // the current state of the ball in the game
//! ball.update(0., Vec3A::new(0., 0., 200.), Vec3A::new(0., 0., -0.1), Vec3A::new(0., 0., 0.));
//!
//! // generate the ball prediction struct
//! // this is a list of 720 slices
//! // it goes 6 seconds into the future with 120 slices per second
//! let ball_prediction: Predictions = ball.get_ball_prediction_struct(&game);
//! assert_eq!(ball_prediction.len(), 720);
//!
//! // ball is not modified, it stays the same!
//! assert_eq!(ball.time, 0.);
//! ```

/// The `rl_ball_sym` crate uses the `glam` crate for SIMD-accelerated linear algebra.
pub extern crate glam;

#[cfg(feature = "compression")]
pub mod compressed;
mod linear_algebra;
pub mod simulation;
#[cfg(feature = "uncompressed")]
mod uncompressed;

pub use crate::simulation::{
    ball::{Ball, Predictions},
    game::Game,
};
pub use glam::Vec3A;
#[cfg(feature = "uncompressed")]
pub use uncompressed::*;
