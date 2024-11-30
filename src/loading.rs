use crate::{Ball, Game};

/// Returns a Game object with a standard field and standard ball.
#[inline]
#[must_use]
#[cfg(feature = "standard")]
pub fn load_standard() -> (Game, Ball) {
    (crate::standard_field(), Ball::DEFAULT_STANDARD)
}

/// Returns a Game object with a standard field and heatseeker ball.
#[inline]
#[must_use]
#[cfg(feature = "heatseeker")]
pub fn load_standard_heatseeker() -> (Game, Ball) {
    (crate::standard_field(), Ball::DEFAULT_HEATSEEKER)
}

/// Returns a Game object with a standard hoops field and hoops ball.
#[must_use]
#[cfg(feature = "hoops")]
pub fn load_hoops() -> (Game, Ball) {
    (crate::hoops_field(), Ball::DEFAULT_HOOPS)
}

/// Returns a Game object with a standard dropshot field and dropshot ball.
#[must_use]
#[cfg(feature = "dropshot")]
pub fn load_dropshot() -> (Game, Ball) {
    (crate::dropshot_field(), Ball::DEFAULT_DROPSHOT)
}

/// Returns a Game object with throwback stadium and a standard ball.
#[must_use]
#[cfg(feature = "throwback")]
pub fn load_standard_throwback() -> (Game, Ball) {
    (crate::throwback_stadium(), Ball::DEFAULT_STANDARD)
}
