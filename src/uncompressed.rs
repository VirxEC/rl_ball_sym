use crate::simulation::{
    field::{initialize_dropshot, initialize_hoops, initialize_soccer, initialize_throwback, InitializeThrowbackParams},
    mesh::Mesh,
};
use crate::{Ball, Game};

macro_rules! include_mesh {
    ($ids:literal, $verts:literal) => {
        Mesh::from_bytes(include_bytes!($ids).as_slice(), include_bytes!($verts).as_slice())
    };
}

/// Returns a Game object with a standard soccer field and soccer ball.
#[must_use]
pub fn load_soccer() -> (Game, Ball) {
    let soccer_corner = include_mesh!("../assets/soccer/soccer_corner_ids.bin", "../assets/soccer/soccer_corner_vertices.bin");
    let soccer_goal = include_mesh!("../assets/soccer/soccer_goal_ids.bin", "../assets/soccer/soccer_goal_vertices.bin");
    let soccer_ramps_0 = include_mesh!("../assets/soccer/soccer_ramps_0_ids.bin", "../assets/soccer/soccer_ramps_0_vertices.bin");
    let soccer_ramps_1 = include_mesh!("../assets/soccer/soccer_ramps_1_ids.bin", "../assets/soccer/soccer_ramps_1_vertices.bin");

    let collision_mesh = initialize_soccer(&soccer_corner, &soccer_goal, &soccer_ramps_0, &soccer_ramps_1);

    (Game::new(collision_mesh), Ball::initialize_soccar())
}

/// Returns a Game object with a standard soccar field and soccar ball.
#[must_use]
#[inline]
pub fn load_soccar() -> (Game, Ball) {
    load_soccer()
}

/// Returns a Game object with a standard hoops field and hoops ball.
#[must_use]
pub fn load_hoops() -> (Game, Ball) {
    let hoops_corner = include_mesh!("../assets/hoops/hoops_corner_ids.bin", "../assets/hoops/hoops_corner_vertices.bin");
    let hoops_net = include_mesh!("../assets/hoops/hoops_net_ids.bin", "../assets/hoops/hoops_net_vertices.bin");
    let hoops_rim = include_mesh!("../assets/hoops/hoops_rim_ids.bin", "../assets/hoops/hoops_rim_vertices.bin");
    let hoops_ramps_0 = include_mesh!("../assets/hoops/hoops_ramps_0_ids.bin", "../assets/hoops/hoops_ramps_0_vertices.bin");
    let hoops_ramps_1 = include_mesh!("../assets/hoops/hoops_ramps_1_ids.bin", "../assets/hoops/hoops_ramps_1_vertices.bin");

    let collision_mesh = initialize_hoops(&hoops_corner, &hoops_net, &hoops_rim, &hoops_ramps_0, &hoops_ramps_1);

    (Game::new(collision_mesh), Ball::initialize_hoops())
}

/// Returns a Game object with a standard dropshot field and dropshot ball.
#[must_use]
pub fn load_dropshot() -> (Game, Ball) {
    let dropshot = include_mesh!("../assets/dropshot/dropshot_ids.bin", "../assets/dropshot/dropshot_vertices.bin");

    let collision_mesh = initialize_dropshot(&dropshot);

    (Game::new(collision_mesh), Ball::initialize_dropshot())
}

/// Returns a Game object with throwback stadium and a standard soccer ball.
#[must_use]
pub fn load_soccer_throwback() -> (Game, Ball) {
    let back_ramps_lower = include_mesh!(
        "../assets/throwback/throwback_back_ramps_lower_ids.bin",
        "../assets/throwback/throwback_back_ramps_lower_vertices.bin"
    );
    let back_ramps_upper = include_mesh!(
        "../assets/throwback/throwback_back_ramps_upper_ids.bin",
        "../assets/throwback/throwback_back_ramps_upper_vertices.bin"
    );
    let corner_ramps_lower = include_mesh!(
        "../assets/throwback/throwback_corner_ramps_lower_ids.bin",
        "../assets/throwback/throwback_corner_ramps_lower_vertices.bin"
    );
    let corner_ramps_upper = include_mesh!(
        "../assets/throwback/throwback_corner_ramps_upper_ids.bin",
        "../assets/throwback/throwback_corner_ramps_upper_vertices.bin"
    );
    let corner_wall_0 = include_mesh!(
        "../assets/throwback/throwback_corner_wall_0_ids.bin",
        "../assets/throwback/throwback_corner_wall_0_vertices.bin"
    );
    let corner_wall_1 = include_mesh!(
        "../assets/throwback/throwback_corner_wall_1_ids.bin",
        "../assets/throwback/throwback_corner_wall_1_vertices.bin"
    );
    let corner_wall_2 = include_mesh!(
        "../assets/throwback/throwback_corner_wall_2_ids.bin",
        "../assets/throwback/throwback_corner_wall_2_vertices.bin"
    );
    let goal = include_mesh!("../assets/throwback/throwback_goal_ids.bin", "../assets/throwback/throwback_goal_vertices.bin");
    let side_ramps_lower = include_mesh!(
        "../assets/throwback/throwback_side_ramps_lower_ids.bin",
        "../assets/throwback/throwback_side_ramps_lower_vertices.bin"
    );
    let side_ramps_upper = include_mesh!(
        "../assets/throwback/throwback_side_ramps_upper_ids.bin",
        "../assets/throwback/throwback_side_ramps_upper_vertices.bin"
    );

    let params = InitializeThrowbackParams {
        back_ramps_lower: &back_ramps_lower,
        back_ramps_upper: &back_ramps_upper,
        corner_ramps_lower: &corner_ramps_lower,
        corner_ramps_upper: &corner_ramps_upper,
        corner_wall_0: &corner_wall_0,
        corner_wall_1: &corner_wall_1,
        corner_wall_2: &corner_wall_2,
        goal: &goal,
        side_ramps_lower: &side_ramps_lower,
        side_ramps_upper: &side_ramps_upper,
    };
    let collision_mesh = initialize_throwback(params);

    (Game::new(collision_mesh), Ball::initialize_soccar())
}

/// Returns a Game object with throwback stadium and a standard soccar ball.
#[must_use]
#[inline]
pub fn load_soccar_throwback() -> (Game, Ball) {
    load_soccer_throwback()
}