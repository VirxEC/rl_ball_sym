pub mod linear_algebra;
pub mod simulation;

use byteorder::{LittleEndian, ReadBytesExt};
use glam::Vec3A;
use simulation::ball::Ball;
use simulation::field::{initialize_dropshot, initialize_hoops, initialize_soccar, initialize_throwback};
use simulation::game::Game;
use simulation::mesh::Mesh;
use std::io::{Cursor, ErrorKind};

use crate::simulation::field::InitializeThrowbackParams;

fn read_mesh(ids_dat: Vec<u8>, vertices_dat: Vec<u8>) -> Mesh {
    let ids_len = ids_dat.len() / 4;
    let vertices_len = vertices_dat.len() / 4;
    let mut ids_dat = Cursor::new(ids_dat);
    let mut vertices_dat = Cursor::new(vertices_dat);
    let mut ids: Vec<usize> = Vec::with_capacity(ids_len);
    let mut vertices: Vec<f32> = Vec::with_capacity(vertices_len);

    loop {
        ids.push(match ids_dat.read_i32::<LittleEndian>() {
            Ok(num) => num as usize,
            Err(error) => match error.kind() {
                ErrorKind::UnexpectedEof => break,
                other_error => {
                    panic!("Problem parsing file: {:?}", other_error)
                }
            },
        });
    }

    loop {
        vertices.push(match vertices_dat.read_f32::<LittleEndian>() {
            Ok(num) => num,
            Err(error) => match error.kind() {
                ErrorKind::UnexpectedEof => break,
                other_error => {
                    panic!("Problem parsing file: {:?}", other_error)
                }
            },
        });
    }

    Mesh::from(ids, vertices)
}

/// Returns a Game object with a standard soccar field and soccar ball.
#[must_use]
pub fn load_soccar() -> (Game, Ball) {
    let soccar_corner: Mesh = read_mesh(
        include_bytes!("../assets/soccar/soccar_corner_ids.bin").to_vec(),
        include_bytes!("../assets/soccar/soccar_corner_vertices.bin").to_vec(),
    );
    let soccar_goal: Mesh = read_mesh(
        include_bytes!("../assets/soccar/soccar_goal_ids.bin").to_vec(),
        include_bytes!("../assets/soccar/soccar_goal_vertices.bin").to_vec(),
    );
    let soccar_ramps_0: Mesh = read_mesh(
        include_bytes!("../assets/soccar/soccar_ramps_0_ids.bin").to_vec(),
        include_bytes!("../assets/soccar/soccar_ramps_0_vertices.bin").to_vec(),
    );
    let soccar_ramps_1: Mesh = read_mesh(
        include_bytes!("../assets/soccar/soccar_ramps_1_ids.bin").to_vec(),
        include_bytes!("../assets/soccar/soccar_ramps_1_vertices.bin").to_vec(),
    );

    let collision_mesh = initialize_soccar(&soccar_corner, &soccar_goal, &soccar_ramps_0, &soccar_ramps_1);

    let gravity = Vec3A::new(0., 0., -650.);

    (Game { gravity, collision_mesh }, Ball::initialize_soccar())
}

/// Returns a Game object with a standard hoops field and hoops ball.
#[must_use]
pub fn load_hoops() -> (Game, Ball) {
    let hoops_corner: Mesh = read_mesh(
        include_bytes!("../assets/hoops/hoops_corner_ids.bin").to_vec(),
        include_bytes!("../assets/hoops/hoops_corner_vertices.bin").to_vec(),
    );
    let hoops_net: Mesh = read_mesh(
        include_bytes!("../assets/hoops/hoops_net_ids.bin").to_vec(),
        include_bytes!("../assets/hoops/hoops_net_vertices.bin").to_vec(),
    );
    let hoops_rim: Mesh = read_mesh(
        include_bytes!("../assets/hoops/hoops_rim_ids.bin").to_vec(),
        include_bytes!("../assets/hoops/hoops_rim_vertices.bin").to_vec(),
    );
    let hoops_ramps_0: Mesh = read_mesh(
        include_bytes!("../assets/hoops/hoops_ramps_0_ids.bin").to_vec(),
        include_bytes!("../assets/hoops/hoops_ramps_0_vertices.bin").to_vec(),
    );
    let hoops_ramps_1: Mesh = read_mesh(
        include_bytes!("../assets/hoops/hoops_ramps_1_ids.bin").to_vec(),
        include_bytes!("../assets/hoops/hoops_ramps_1_vertices.bin").to_vec(),
    );

    let collision_mesh = initialize_hoops(&hoops_corner, &hoops_net, &hoops_rim, &hoops_ramps_0, &hoops_ramps_1);

    let gravity = Vec3A::new(0., 0., -650.);

    (Game { gravity, collision_mesh }, Ball::initialize_hoops())
}

/// Returns a Game object with a standard dropshot field and dropshot ball.
#[must_use]
pub fn load_dropshot() -> (Game, Ball) {
    let dropshot: Mesh = read_mesh(
        include_bytes!("../assets/dropshot/dropshot_ids.bin").to_vec(),
        include_bytes!("../assets/dropshot/dropshot_vertices.bin").to_vec(),
    );

    let collision_mesh = initialize_dropshot(&dropshot);

    let gravity = Vec3A::new(0., 0., -650.);

    (Game { gravity, collision_mesh }, Ball::initialize_dropshot())
}

/// Returns a Game object with throwback stadium and a standard soccar ball.
#[must_use]
pub fn load_soccar_throwback() -> (Game, Ball) {
    let back_ramps_lower: Mesh = read_mesh(
        include_bytes!("../assets/throwback/throwback_back_ramps_lower_ids.bin").to_vec(),
        include_bytes!("../assets/throwback/throwback_back_ramps_lower_vertices.bin").to_vec(),
    );
    let back_ramps_upper: Mesh = read_mesh(
        include_bytes!("../assets/throwback/throwback_back_ramps_upper_ids.bin").to_vec(),
        include_bytes!("../assets/throwback/throwback_back_ramps_upper_vertices.bin").to_vec(),
    );
    let corner_ramps_lower: Mesh = read_mesh(
        include_bytes!("../assets/throwback/throwback_corner_ramps_lower_ids.bin").to_vec(),
        include_bytes!("../assets/throwback/throwback_corner_ramps_lower_vertices.bin").to_vec(),
    );
    let corner_ramps_upper: Mesh = read_mesh(
        include_bytes!("../assets/throwback/throwback_corner_ramps_upper_ids.bin").to_vec(),
        include_bytes!("../assets/throwback/throwback_corner_ramps_upper_vertices.bin").to_vec(),
    );
    let corner_wall_0: Mesh = read_mesh(
        include_bytes!("../assets/throwback/throwback_corner_wall_0_ids.bin").to_vec(),
        include_bytes!("../assets/throwback/throwback_corner_wall_0_vertices.bin").to_vec(),
    );
    let corner_wall_1: Mesh = read_mesh(
        include_bytes!("../assets/throwback/throwback_corner_wall_1_ids.bin").to_vec(),
        include_bytes!("../assets/throwback/throwback_corner_wall_1_vertices.bin").to_vec(),
    );
    let corner_wall_2: Mesh = read_mesh(
        include_bytes!("../assets/throwback/throwback_corner_wall_2_ids.bin").to_vec(),
        include_bytes!("../assets/throwback/throwback_corner_wall_2_vertices.bin").to_vec(),
    );
    let goal: Mesh = read_mesh(
        include_bytes!("../assets/throwback/throwback_goal_ids.bin").to_vec(),
        include_bytes!("../assets/throwback/throwback_goal_vertices.bin").to_vec(),
    );
    let side_ramps_lower: Mesh = read_mesh(
        include_bytes!("../assets/throwback/throwback_side_ramps_lower_ids.bin").to_vec(),
        include_bytes!("../assets/throwback/throwback_side_ramps_lower_vertices.bin").to_vec(),
    );
    let side_ramps_upper: Mesh = read_mesh(
        include_bytes!("../assets/throwback/throwback_side_ramps_upper_ids.bin").to_vec(),
        include_bytes!("../assets/throwback/throwback_side_ramps_upper_vertices.bin").to_vec(),
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

    let gravity = Vec3A::new(0., 0., -650.);

    (Game { gravity, collision_mesh }, Ball::initialize_soccar())
}
