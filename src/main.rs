#![allow(dead_code)]

use byteorder::{LittleEndian, ReadBytesExt};
use std::io::{Cursor, ErrorKind};
use std::time::Instant;

pub mod linear_algebra;
pub mod simulation;

use simulation::field::Field;
use simulation::game::Game;
use simulation::mesh::Mesh;
use simulation::ball::Ball;

fn read_mesh(ids_dat: Vec<u8>, vertices_dat: Vec<u8>) -> Mesh {
    let mut ids_dat = Cursor::new(ids_dat);
    let mut vertices_dat = Cursor::new(vertices_dat);
    let mut ids: Vec<i32> = Vec::new();
    let mut vertices: Vec<f32> = Vec::new();

    loop {
        ids.push(match ids_dat.read_i32::<LittleEndian>() {
            Ok(num) => num,
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

    return Mesh {
        ids,
        vertices,
    };
}

pub fn load_soccar(index: u8, team: u8) -> Game {
    let start = Instant::now();

    let soccar_corner: Mesh = read_mesh(include_bytes!("../assets/soccar/soccar_corner_ids.bin").to_vec(), include_bytes!("../assets/soccar/soccar_corner_vertices.bin").to_vec());
    let soccar_goal: Mesh = read_mesh(include_bytes!("../assets/soccar/soccar_goal_ids.bin").to_vec(), include_bytes!("../assets/soccar/soccar_goal_vertices.bin").to_vec());
    let soccar_ramps_0: Mesh = read_mesh(include_bytes!("../assets/soccar/soccar_ramps_0_ids.bin").to_vec(), include_bytes!("../assets/soccar/soccar_ramps_0_vertices.bin").to_vec());
    let soccar_ramps_1: Mesh = read_mesh(include_bytes!("../assets/soccar/soccar_ramps_1_ids.bin").to_vec(), include_bytes!("../assets/soccar/soccar_ramps_1_vertices.bin").to_vec());

    let field = Field::initialize_soccar(&soccar_corner, &soccar_goal, &soccar_ramps_0, &soccar_ramps_1);

    let ball = Ball::initialize_soccar();

    println!("Loaded soccar in {}ms", start.elapsed().as_millis());

    Game {
        index,
        team,
        field,
        ball
    }
}

fn main() {
    let game = load_soccar(0, 0);

    dbg!(&game.index);
    dbg!(&game.team);

    dbg!(&game.field.field_mesh.ids.len());
    dbg!(&game.field.field_mesh.vertices.len());

    dbg!(&game.field.collision_mesh.global);
    dbg!(&game.field.collision_mesh.mask);
    dbg!(&game.field.collision_mesh.num_leaves);
    dbg!(&game.field.collision_mesh.primitives.len());
    dbg!(&game.field.collision_mesh.code_ids.len());
    dbg!(&game.field.collision_mesh.siblings.len());
    dbg!(&game.field.collision_mesh.parents.len());
    dbg!(&game.field.collision_mesh.ready.len());
    dbg!(&game.field.collision_mesh.ranges.len());

    dbg!(&game.ball.location);
    dbg!(&game.ball.velocity);
    dbg!(&game.ball.radius);
    dbg!(&game.ball.collision_radius);
}
