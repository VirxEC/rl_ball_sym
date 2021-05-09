#![allow(dead_code)]

use byteorder::{LittleEndian, ReadBytesExt};
use std::io::{Cursor, ErrorKind};
use std::time::Instant;

pub mod linear_algebra;
pub mod simulation;

use linear_algebra::vector::Vec3;
use simulation::ball::Ball;
use simulation::field::Field;
use simulation::game::Game;
use simulation::mesh::Mesh;

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

    let gravity = Vec3 {
        x: 0.,
        y: 0.,
        z: -650.,
    };

    println!("Loaded soccar game mode in {}ms", start.elapsed().as_millis());

    Game {
        index,
        team,
        gravity,
        field,
        ball,
    }
}
