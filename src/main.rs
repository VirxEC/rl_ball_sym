#![allow(dead_code)]

use byteorder::{LittleEndian, ReadBytesExt};
use std::io::{Cursor, ErrorKind};
use std::time::Instant;

pub mod linear_algebra;
pub mod simulation;

use simulation::field::initialize_soccar;
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

fn main() {
    let start = Instant::now();

    let soccar_corner: Mesh = read_mesh(include_bytes!("../assets/soccar/soccar_corner_ids.bin").to_vec(), include_bytes!("../assets/soccar/soccar_corner_vertices.bin").to_vec());
    let soccar_goal: Mesh = read_mesh(include_bytes!("../assets/soccar/soccar_goal_ids.bin").to_vec(), include_bytes!("../assets/soccar/soccar_goal_vertices.bin").to_vec());
    let soccar_ramps_0: Mesh = read_mesh(include_bytes!("../assets/soccar/soccar_ramps_0_ids.bin").to_vec(), include_bytes!("../assets/soccar/soccar_ramps_0_vertices.bin").to_vec());
    let soccar_ramps_1: Mesh = read_mesh(include_bytes!("../assets/soccar/soccar_ramps_1_ids.bin").to_vec(), include_bytes!("../assets/soccar/soccar_ramps_1_vertices.bin").to_vec());

    let soccar_field = initialize_soccar(&soccar_corner, &soccar_goal, &soccar_ramps_0, &soccar_ramps_1);

    dbg!(soccar_field.0.ids.len());
    dbg!(soccar_field.0.vertices.len());

    println!("Loaded soccar mesh in {}ms", start.elapsed().as_millis());

    dbg!(soccar_field.1.global);
    dbg!(soccar_field.1.mask);
    dbg!(soccar_field.1.num_leaves);
    dbg!(soccar_field.1.primitives.len());
    dbg!(soccar_field.1.code_ids.len());
    dbg!(soccar_field.1.siblings.len());
    dbg!(soccar_field.1.parents.len());
    dbg!(soccar_field.1.ready.len());
    dbg!(soccar_field.1.ranges.len());
}
