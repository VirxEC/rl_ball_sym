#![allow(dead_code)]

use std::io::{Cursor, ErrorKind};
use byteorder::{LittleEndian, ReadBytesExt};

mod mesh;
mod mat;
mod math;
mod vector;
mod geometry;
mod field;

fn read_mesh(ids_dat: Vec<u8>, vertices_dat: Vec<u8>) -> mesh::Mesh {
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

    return mesh::Mesh {
        ids,
        vertices
    }
}

fn main() {
    let soccar_corner: mesh::Mesh = read_mesh(include_bytes!("../assets/soccar/soccar_corner_ids.bin").to_vec(), include_bytes!("../assets/soccar/soccar_corner_vertices.bin").to_vec());
    dbg!(soccar_corner.ids.len());
    dbg!(soccar_corner.vertices.len());
    let soccar_goal: mesh::Mesh = read_mesh(include_bytes!("../assets/soccar/soccar_goal_ids.bin").to_vec(), include_bytes!("../assets/soccar/soccar_goal_vertices.bin").to_vec());
    dbg!(soccar_goal.ids.len());
    dbg!(soccar_goal.vertices.len());
    let soccar_ramps_0: mesh::Mesh = read_mesh(include_bytes!("../assets/soccar/soccar_ramps_0_ids.bin").to_vec(), include_bytes!("../assets/soccar/soccar_ramps_0_vertices.bin").to_vec());
    dbg!(soccar_ramps_0.ids.len());
    dbg!(soccar_ramps_0.vertices.len());
    let soccar_ramps_1: mesh::Mesh = read_mesh(include_bytes!("../assets/soccar/soccar_ramps_1_ids.bin").to_vec(), include_bytes!("../assets/soccar/soccar_ramps_1_vertices.bin").to_vec());
    dbg!(soccar_ramps_1.ids.len());
    dbg!(soccar_ramps_1.vertices.len());

    let soccer_field = field::initialize_soccar(&soccar_corner, &soccar_goal, &soccar_ramps_0, &soccar_ramps_1);
    dbg!(soccer_field.ids.len());
    dbg!(soccer_field.vertices.len());

    println!("Loaded soccar mesh");
}
