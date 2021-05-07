#![allow(dead_code)]

use std::fs::File;
use std::io::Read;

mod mesh;
mod mat;
mod math;
mod vector;
mod geometry;
mod field;

// pub static soccar_corner: mesh::Mesh = mesh::Mesh {
//     ids: include!("../assets/soccar/soccar_corner_ids.bin"),
//     vertices: include!("../assets/soccar/soccar_corner_vertices.bin")
// };
// pub static soccar_goal: mesh::Mesh;
// pub static soccar_ramps_0: mesh::Mesh;
// pub static soccar_ramps_1: mesh::Mesh;

fn main() {
    let mut ids: Vec<i32> = Vec::new();

    let mut file = match File::open("assets/soccar/soccar_corner_ids.bin") {
        Ok(file) => file,
        Err(error) => panic!("Problem opening the file: {:?}", error)
    };

    let mut chunks = Vec::new();

    let n = match file.read_to_end(&mut chunks) {
        Ok(number) => number,
        Err(error) => panic!("Problem reading the file: {:?}", error)
    };

    println!("{}", chunks.len());

    let mut num = String::from("");

    for i in 0..chunks.len() {
        num.push_str(&String::from(format!("{:b}", &chunks[i])));

        if num.len() == 32 {
            println!("{} | {} bits", num, num.len());
            ids.push(match i32::from_str_radix(&num, 2) {
                Ok(num) => num,
                Err(error) => panic!("Problem converting binary to i32: {:?}", error)
            });
            num = String::from("");
        }
    }

    // let mut ids = soccar_corner.ids.clone();
    ids.sort();

    for id in ids.iter() {
        println!("{}", id);
    }
}
