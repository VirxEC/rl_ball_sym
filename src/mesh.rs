use crate::mat::{ Mat3, det };
use crate::math::{ dot };
use crate::geometry::Tri;
use crate::vector::Vec3;

#[derive(Debug)]
pub struct Mesh {
    pub ids: Vec<i32>,
    pub vertices: Vec<f64>
}

pub fn from_meshes(other_meshes: Vec<Mesh>) -> Mesh {
	let mut ids: Vec<i32> = Vec::new();
	let mut vertices: Vec<f64> = Vec::new();

	for m in other_meshes {
		// for id in m.ids {
		// 	ids.push(id);
		// }

		// for vertex in m.vertices {
		// 	vertices.push(vertex);
		// }
	}

    Mesh {
        ids,
        vertices
    }
}

impl Mesh {
    fn transform(&self, a: Mat3) -> Self {
        let mut ids: Vec<i32> = Vec::new();
        let mut vertices: Vec<f64> = Vec::new();

        for v in &self.vertices {
            // vertices.push(dot(&a, &v));
        }

        // for transformations that flip things
        // inside-out, change triangle winding
        if det(&a) < 0 as f64 {
            // for id in &self.ids {
            //     let v = Vec3 {
            //         x: id.y,
            //         y: id.x,
            //         z: id.z
            //     };
            //     ids.push(v);
            // }
        }

        Mesh {
            ids,
            vertices
        }
    }

    fn translate(&self, p: Vec3) -> Self {
        let ids: Vec<i32> = self.ids.clone();
        let mut vertices: Vec<f64> = Vec::new();

        for vertex in &self.vertices {
            // vertices.push(*vertex + p);
        }

        Self {
            ids,
            vertices
        }
    }

    fn to_triangles(&self) -> Vec<Tri> {
        let mut triangles: Vec<Tri> = Vec::new();

        // for (int i = 0; i < triangles.size(); i++) {
        //     for (int j = 0; j < 3; j++) {
        //         int id = ids[i * 3 + j];
        //         triangles[i].p[j] = vec3{ vertices[id * 3 + 0], vertices[id * 3 + 1], vertices[id * 3 + 2] };
        //     }
        // }

        triangles
    }
}