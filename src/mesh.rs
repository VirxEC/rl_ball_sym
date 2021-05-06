use crate::mat::{ Mat3, det };
use crate::math::{ dot };
use crate::vector::Vec3;

#[derive(Debug)]
pub struct Mesh {
    pub ids: Vec<Vec3>,
    pub vertices: Vec<Vec3>
}

pub fn from_meshes(other_meshes: Vec<Mesh>) -> Mesh {
	let mut ids: Vec<Vec3> = Vec::new();
	let mut vertices: Vec<Vec3> = Vec::new();

	for m in other_meshes {
		for id in m.ids {
			ids.push(id);
		}

		for vertex in m.vertices {
			vertices.push(vertex);
		}
	}

    Mesh {
        ids,
        vertices
    }
}

impl Mesh {
    fn transform(&self, a: Mat3) -> Self {
        let mut ids: Vec<Vec3> = Vec::new();
        let mut vertices: Vec<Vec3> = Vec::new();

        for v in &self.vertices {
            vertices.push(dot(&a, &v));
        }

        // for transformations that flip things 
        // inside-out, change triangle winding
        if det(&a) < 0 as f64 {
            for id in &self.ids {
                let v = Vec3 {
                    x: id.y,
                    y: id.x,
                    z: id.z
                };
                ids.push(v);
            }
        }

        Mesh {
            ids,
            vertices
        }
    }

    fn translate(&self, p: Vec3) -> Self {
        let ids: Vec<Vec3> = self.ids.clone();
        let mut vertices: Vec<Vec3> = Vec::new();

        for vertex in &self.vertices {
            vertices.push(*vertex + p);
        }

        Self {
            ids,
            vertices
        }
    }
}