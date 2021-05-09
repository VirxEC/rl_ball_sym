use crate::geometry::Tri;
use crate::mat::{det, Mat3};
use crate::math::dot;
use crate::vector::Vec3;

#[derive(Debug)]
pub struct Mesh {
    pub ids: Vec<i32>,
    pub vertices: Vec<f32>,
}

impl Mesh {
    pub fn from(other_meshes: Vec<&Self>) -> Self {
        let mut id_offset = 0;

        let mut nids = 0;
        let mut nvertices = 0;

        for m in &other_meshes {
            nids += m.ids.len();
            nvertices += m.vertices.len();
        }

        let mut ids: Vec<i32> = Vec::with_capacity(nids);
        let mut vertices: Vec<f32> = Vec::with_capacity(nvertices);

        for m in other_meshes {
            for id in &m.ids {
                ids.push(id + id_offset);
            }

            for vertex in &m.vertices {
                vertices.push(*vertex);
            }

            id_offset += (m.vertices.len() / 3) as i32;
        }

        Self {
            ids,
            vertices,
        }
    }

    pub fn transform(&self, a: &Mat3) -> Self {
        let mut ids: Vec<i32> = self.ids.clone();
        let mut vertices: Vec<f32> = self.vertices.clone();

        let n = self.vertices.len() / 3;

        for i in 0..n {
            let v = dot(
                &a,
                &Vec3 {
                    x: self.vertices[i * 3 + 0] as f64,
                    y: self.vertices[i * 3 + 1] as f64,
                    z: self.vertices[i * 3 + 2] as f64,
                },
            );

            vertices[i * 3 + 0] = v.x as f32;
            vertices[i * 3 + 1] = v.y as f32;
            vertices[i * 3 + 2] = v.z as f32;
        }

        // for transformations that flip things
        // inside-out, change triangle winding
        if det(&a) < 0 as f64 {
            let n = ids.len() / 3;
            for i in 0..n {
                ids[i * 3 + 0] = self.ids[i * 3 + 1];
                ids[i * 3 + 1] = self.ids[i * 3 + 0];
                ids[i * 3 + 2] = self.ids[i * 3 + 2];
            }
        }

        Mesh {
            ids,
            vertices,
        }
    }

    pub fn translate(&self, p: &Vec3) -> Self {
        let ids: Vec<i32> = self.ids.clone();
        let mut vertices: Vec<f32> = self.vertices.clone();

        let n = vertices.len() / 3;
        for i in 0..n {
            vertices[i * 3 + 0] += p.x as f32;
            vertices[i * 3 + 1] += p.y as f32;
            vertices[i * 3 + 2] += p.z as f32;
        }

        Self {
            ids,
            vertices,
        }
    }

    pub fn to_triangles(&self) -> Vec<Tri> {
        let n = self.ids.len() / 3;
        let mut triangles: Vec<Tri> = Vec::with_capacity(n);

        for i in 0..n {
            triangles.push(Tri::default());
            for j in 0..3 {
                let id = self.ids[i * 3 + j] as usize;
                triangles[i].p[j] = Vec3 {
                    x: self.vertices[id * 3 + 0] as f64,
                    y: self.vertices[id * 3 + 1] as f64,
                    z: self.vertices[id * 3 + 2] as f64,
                };
            }
        }

        triangles
    }
}
