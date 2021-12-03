use super::geometry::Tri;
use crate::linear_algebra::mat::Mat3;
use crate::linear_algebra::math::dot;
use vvec3::Vec3;

#[derive(Clone, Debug)]
pub struct Mesh {
    pub ids: Vec<i32>,
    pub vertices: Vec<f32>,
}

impl Default for Mesh {
    fn default() -> Self {
        Self {
            ids: Vec::new(),
            vertices: Vec::new(),
        }
    }
}

impl Mesh {
    pub fn from(other_meshes: Vec<&Self>) -> Self {
        let mut id_offset = 0;

        let mut n_ids = 0;
        let mut n_vertices = 0;

        for m in &other_meshes {
            n_ids += m.ids.len();
            n_vertices += m.vertices.len();
        }

        let mut ids: Vec<i32> = Vec::with_capacity(n_ids);
        let mut vertices: Vec<f32> = Vec::with_capacity(n_vertices);

        for m in other_meshes {
            for id in &m.ids {
                ids.push(id + id_offset);
            }

            for vertex in &m.vertices {
                vertices.push(*vertex);
            }

            id_offset += (m.vertices.len() / 3) as i32;
        }

        // remove duplicate vertices and their ids
        // let mut i = 0;
        // let mut dups = 0;

        // loop {
        //     let id = (ids[i] * 3) as usize;
        //     match vertices[..id].iter().position(|&x| x == vertices[id + 0]) {
        //         Some(j) => {
        //             if vertices[j + 1] == vertices[id + 1] && vertices[j + 2] == vertices[id + 2] {
        //                 // dups += 1;
        //                 n_ids -= 3;

        //                 vertices.remove(id + 0);
        //                 vertices.remove(id + 1);
        //                 vertices.remove(id + 2);

        //                 ids.remove(i + 0);
        //                 ids.remove(i + 1);
        //                 ids.remove(i + 2);

        //                 for l in i..n_ids {
        //                     ids[l] -= 3;
        //                 }
        //             }
        //         }
        //         None => {}
        //     };

        //     i += 3;
        //     if i >= n_ids {
        //         break;
        //     }
        // }
        // println!("Removed {} duplicate vertices!", dups);

        Self {
            ids,
            vertices,
        }
    }

    #[rustfmt::skip]
    pub fn transform(&self, a: Mat3) -> Self {
        let mut ids: Vec<i32> = self.ids.clone();
        let mut vertices: Vec<f32> = self.vertices.clone();

        let n = self.vertices.len() / 3;

        for i in 0..n {
            let v = dot(a, Vec3::new(self.vertices[i * 3    ] as f32, self.vertices[i * 3 + 1] as f32, self.vertices[i * 3 + 2] as f32));

            vertices[i * 3    ] = v.x as f32;
            vertices[i * 3 + 1] = v.y as f32;
            vertices[i * 3 + 2] = v.z as f32;
        }

        // for transformations that flip things
        // inside-out, change triangle winding
        if a.det() < 0 as f32 {
            let n = ids.len() / 3;
            for i in 0..n {
                ids[i * 3    ] = self.ids[i * 3 + 1];
                ids[i * 3 + 1] = self.ids[i * 3    ];
                ids[i * 3 + 2] = self.ids[i * 3 + 2];
            }
        }

        Mesh {
            ids,
            vertices,
        }
    }

    #[rustfmt::skip]
    pub fn translate(&self, p: Vec3) -> Self {
        let ids: Vec<i32> = self.ids.clone();
        let mut vertices: Vec<f32> = self.vertices.clone();

        let n = vertices.len() / 3;
        for i in 0..n {
            vertices[i * 3    ] += p.x as f32;
            vertices[i * 3 + 1] += p.y as f32;
            vertices[i * 3 + 2] += p.z as f32;
        }

        Self {
            ids,
            vertices,
        }
    }

    #[rustfmt::skip]
    pub fn to_triangles(&self) -> Vec<Tri> {
        let n = self.ids.len() / 3;
        let mut triangles: Vec<Tri> = Vec::with_capacity(n);

        for i in 0..n {
            triangles.push(Tri::default());
            for j in 0..3 {
                let id = (self.ids[i * 3 + j] * 3) as usize;
                triangles[i].p[j].x = self.vertices[id    ] as f32;
                triangles[i].p[j].y = self.vertices[id + 1] as f32;
                triangles[i].p[j].z = self.vertices[id + 2] as f32;
            }
        }

        triangles
    }
}
