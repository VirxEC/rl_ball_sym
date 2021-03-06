use super::geometry::Tri;
use crate::linear_algebra::math::dot;
use glam::{Mat3A, Vec3A};

/// A collection of inter-connected triangles.
#[derive(Clone, Debug, Default)]
pub struct Mesh {
    ids: Vec<usize>,
    vertices: Vec<f32>,
}

impl Mesh {
    /// Create a new Mesh from a list of ids and vertices.
    #[must_use]
    pub const fn from(ids: Vec<usize>, vertices: Vec<f32>) -> Self {
        Self { ids, vertices }
    }

    /// Combine different meshes all into one
    #[must_use]
    pub fn combine(other_meshes: Vec<&Self>) -> Self {
        let mut id_offset = 0;

        let n_ids = other_meshes.iter().map(|mesh| mesh.ids.len()).sum();
        let n_vertices = other_meshes.iter().map(|mesh| mesh.vertices.len()).sum();

        let mut ids: Vec<usize> = Vec::with_capacity(n_ids);
        let mut vertices: Vec<f32> = Vec::with_capacity(n_vertices);

        for m in other_meshes {
            for id in &m.ids {
                ids.push(id + id_offset);
            }

            for vertex in &m.vertices {
                vertices.push(*vertex);
            }

            id_offset += m.vertices.len() / 3;
        }

        Self { ids, vertices }
    }

    /// Transform the mesh by the given matrix.
    #[must_use]
    pub fn transform(&self, a: Mat3A) -> Self {
        debug_assert_eq!(self.vertices.len() % 3, 0);
        debug_assert_eq!(self.ids.len() % 3, 0);

        let vertices = self
            .vertices
            .chunks(3)
            .flat_map(|vertex| {
                let v = dot(a, Vec3A::from_slice(vertex));
                v.to_array()
            })
            .collect();

        // for transformations that flip things
        // inside-out, change triangle winding
        let ids = if a.determinant() < 0. {
            self.ids.chunks(3).flat_map(|ids| [ids[1], ids[0], ids[2]]).collect()
        } else {
            self.ids.clone()
        };

        Self { ids, vertices }
    }

    /// Translate the mesh by the given vector.
    #[must_use]
    pub fn translate(&self, p: Vec3A) -> Self {
        debug_assert_eq!(self.vertices.len() % 3, 0);

        let vertices = self.vertices.chunks(3).flat_map(|vertex| (Vec3A::from_slice(vertex) + p).to_array()).collect();

        Self { ids: self.ids.clone(), vertices }
    }

    /// Convert the mesh to a list of triangles.
    #[rustfmt::skip]
    #[must_use]
    pub fn to_triangles(&self) -> Vec<Tri> {
        let n = self.ids.len() / 3;
        let mut triangles: Vec<Tri> = Vec::with_capacity(n);

        for i in 0..n {
            triangles.push(Tri::default());
            for j in 0..3 {
                let id = self.ids[i * 3 + j] * 3;
                triangles[i].p[j].x = self.vertices[id    ] as f32;
                triangles[i].p[j].y = self.vertices[id + 1] as f32;
                triangles[i].p[j].z = self.vertices[id + 2] as f32;
            }
        }

        triangles
    }
}
