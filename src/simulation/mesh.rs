//! Tools for creating space-efficient meshes and transforming them into a list of triangles.

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
    #[must_use]
    #[inline]
    /// Create a new Mesh from a list of ids and vertices.
    pub const fn from(ids: Vec<usize>, vertices: Vec<f32>) -> Self {
        Self { ids, vertices }
    }

    #[must_use]
    /// Combine different meshes all into one
    pub fn combine(other_meshes: &[&Self]) -> Self {
        let mut id_offset = 0;

        let n_ids = other_meshes.iter().map(|mesh| mesh.ids.len()).sum();
        let mut ids: Vec<usize> = Vec::with_capacity(n_ids);

        for m in other_meshes {
            ids.extend(m.ids.iter().map(|id| id + id_offset));
            id_offset += m.vertices.len() / 3;
        }

        let vertices: Vec<f32> = other_meshes.iter().flat_map(|m| &m.vertices).cloned().collect();

        Self { ids, vertices }
    }

    #[must_use]
    /// Transform the mesh by the given matrix.
    pub fn transform(&self, a: Mat3A) -> Self {
        debug_assert_eq!(self.vertices.len() % 3, 0);
        debug_assert_eq!(self.ids.len() % 3, 0);

        let vertices = self.vertices.chunks(3).flat_map(|vertex| dot(a, Vec3A::from_slice(vertex)).to_array()).collect();

        // for transformations that flip things
        // inside-out, change triangle winding
        let ids = if a.determinant() < 0. {
            self.ids.chunks(3).flat_map(|ids| [ids[1], ids[0], ids[2]]).collect()
        } else {
            self.ids.clone()
        };

        Self { ids, vertices }
    }

    #[must_use]
    /// Translate the mesh by the given vector.
    pub fn translate(&self, p: Vec3A) -> Self {
        debug_assert_eq!(self.vertices.len() % 3, 0);

        let vertices = self.vertices.chunks(3).flat_map(|vertex| (Vec3A::from_slice(vertex) + p).to_array()).collect();

        Self { ids: self.ids.clone(), vertices }
    }

    #[must_use]
    /// Convert the mesh to a list of triangles.
    pub fn to_triangles(&self) -> Vec<Tri> {
        debug_assert_eq!(self.ids.len() % 3, 0);

        (0..self.ids.len() / 3).map(|i| self.triangle_from_index(i)).collect()
    }

    fn triangle_from_index(&self, i: usize) -> Tri {
        let mut points = (0..3).map(|j| {
            let id = self.ids[i * 3 + j] * 3;
            Vec3A::new(self.vertices[id], self.vertices[id + 1], self.vertices[id + 2])
        });
        Tri::from_points(points.next().unwrap(), points.next().unwrap(), points.next().unwrap())
    }
}
