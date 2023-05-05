//! Tools for creating space-efficient meshes and transforming them into a list of triangles.

use super::geometry::Tri;
use crate::linear_algebra::math::dot;
use byteorder::{LittleEndian, ReadBytesExt};
use glam::{Mat3A, Vec3A};
use std::io::{Cursor, Read};

#[inline]
fn extract_f32(cursor: &mut Cursor<&[u8]>) -> f32 {
    match cursor.read_f32::<LittleEndian>() {
        Ok(float) => float,
        Err(e) => panic!("Problem parsing ***_vertices.dat: {e:?}"),
    }
}

/// A collection of inter-connected triangles.
#[derive(Clone, Debug, Default)]
pub(crate) struct Mesh {
    ids: Vec<usize>,
    vertices: Vec<Vec3A>,
}

impl Mesh {
    #[must_use]
    pub fn from_bytes(ids_dat: &[u8], vertices_dat: &[u8]) -> Mesh {
        let ids = {
            let ids_len = ids_dat.len() / 4;
            let mut ids_cursor = Cursor::new(ids_dat);

            (0..ids_len)
                .map(|_| match ids_cursor.read_u32::<LittleEndian>() {
                    Ok(id) => id as usize,
                    Err(e) => panic!("Problem parsing ***_ids.dat: {e:?}"),
                })
                .collect::<Vec<_>>()
        };

        let vertices = {
            let vertices_len = vertices_dat.len() / 4;
            let mut vertices_cursor = Cursor::new(vertices_dat);

            (0..vertices_len / 3)
                .map(|_| {
                    Vec3A::new(
                        extract_f32(vertices_cursor.by_ref()),
                        extract_f32(vertices_cursor.by_ref()),
                        extract_f32(vertices_cursor.by_ref()),
                    )
                })
                .collect::<Vec<_>>()
        };

        Mesh { ids, vertices }
    }

    #[must_use]
    #[inline]
    /// Create a new Mesh from a list of ids and vertices.
    pub const fn new(ids: Vec<usize>, vertices: Vec<Vec3A>) -> Self {
        Self { ids, vertices }
    }

    #[must_use]
    /// Combine different meshes all into one
    pub fn combine<const N: usize>(other_meshes: [Self; N]) -> Self {
        let (n_ids, n_verts) = other_meshes.iter().fold((0, 0), |(n_ids, n_verts), m| (n_ids + m.ids.len(), n_verts + m.vertices.len()));
        let mut id_offset = 0;

        let (ids, vertices) = other_meshes
            .into_iter()
            .fold((Vec::with_capacity(n_ids), Vec::with_capacity(n_verts)), |(mut ids, mut vertices), m| {
                ids.extend(m.ids.into_iter().map(|id| id + id_offset));
                id_offset += m.vertices.len();
                vertices.extend(m.vertices);
                (ids, vertices)
            });

        Self { ids, vertices }
    }

    #[must_use]
    /// Transform the mesh by the given matrix.
    pub fn transform(&self, a: Mat3A) -> Self {
        debug_assert_eq!(self.ids.len() % 3, 0);

        let vertices = self.vertices.iter().map(|&vertex| dot(a, vertex)).collect::<Vec<_>>();

        // for transformations that flip things
        // inside-out, change triangle winding
        let ids = if a.determinant() < 0. {
            self.ids.chunks_exact(3).flat_map(|ids| [ids[1], ids[0], ids[2]]).collect()
        } else {
            self.ids.clone()
        };

        Self { ids, vertices }
    }

    #[must_use]
    #[inline]
    /// Translate the mesh by the given vector.
    pub fn translate(&self, p: Vec3A) -> Self {
        Self {
            ids: self.ids.clone(),
            vertices: self.vertices.iter().map(|&vertex| vertex + p).collect(),
        }
    }

    #[must_use]
    /// Convert the mesh to a list of triangles.
    pub fn into_triangles(self) -> Vec<Tri> {
        debug_assert_eq!(self.ids.len() % 3, 0);

        (0..self.ids.len() / 3)
            .map(|i| Tri::from_points_iter((0..3).map(|j| self.vertices[self.ids[i * 3 + j]])))
            .collect()
    }
}
