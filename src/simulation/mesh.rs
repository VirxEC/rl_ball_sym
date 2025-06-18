//! Tools for creating space-efficient meshes and transforming them into a list of triangles.

use super::geometry::Tri;
use byteorder::{LittleEndian, ReadBytesExt};
use glam::{Mat3A, Vec3A};
use std::io::Cursor;

#[inline]
fn extract_usize(cursor: &mut Cursor<&[u8]>) -> usize {
    cursor
        .read_u32::<LittleEndian>()
        .unwrap_or_else(|e| unreachable!("Problem parsing ***_ids.dat: {e:?}")) as usize
}

#[inline]
fn extract_f32(cursor: &mut Cursor<&[u8]>) -> f32 {
    cursor
        .read_f32::<LittleEndian>()
        .unwrap_or_else(|e| unreachable!("Problem parsing ***_vertices.dat: {e:?}"))
}

/// A collection of inter-connected triangles.
#[derive(Clone, Debug, Default)]
pub struct Mesh {
    ids: Vec<usize>,
    vertices: Vec<Vec3A>,
}

impl Mesh {
    #[must_use]
    pub fn from_bytes(ids_dat: &[u8], vertices_dat: &[u8]) -> Self {
        let ids = {
            let ids_len = ids_dat.len() / 4;
            let mut ids_cursor = Cursor::new(ids_dat);

            (0..ids_len)
                .map(|_| extract_usize(&mut ids_cursor))
                .collect()
        };

        let vertices = {
            let vertices_len = vertices_dat.len() / 4;
            let mut vertices_cursor = Cursor::new(vertices_dat);

            (0..vertices_len / 3)
                .map(|_| {
                    Vec3A::new(
                        extract_f32(&mut vertices_cursor),
                        extract_f32(&mut vertices_cursor),
                        extract_f32(&mut vertices_cursor),
                    )
                })
                .collect()
        };

        Self { ids, vertices }
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
        let (n_ids, n_verts) = other_meshes.iter().fold((0, 0), |(n_ids, n_verts), m| {
            (n_ids + m.ids.len(), n_verts + m.vertices.len())
        });
        let mut id_offset = 0;

        let (ids, vertices) = other_meshes.into_iter().fold(
            (Vec::with_capacity(n_ids), Vec::with_capacity(n_verts)),
            |(mut ids, mut vertices), m| {
                ids.extend(m.ids.iter().map(|id| id + id_offset));
                id_offset += m.vertices.len();
                vertices.extend(m.vertices.iter());
                (ids, vertices)
            },
        );

        Self { ids, vertices }
    }

    #[must_use]
    /// Transform the mesh by the given matrix.
    pub fn transform(mut self, a: Mat3A) -> Self {
        debug_assert_eq!(self.ids.len() % 3, 0);

        for vertex in &mut *self.vertices {
            *vertex = a * *vertex;
        }

        // for transformations that flip things
        // inside-out, change triangle winding
        if a.determinant() < 0. {
            for ids in self.ids.chunks_exact_mut(3) {
                ids.swap(1, 2);
            }
        }

        self
    }

    #[must_use]
    #[inline]
    /// Translate the mesh on the z axis by the given value.
    pub fn translate_z(mut self, p: f32) -> Self {
        for vertex in &mut *self.vertices {
            vertex.z += p;
        }

        self
    }

    #[must_use]
    #[inline]
    /// Translate the mesh on the y axis by the given value.
    pub fn translate_y(mut self, p: f32) -> Self {
        for vertex in &mut *self.vertices {
            vertex.y += p;
        }

        self
    }

    #[must_use]
    /// Convert the mesh to a list of triangles.
    pub fn into_triangles(self) -> Box<[Tri]> {
        debug_assert_eq!(self.ids.len() % 3, 0);

        (0..self.ids.len() / 3)
            .map(|i| i * 3)
            .map(|i| Tri::from_points_iter(self.ids[i..i + 3].iter().map(|&j| self.vertices[j])))
            .collect()
    }
}
