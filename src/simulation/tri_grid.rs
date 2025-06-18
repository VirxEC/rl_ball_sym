use crate::simulation::geometry::Tri;

use super::{
    game::Constraints,
    geometry::{Aabb, Contact, Sphere},
    tri_bvh::TriangleBvh,
};
use arrayvec::ArrayVec;
use glam::{USizeVec3, Vec3A};

#[derive(Clone, Debug)]
pub struct TriBvhGrid {
    pub(crate) aabb: Aabb,
    num_cells: USizeVec3,
    cells: Box<[Option<TriangleBvh>]>,
    pub(crate) primitives: Box<[Tri]>,
}

impl TriBvhGrid {
    const CELL_SIZE: f32 = 370.0;

    pub fn new(primitives: Box<[Tri]>, aabbs: &[Aabb], bvh: &TriangleBvh) -> Self {
        let global_aabb = bvh.nodes[bvh.nodes.len() - 1].aabb;
        let range = global_aabb.max() - global_aabb.min();
        let num_cells = (range / Self::CELL_SIZE)
            .ceil()
            .as_usizevec3()
            .max(USizeVec3::ONE);
        let mut cells = Vec::with_capacity(num_cells.element_product());

        for x in 0..num_cells.x {
            for y in 0..num_cells.y {
                for z in 0..num_cells.z {
                    let min_pos =
                        global_aabb.min() + USizeVec3::new(x, y, z).as_vec3a() * Self::CELL_SIZE;
                    let mut bounds = Aabb::new(min_pos, min_pos + Self::CELL_SIZE);

                    for x1 in 0..=2 {
                        if x + x1 == 0 || x + x1 > num_cells.x {
                            continue;
                        }

                        for y1 in 0..=2 {
                            if y + y1 == 0 || y + y1 > num_cells.y {
                                continue;
                            }

                            for z1 in 0..=2 {
                                if z + z1 == 0 || z + z1 > num_cells.z {
                                    continue;
                                }

                                let cell = USizeVec3::new(x + x1, y + y1, z + z1) - USizeVec3::ONE;
                                let min_pos = global_aabb.min() + cell.as_vec3a() * Self::CELL_SIZE;
                                bounds += Aabb::new(min_pos, min_pos + Self::CELL_SIZE);
                            }
                        }
                    }

                    cells.push(bvh.get_child_bvh(aabbs, bounds));
                }
            }
        }

        Self {
            num_cells,
            primitives,
            aabb: global_aabb,
            cells: cells.into_boxed_slice(),
        }
    }

    fn get_cell_indices(&self, pos: Vec3A) -> USizeVec3 {
        let cell_idx_f = (pos - self.aabb.min()) / Self::CELL_SIZE;
        cell_idx_f.as_usizevec3()
    }

    const fn cell_indices_to_index(&self, indices: USizeVec3) -> usize {
        indices.x * self.num_cells.y * self.num_cells.z + indices.y * self.num_cells.z + indices.z
    }

    fn get_cell_index(&self, pos: Vec3A) -> usize {
        self.cell_indices_to_index(self.get_cell_indices(pos))
    }

    #[must_use]
    /// Returns a Vec of the collision rays and triangle normals from the triangles intersecting with the `query_object`.
    pub fn collide(
        &self,
        query_object: Sphere,
    ) -> Option<ArrayVec<Contact, { Constraints::MAX_CONTACTS }>> {
        let cell_idx = self.get_cell_index(query_object.center);
        self.cells[cell_idx]
            .as_ref()
            .and_then(|bvh| bvh.collide(&self.primitives, query_object))
    }
}
