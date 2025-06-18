use super::geometry::Aabb;
use std::ops::Add;

/// A branch in the BVH.
#[derive(Clone, Debug)]
pub struct Branch {
    /// The left child of this branch.
    pub left: usize,
    /// The right child of this branch.
    pub right: usize,
}

#[derive(Clone, Debug)]
pub enum NodeType {
    /// A leaf node at the end of a series of branches
    Leaf {
        /// The index of the primitive that this leaf represents.
        idx: usize,
    },
    /// A branch node that connects to more nodes
    Branch(Branch),
}

/// A node in the BVH.
#[derive(Clone, Debug)]
pub struct Node {
    /// The bounding box of this branch.
    pub aabb: Aabb,
    pub node_type: NodeType,
}

impl Node {
    #[must_use]
    #[inline]
    /// Creates a new leaf node for the BVH.
    pub const fn leaf(aabb: Aabb, idx: usize) -> Self {
        Self {
            aabb,
            node_type: NodeType::Leaf { idx },
        }
    }

    #[must_use]
    #[inline]
    /// Creates a new branch for the BVH given two children.
    pub const fn branch(aabb: Aabb, right: usize, left: usize) -> Self {
        Self {
            aabb,
            node_type: NodeType::Branch(Branch { left, right }),
        }
    }
}

#[inline]
/// Returns the global bounding box of a slice of bounding boxes.
pub fn global_aabb(boxes: &[Aabb]) -> Aabb {
    boxes.iter().skip(1).copied().fold(boxes[0], Add::add)
}

#[cfg(test)]
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
mod test {
    use super::*;
    use glam::Vec3A;

    const MIN_X: f32 = -4107.33;
    const MIN_Y: f32 = -6000.0;
    const MIN_Z: f32 = -13.2678;
    const MAX_X: f32 = 4107.33;
    const MAX_Y: f32 = 6000.0;
    const MAX_Z: f32 = 2075.45;

    #[test]
    fn global_bounding_box() {
        let bounding_boxes = vec![
            Aabb::new(Vec3A::new(MIN_X, 0.0, 0.0), Vec3A::ZERO),
            Aabb::new(Vec3A::new(0.0, MIN_Y, 0.0), Vec3A::ZERO),
            Aabb::new(Vec3A::new(0.0, 0.0, MIN_Z), Vec3A::ZERO),
            Aabb::new(Vec3A::ZERO, Vec3A::new(MAX_X, 0.0, 0.0)),
            Aabb::new(Vec3A::ZERO, Vec3A::new(0.0, MAX_Y, 0.0)),
            Aabb::new(Vec3A::ZERO, Vec3A::new(0.0, 0.0, MAX_Z)),
        ];

        let global = global_aabb(&bounding_boxes);

        assert!((global.min().x - MIN_X).abs() < f32::EPSILON);
        assert!((global.min().y - MIN_Y).abs() < f32::EPSILON);
        assert!((global.min().z - MIN_Z).abs() < f32::EPSILON);
        assert!((global.max().x - MAX_X).abs() < f32::EPSILON);
        assert!((global.max().y - MAX_Y).abs() < f32::EPSILON);
        assert!((global.max().z - MAX_Z).abs() < f32::EPSILON);
    }

    #[test]
    fn global_bounding_box_min() {
        let bounding_boxes = vec![
            Aabb::new(
                Vec3A::new(MIN_X, MIN_Y, MIN_Z),
                Vec3A::new(MIN_X, MIN_Y, MIN_Z),
            ),
            Aabb::new(
                Vec3A::new(MIN_X, MIN_Y, MIN_Z),
                Vec3A::new(MIN_X, MIN_Y, MIN_Z),
            ),
        ];
        let global = global_aabb(&bounding_boxes);

        assert!((global.min().x - MIN_X).abs() < f32::EPSILON);
        assert!((global.min().y - MIN_Y).abs() < f32::EPSILON);
        assert!((global.min().z - MIN_Z).abs() < f32::EPSILON);
        assert!((global.max().x - MIN_X).abs() < f32::EPSILON);
        assert!((global.max().y - MIN_Y).abs() < f32::EPSILON);
        assert!((global.max().z - MIN_Z).abs() < f32::EPSILON);
    }

    #[test]
    fn global_bounding_box_max() {
        let bounding_boxes = vec![
            Aabb::new(
                Vec3A::new(MAX_X, MAX_Y, MAX_Z),
                Vec3A::new(MAX_X, MAX_Y, MAX_Z),
            ),
            Aabb::new(
                Vec3A::new(MAX_X, MAX_Y, MAX_Z),
                Vec3A::new(MAX_X, MAX_Y, MAX_Z),
            ),
        ];
        let global = global_aabb(&bounding_boxes);

        assert!((global.min().x - MAX_X).abs() < f32::EPSILON);
        assert!((global.min().y - MAX_Y).abs() < f32::EPSILON);
        assert!((global.min().z - MAX_Z).abs() < f32::EPSILON);
        assert!((global.max().x - MAX_X).abs() < f32::EPSILON);
        assert!((global.max().y - MAX_Y).abs() < f32::EPSILON);
        assert!((global.max().z - MAX_Z).abs() < f32::EPSILON);
    }
}
