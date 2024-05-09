use super::geometry::Aabb;
use std::ops::Add;

/// A leaf in the BVH.
#[derive(Clone, Copy, Debug)]
pub struct Leaf {
    /// The bounding box of this leaf.
    pub aabb: Aabb,
    /// The primitive that this leaf represents.
    pub idx: usize,
}

impl Leaf {
    #[must_use]
    #[inline]
    /// Create a new leaf.
    pub const fn new(aabb: Aabb, idx: usize) -> Self {
        Self { aabb, idx }
    }
}

/// A branch in the BVH.
#[derive(Clone, Debug)]
pub struct Branch {
    /// The bounding box of this branch.
    pub aabb: Aabb,
    /// The left child of this branch.
    pub left: Box<Node>,
    /// The right child of this branch.
    pub right: Box<Node>,
}

impl Branch {
    #[must_use]
    #[inline]
    /// Create a new branch.
    pub const fn new(aabb: Aabb, left: Box<Node>, right: Box<Node>) -> Self {
        Self { aabb, left, right }
    }
}

/// A node in the BVH.
#[derive(Clone, Debug)]
pub enum Node {
    /// A leaf node at the end of a series of branches
    Leaf(Leaf),
    /// A branch node that connects to more nodes
    Branch(Branch),
}

impl Node {
    #[must_use]
    #[inline]
    /// Creates a new branch for the BVH given two children.
    pub fn branch(right: Self, left: Self) -> Self {
        Self::Branch(Branch::new(right.aabb() + left.aabb(), Box::new(left), Box::new(right)))
    }

    #[must_use]
    #[inline]
    /// Returns the bounding box of this node.
    pub const fn aabb(&self) -> Aabb {
        match self {
            Self::Leaf(leaf) => leaf.aabb,
            Self::Branch(branch) => branch.aabb,
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
            Aabb::new(Vec3A::new(MIN_X, MIN_Y, MIN_Z), Vec3A::new(MIN_X, MIN_Y, MIN_Z)),
            Aabb::new(Vec3A::new(MIN_X, MIN_Y, MIN_Z), Vec3A::new(MIN_X, MIN_Y, MIN_Z)),
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
            Aabb::new(Vec3A::new(MAX_X, MAX_Y, MAX_Z), Vec3A::new(MAX_X, MAX_Y, MAX_Z)),
            Aabb::new(Vec3A::new(MAX_X, MAX_Y, MAX_Z), Vec3A::new(MAX_X, MAX_Y, MAX_Z)),
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
