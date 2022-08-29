use super::{
    geometry::{Aabb, Ray, Sphere, Tri},
    morton::Morton,
};
use std::boxed::Box;

/// A leaf in the BVH.
#[derive(Clone, Copy, Debug)]
pub struct Leaf {
    /// The bounding box of this leaf.
    pub box_: Aabb,
    /// The primitive that this leaf represents.
    pub primitive: Tri,
    /// The morton code of this leaf.
    pub morton: u64,
}

impl Leaf {
    /// Create a new leaf.
    #[must_use]
    #[inline]
    pub const fn new(primitive: Tri, box_: Aabb, morton: u64) -> Self {
        Self { box_, primitive, morton }
    }
}

// A branch in the BVH.
#[derive(Clone, Debug)]
pub struct Branch {
    /// The bounding box of this branch.
    pub box_: Aabb,
    /// The left child of this branch.
    pub left: Box<BvhNode>,
    /// The right child of this branch.
    pub right: Box<BvhNode>,
}

impl Branch {
    /// Create a new branch.
    #[must_use]
    #[inline]
    pub const fn new(box_: Aabb, left: Box<BvhNode>, right: Box<BvhNode>) -> Self {
        Self { box_, left, right }
    }
}

/// A node in the BVH.
#[derive(Clone, Debug)]
pub enum BvhNode {
    Leaf(Leaf),
    Branch(Branch),
}

impl BvhNode {
    /// Creates a new branch for the BVH given two children.
    #[must_use]
    pub fn branch(right: Self, left: Self) -> Self {
        Self::Branch(Branch::new(right.box_().add(&left.box_()), Box::new(left), Box::new(right)))
    }

    #[must_use]
    #[inline]
    pub const fn box_(&self) -> Aabb {
        match self {
            Self::Leaf(leaf) => leaf.box_,
            Self::Branch(branch) => branch.box_,
        }
    }
}

/// A bounding volume hierarchy.
#[derive(Clone, Debug)]
pub struct Bvh {
    /// The bounding box of the entire BVH.
    pub global_box: Aabb,
    /// The number of leaves that the BVH has.
    pub num_leaves: u64,
    /// The root of the BVH.
    pub root: BvhNode,
}

fn global_aabb(boxes: &[Aabb]) -> Aabb {
    let mut global_box = boxes[0];

    for b in boxes {
        global_box = global_box.add(b);
    }

    global_box
}

impl Bvh {
    /// Creates a new BVH from a list of primitives.
    #[must_use]
    pub fn from(primitives: &[Tri]) -> Self {
        let num_leaves = primitives.len();

        let mut boxes: Vec<Aabb> = Vec::with_capacity(num_leaves);
        for primitive in primitives {
            boxes.push(primitive.into());
        }

        let global_box = global_aabb(&boxes);

        let morton = Morton::from(&global_box);
        let mut sorted_leaves: Vec<Leaf> = Vec::with_capacity(num_leaves);

        for (i, box_) in boxes.iter().enumerate() {
            sorted_leaves.push(Leaf::new(primitives[i], *box_, morton.get_code(box_)));
        }

        radsort::sort_by_key(&mut sorted_leaves, |leaf| leaf.morton);

        let root = Self::generate_hierarchy(&sorted_leaves, 0, num_leaves - 1);

        Self {
            global_box,
            num_leaves: num_leaves as u64,
            root,
        }
    }

    fn generate_hierarchy(sorted_leaves: &[Leaf], first: usize, last: usize) -> BvhNode {
        // If we're dealing with a single object, return the leaf node
        if first == last {
            return BvhNode::Leaf(sorted_leaves[first]);
        }

        // Determine where to split the range

        let split = first + ((last - first) / 2);

        // Process the resulting sub-ranges recursively

        let right = Self::generate_hierarchy(sorted_leaves, first, split);
        let left = Self::generate_hierarchy(sorted_leaves, split + 1, last);

        BvhNode::branch(right, left)
    }

    /// Returns a Vec of the triangles intersecting with the `query_object`.
    #[must_use]
    pub fn intersect(&self, query_object: &Sphere) -> Vec<Tri> {
        let query_box: Aabb = query_object.into();

        let mut hits = Vec::with_capacity(16);

        // Allocate traversal stack from thread-local memory
        let mut stack: Vec<&BvhNode> = Vec::with_capacity(32);

        // Traverse nodes starting from the root.
        let mut node = &self.root;

        // Check each child node for overlap.
        loop {
            if let BvhNode::Branch(branch) = node {
                let mut traverse_left = false;

                let left = branch.left.as_ref();
                let right = branch.right.as_ref();

                if left.box_().intersect_self(&query_box) {
                    if let BvhNode::Leaf(left) = left {
                        if left.primitive.intersect_sphere(query_object) {
                            hits.push(left.primitive);
                        }
                    } else {
                        traverse_left = true;
                    }
                }

                if right.box_().intersect_self(&query_box) {
                    if let BvhNode::Leaf(right) = right {
                        if right.primitive.intersect_sphere(query_object) {
                            hits.push(right.primitive);
                        }
                    } else if traverse_left {
                        stack.push(right);
                    } else {
                        node = right;
                        continue;
                    }
                }

                if traverse_left {
                    node = left;
                    continue;
                }
            }

            match stack.pop() {
                Some(n) => node = n,
                None => break,
            }
        }

        hits
    }

    /// Returns the calculated ray-intersection of the given Sphere and the BVH.
    /// Returns None if no intersecting objects were found in the BVH.
    #[must_use]
    pub fn collide(&self, s: &Sphere) -> Option<Ray> {
        let mut contact_point = Ray::default();
        let mut count = 0;

        let tris_hit = self.intersect(s);

        for tri in tris_hit {
            let p = tri.center();
            let n = tri.unit_normal();

            let separation = (s.center - p).dot(n);
            if separation <= s.radius {
                count += 1;
                contact_point.start += s.center - n * separation;
                contact_point.direction += n * (s.radius - separation);
            }
        }

        if count == 0 {
            return None;
        }

        contact_point.start /= count as f32;
        contact_point.direction = contact_point.direction.normalize_or_zero();

        Some(contact_point)
    }
}

#[cfg(test)]
mod test {
    use criterion::black_box;
    use glam::Vec3A;

    use super::*;

    const MIN_X: f32 = -4107.33;
    const MIN_Y: f32 = -6000.0;
    const MIN_Z: f32 = -13.2678;
    const MAX_X: f32 = 4107.33;
    const MAX_Y: f32 = 6000.0;
    const MAX_Z: f32 = 2075.45;

    #[test]
    fn global_bounding_box() {
        let bounding_boxes = vec![
            Aabb::from(Vec3A::new(MIN_X, 0.0, 0.0), Vec3A::ZERO),
            Aabb::from(Vec3A::new(0.0, MIN_Y, 0.0), Vec3A::ZERO),
            Aabb::from(Vec3A::new(0.0, 0.0, MIN_Z), Vec3A::ZERO),
            Aabb::from(Vec3A::ZERO, Vec3A::new(MAX_X, 0.0, 0.0)),
            Aabb::from(Vec3A::ZERO, Vec3A::new(0.0, MAX_Y, 0.0)),
            Aabb::from(Vec3A::ZERO, Vec3A::new(0.0, 0.0, MAX_Z)),
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
            Aabb::from(Vec3A::new(MIN_X, MIN_Y, MIN_Z), Vec3A::new(MIN_X, MIN_Y, MIN_Z)),
            Aabb::from(Vec3A::new(MIN_X, MIN_Y, MIN_Z), Vec3A::new(MIN_X, MIN_Y, MIN_Z)),
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
            Aabb::from(Vec3A::new(MAX_X, MAX_Y, MAX_Z), Vec3A::new(MAX_X, MAX_Y, MAX_Z)),
            Aabb::from(Vec3A::new(MAX_X, MAX_Y, MAX_Z), Vec3A::new(MAX_X, MAX_Y, MAX_Z)),
        ];
        let global = global_aabb(&bounding_boxes);

        assert!((global.min().x - MAX_X).abs() < f32::EPSILON);
        assert!((global.min().y - MAX_Y).abs() < f32::EPSILON);
        assert!((global.min().z - MAX_Z).abs() < f32::EPSILON);
        assert!((global.max().x - MAX_X).abs() < f32::EPSILON);
        assert!((global.max().y - MAX_Y).abs() < f32::EPSILON);
        assert!((global.max().z - MAX_Z).abs() < f32::EPSILON);
    }

    static VERT_MAP: &[[usize; 3]; 12] = &[
        [1, 0, 2],
        [3, 1, 2],
        [7, 5, 6],
        [4, 6, 5],
        [2, 0, 4],
        [6, 2, 4],
        [7, 3, 5],
        [1, 5, 3],
        [4, 0, 1],
        [5, 4, 1],
        [7, 6, 3],
        [2, 3, 6],
    ];

    fn generate_tris() -> Vec<Tri> {
        let verts = &[
            Vec3A::new(-4096.0, -5120.0, 0.0),
            Vec3A::new(-4096.0, -5120.0, 2044.0),
            Vec3A::new(-4096.0, 5120.0, 0.0),
            Vec3A::new(-4096.0, 5120.0, 2044.0),
            Vec3A::new(4096.0, -5120.0, 0.0),
            Vec3A::new(4096.0, -5120.0, 2044.0),
            Vec3A::new(4096.0, 5120.0, 0.0),
            Vec3A::new(4096.0, 5120.0, 2044.0),
        ];
        VERT_MAP
            .iter()
            .map(|map| {
                let p = [verts[map[0]], verts[map[1]], verts[map[2]]];
                Tri { p }
            })
            .collect()
    }

    #[test]
    fn test_bvh_build() {
        let triangles = generate_tris();

        let _ = black_box(Bvh::from(&triangles));
    }

    #[test]
    fn test_bvh_intersect() {
        let triangles = generate_tris();

        let bvh = Bvh::from(&triangles);
        {
            // Sphere hits nothing
            let sphere = Sphere {
                center: Vec3A::new(0., 0., 1022.),
                radius: 100.,
            };
            let hits = bvh.intersect(&sphere);
            assert_eq!(hits.len(), 0);
        }
        {
            // Sphere hits one Tri
            let sphere = Sphere {
                center: Vec3A::new(4096. / 2., 5120. / 2., 100.),
                radius: 100.,
            };
            let hits = bvh.intersect(&sphere);

            assert_eq!(hits.len(), 1);
            let p0 = hits[0].p[0];
            assert!((p0.x - 4096.).abs() < f32::EPSILON);
            assert!((p0.y - 5120.).abs() < f32::EPSILON);
            assert!((p0.z - 0.).abs() < f32::EPSILON);
            let p1 = hits[0].p[1];
            assert!((p1.x - -4096.).abs() < f32::EPSILON);
            assert!((p1.y - 5120.).abs() < f32::EPSILON);
            assert!((p1.z - 0.).abs() < f32::EPSILON);
            let p2 = hits[0].p[2];
            assert!((p2.x - 4096.).abs() < f32::EPSILON);
            assert!((p2.y - -5120.).abs() < f32::EPSILON);
            assert!((p2.z - 0.).abs() < f32::EPSILON);
        }
        {
            // Middle of two Tris
            let sphere = Sphere {
                center: Vec3A::ZERO,
                radius: 100.,
            };
            let hits = bvh.intersect(&sphere);

            assert_eq!(hits.len(), 2);
        }
        {
            // Sphere is in a corner
            let sphere = Sphere {
                center: Vec3A::new(4096., 5120., 0.),
                radius: 100.,
            };
            let hits = bvh.intersect(&sphere);

            assert_eq!(hits.len(), 5);
        }
    }

    #[test]
    fn test_bvh_collide() {
        let triangles = generate_tris();

        let bvh = Bvh::from(&triangles);

        {
            // Sphere hits nothing
            let sphere = Sphere {
                center: Vec3A::new(0., 0., 1022.),
                radius: 100.,
            };

            let ray = bvh.collide(&sphere);

            assert!(ray.is_none());
        }
        {
            // Sphere hits one Tri
            let sphere = Sphere {
                center: Vec3A::new(4096. / 2., 5120. / 2., 99.),
                radius: 100.,
            };

            let ray = bvh.collide(&sphere);

            assert!(ray.is_some());
            let ray = ray.unwrap();
            assert!((ray.start.x - 2048.).abs() < f32::EPSILON);
            assert!((ray.start.y - 2560.).abs() < f32::EPSILON);
            assert!((ray.start.z - 0.).abs() < f32::EPSILON);
            assert!((ray.direction.x - 0.0).abs() < f32::EPSILON);
            assert!((ray.direction.y - 0.0).abs() < f32::EPSILON);
            assert!((ray.direction.z - 1.0).abs() < f32::EPSILON);
        }
        {
            // Middle of two Tris
            let sphere = Sphere {
                center: Vec3A::ZERO,
                radius: 100.,
            };

            let ray = bvh.collide(&sphere);

            assert!(ray.is_some());
            let ray = ray.unwrap();
            assert!((ray.start.x - 0.0).abs() < f32::EPSILON);
            assert!((ray.start.y - 0.0).abs() < f32::EPSILON);
            assert!((ray.start.z - 0.0).abs() < f32::EPSILON);
            assert!((ray.direction.x - 0.0).abs() < f32::EPSILON);
            assert!((ray.direction.y - 0.0).abs() < f32::EPSILON);
            assert!((ray.direction.z - 1.0).abs() < f32::EPSILON);
        }
        {
            // Sphere is in a corner
            let sphere = Sphere {
                center: Vec3A::new(4096., 5120., 0.),
                radius: 100.,
            };

            let ray = bvh.collide(&sphere);

            assert!(ray.is_some());
            let ray = ray.unwrap();
            assert!((ray.start.x - 4096.).abs() < f32::EPSILON);
            assert!((ray.start.y - 5120.).abs() < f32::EPSILON);
            assert!((ray.start.z - 0.0).abs() < f32::EPSILON);
            assert!((ray.direction.x - 0.666_666_7).abs() < f32::EPSILON);
            assert!((ray.direction.y - 0.666_666_7).abs() < f32::EPSILON);
            assert!((ray.direction.z - 0.333_333_34).abs() < f32::EPSILON);
        }
    }

    #[test]
    fn is_collision_ray_finite() {
        let triangles = generate_tris();

        let bvh = Bvh::from(&triangles);

        {
            let sphere = Sphere {
                center: Vec3A::new(0., 0., 93.15),
                radius: 93.15,
            };

            let ray = bvh.collide(&sphere);

            assert!(ray.is_some());
            let ray = ray.unwrap();
            assert!(ray.direction.is_finite());
            assert!(ray.start.is_finite());
        }
    }
}
