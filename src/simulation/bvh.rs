//! Tools for calculating collisions between objects and the Rocket League field.

use combo_vec::{rearr, ReArr};

use super::{
    geometry::{Aabb, Ray, Sphere, Tri},
    morton::Morton,
};
use std::{boxed::Box, convert::Into};

/// A leaf in the BVH.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Leaf {
    /// The bounding box of this leaf.
    pub box_: Aabb,
    /// The primitive that this leaf represents.
    pub primitive: Tri,
    /// The morton code of this leaf.
    pub morton: u64,
}

impl Leaf {
    #[must_use]
    #[inline]
    /// Create a new leaf.
    pub const fn new(primitive: Tri, box_: Aabb, morton: u64) -> Self {
        Self { box_, primitive, morton }
    }
}

/// A branch in the BVH.
#[derive(Clone, Debug)]
pub(crate) struct Branch {
    /// The bounding box of this branch.
    pub box_: Aabb,
    /// The left child of this branch.
    pub left: Box<BvhNode>,
    /// The right child of this branch.
    pub right: Box<BvhNode>,
}

impl Branch {
    #[must_use]
    #[inline]
    /// Create a new branch.
    pub const fn new(box_: Aabb, left: Box<BvhNode>, right: Box<BvhNode>) -> Self {
        Self { box_, left, right }
    }
}

/// A node in the BVH.
#[derive(Clone, Debug)]
pub(crate) enum BvhNode {
    /// A leaf node at the end of a series of branches
    Leaf(Leaf),
    /// A branch node that connects to more nodes
    Branch(Branch),
}

impl BvhNode {
    #[must_use]
    #[inline]
    /// Creates a new branch for the BVH given two children.
    pub fn branch(right: Self, left: Self) -> Self {
        Self::Branch(Branch::new(*right.box_() + *left.box_(), Box::new(left), Box::new(right)))
    }

    #[must_use]
    #[inline]
    /// Returns the bounding box of this node.
    pub const fn box_(&self) -> &Aabb {
        match self {
            Self::Leaf(leaf) => &leaf.box_,
            Self::Branch(branch) => &branch.box_,
        }
    }
}

/// A bounding volume hierarchy.
#[derive(Clone, Debug)]
pub(crate) struct Bvh {
    /// The bounding box of the entire BVH; used in tests
    #[allow(dead_code)]
    global_box: Aabb,
    /// The number of leaves that the BVH has; used in tests
    #[allow(dead_code)]
    num_leaves: usize,
    /// The root of the BVH.
    pub root: BvhNode,
}

#[inline]
fn global_aabb(boxes: &[Aabb]) -> Aabb {
    boxes.iter().copied().fold(boxes[0], |a, b| a + b)
}

impl Bvh {
    #[must_use]
    /// Creates a new BVH from a list of primitives.
    pub fn from(primitives: &[Tri]) -> Self {
        let num_leaves = primitives.len();

        let boxes: Vec<Aabb> = primitives.iter().map(Into::into).collect();
        let global_box = global_aabb(&boxes);
        let morton = Morton::from(global_box);

        let mut sorted_leaves: Vec<Leaf> = boxes
            .into_iter()
            .enumerate()
            .map(|(i, box_)| Leaf::new(primitives[i], box_, morton.get_code(box_)))
            .collect();
        radsort::sort_by_key(&mut sorted_leaves, |leaf| leaf.morton);

        let root = Self::generate_hierarchy(&sorted_leaves, 0, num_leaves - 1);

        Self { global_box, num_leaves, root }
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

    #[must_use]
    /// Returns a Vec of the triangles intersecting with the `query_object`.
    pub fn intersect(&self, query_object: Sphere) -> Vec<Tri> {
        const STACK: ReArr<&BvhNode, 8> = rearr![];

        let query_box = Aabb::from(query_object);
        let mut hits = Vec::with_capacity(4);

        // Allocate traversal stack from thread-local memory
        let mut stack = STACK;

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

            let Some(next_node) = stack.pop() else {
                break;
            };

            node = next_node;
        }

        hits
    }

    #[must_use]
    /// Returns the calculated ray-intersection of the given Sphere and the BVH.
    /// Returns None if no intersecting objects were found in the BVH.
    pub fn collide(&self, obj: Sphere) -> Option<Ray> {
        let tris_hit = self.intersect(obj);
        if tris_hit.is_empty() {
            return None;
        }

        let (mut contact_point, count) = tris_hit.into_iter().fold((Ray::default(), 0u8), |(mut contact_point, mut count), tri| {
            let point = tri.center();
            let normal = tri.unit_normal();

            let separation = (obj.center - point).dot(normal);
            if separation <= obj.radius {
                count += 1;
                contact_point.start += obj.center - normal * separation;
                contact_point.direction += normal * (obj.radius - separation);
            }

            (contact_point, count)
        });

        if count == 0 {
            None
        } else {
            contact_point.start /= f32::from(count);
            contact_point.direction = contact_point.direction.normalize();

            Some(contact_point)
        }
    }
}

#[cfg(test)]
mod test {
    #![allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]

    use super::*;
    use crate::{load_dropshot, load_hoops, load_standard, load_standard_throwback};
    use criterion::black_box;
    use glam::Vec3A;
    use rand::Rng;

    const MIN_X: f32 = -4107.33;
    const MIN_Y: f32 = -6000.0;
    const MIN_Z: f32 = -13.2678;
    const MAX_X: f32 = 4107.33;
    const MAX_Y: f32 = 6000.0;
    const MAX_Z: f32 = 2075.45;

    #[test]
    fn global_bounding_box() {
        let bounding_boxes = vec![
            Aabb::from_minmax(Vec3A::new(MIN_X, 0.0, 0.0), Vec3A::ZERO),
            Aabb::from_minmax(Vec3A::new(0.0, MIN_Y, 0.0), Vec3A::ZERO),
            Aabb::from_minmax(Vec3A::new(0.0, 0.0, MIN_Z), Vec3A::ZERO),
            Aabb::from_minmax(Vec3A::ZERO, Vec3A::new(MAX_X, 0.0, 0.0)),
            Aabb::from_minmax(Vec3A::ZERO, Vec3A::new(0.0, MAX_Y, 0.0)),
            Aabb::from_minmax(Vec3A::ZERO, Vec3A::new(0.0, 0.0, MAX_Z)),
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
            Aabb::from_minmax(Vec3A::new(MIN_X, MIN_Y, MIN_Z), Vec3A::new(MIN_X, MIN_Y, MIN_Z)),
            Aabb::from_minmax(Vec3A::new(MIN_X, MIN_Y, MIN_Z), Vec3A::new(MIN_X, MIN_Y, MIN_Z)),
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
            Aabb::from_minmax(Vec3A::new(MAX_X, MAX_Y, MAX_Z), Vec3A::new(MAX_X, MAX_Y, MAX_Z)),
            Aabb::from_minmax(Vec3A::new(MAX_X, MAX_Y, MAX_Z), Vec3A::new(MAX_X, MAX_Y, MAX_Z)),
        ];
        let global = global_aabb(&bounding_boxes);

        assert!((global.min().x - MAX_X).abs() < f32::EPSILON);
        assert!((global.min().y - MAX_Y).abs() < f32::EPSILON);
        assert!((global.min().z - MAX_Z).abs() < f32::EPSILON);
        assert!((global.max().x - MAX_X).abs() < f32::EPSILON);
        assert!((global.max().y - MAX_Y).abs() < f32::EPSILON);
        assert!((global.max().z - MAX_Z).abs() < f32::EPSILON);
    }

    const VERT_MAP: &[[usize; 3]; 12] = &[
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
        VERT_MAP.iter().map(|map| Tri::from_points(verts[map[0]], verts[map[1]], verts[map[2]])).collect()
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
            let hits = bvh.intersect(sphere);
            assert_eq!(hits.len(), 0);
        }
        {
            // Sphere hits one Tri
            let sphere = Sphere {
                center: Vec3A::new(4096. / 2., 5120. / 2., 100.),
                radius: 100.,
            };
            let hits = bvh.intersect(sphere);

            assert_eq!(hits.len(), 1);
            let p0 = hits[0].get_point(0);
            assert!((p0.x - 4096.).abs() < f32::EPSILON);
            assert!((p0.y - 5120.).abs() < f32::EPSILON);
            assert!((p0.z - 0.).abs() < f32::EPSILON);
            let p1 = hits[0].get_point(1);
            assert!((p1.x - -4096.).abs() < f32::EPSILON);
            assert!((p1.y - 5120.).abs() < f32::EPSILON);
            assert!((p1.z - 0.).abs() < f32::EPSILON);
            let p2 = hits[0].get_point(2);
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
            let hits = bvh.intersect(sphere);

            assert_eq!(hits.len(), 2);
        }
        {
            // Sphere is in a corner
            let sphere = Sphere {
                center: Vec3A::new(4096., 5120., 0.),
                radius: 100.,
            };
            let hits = bvh.intersect(sphere);

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

            let ray = bvh.collide(sphere);

            assert!(ray.is_none());
        }
        {
            // Sphere hits one Tri
            let sphere = Sphere {
                center: Vec3A::new(4096. / 2., 5120. / 2., 99.),
                radius: 100.,
            };

            let ray = bvh.collide(sphere);

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

            let ray = bvh.collide(sphere);

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

            let ray = bvh.collide(sphere);

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

            let ray = bvh.collide(sphere);

            assert!(ray.is_some());
            let ray = ray.unwrap();
            assert!(ray.direction.is_finite());
            assert!(ray.start.is_finite());
        }
    }

    #[test]
    fn basic_predict_standard() {
        let (game, mut ball) = load_standard();

        assert_eq!(ball.time as i64, 0);
        assert_eq!(ball.location.x as i64, 0);
        assert_eq!(ball.location.y as i64, 0);
        assert_eq!(ball.location.z as i64, 102);
        assert_eq!(ball.velocity.x as i64, 0);
        assert_eq!(ball.velocity.y as i64, 0);
        assert_eq!(ball.velocity.z as i64, 0);
        assert_eq!(ball.angular_velocity.x as i64, 0);
        assert_eq!(ball.angular_velocity.y as i64, 0);
        assert_eq!(ball.angular_velocity.z as i64, 0);
        assert_eq!(ball.radius as i64, 91);
        assert_eq!(ball.collision_radius as i64, 93);

        ball.update(
            0.098_145,
            Vec3A::new(-2294.5247, 1684.136, 317.17673),
            Vec3A::new(1273.7537, -39.792_305, 763.2827),
            Vec3A::new(2.3894, -0.8755, 3.8078),
        );

        let time = 60.; // 1 minute, lol
        let ball_prediction = ball.get_ball_prediction_struct_for_time(&game, time);
        assert_eq!(ball_prediction.len(), time as usize * 120);

        let iters = 200;
        let time = 10.; // 10 seconds
        let num_slices = time as usize * 120 * iters;
        let mut rng = rand::thread_rng();

        let mut x_locs = Vec::with_capacity(num_slices);
        let mut y_locs = Vec::with_capacity(num_slices);
        let mut z_locs = Vec::with_capacity(num_slices);

        dbg!(game.collision_mesh.global_box);

        for _ in 0..iters {
            ball.update(
                0.,
                Vec3A::new(rng.gen_range(-3200.0..3200.), rng.gen_range(-4500.0..4500.), rng.gen_range(100.0..1900.)),
                Vec3A::new(rng.gen_range(-2000.0..2000.), rng.gen_range(-2000.0..2000.), rng.gen_range(-2000.0..2000.)),
                Vec3A::new(rng.gen_range(-3.0..3.), rng.gen_range(-3.0..3.), rng.gen_range(-3.0..3.)),
            );

            let ball_prediction = ball.get_ball_prediction_struct(&game);

            for slice in ball_prediction {
                x_locs.push(slice.location.x as isize);
                y_locs.push(slice.location.y as isize);
                z_locs.push(slice.location.z as isize);
            }
        }

        dbg!(*x_locs.iter().min().unwrap());
        dbg!(*x_locs.iter().max().unwrap());

        dbg!(*y_locs.iter().min().unwrap());
        dbg!(*y_locs.iter().max().unwrap());

        dbg!(*z_locs.iter().min().unwrap());
        dbg!(*z_locs.iter().max().unwrap());

        assert!(*z_locs.iter().min().unwrap() > game.collision_mesh.global_box.min().z as isize);
        assert!(*z_locs.iter().max().unwrap() < game.collision_mesh.global_box.max().z as isize);

        assert!(*y_locs.iter().min().unwrap() > game.collision_mesh.global_box.min().y as isize);
        assert!(*y_locs.iter().max().unwrap() < game.collision_mesh.global_box.max().y as isize);

        assert!(*x_locs.iter().min().unwrap() > game.collision_mesh.global_box.min().x as isize);
        assert!(*x_locs.iter().max().unwrap() < game.collision_mesh.global_box.max().x as isize);
    }

    #[test]
    fn basic_predict_throwback() {
        let (game, mut ball) = load_standard_throwback();

        assert_eq!(ball.time as i64, 0);
        assert_eq!(ball.location.x as i64, 0);
        assert_eq!(ball.location.y as i64, 0);
        assert_eq!(ball.location.z as i64, 102);
        assert_eq!(ball.velocity.x as i64, 0);
        assert_eq!(ball.velocity.y as i64, 0);
        assert_eq!(ball.velocity.z as i64, 0);
        assert_eq!(ball.angular_velocity.x as i64, 0);
        assert_eq!(ball.angular_velocity.y as i64, 0);
        assert_eq!(ball.angular_velocity.z as i64, 0);
        assert_eq!(ball.radius as i64, 91);
        assert_eq!(ball.collision_radius as i64, 93);

        ball.update(
            0.098_145,
            Vec3A::new(-2294.5247, 1684.136, 317.17673),
            Vec3A::new(1273.7537, -39.792_305, 763.2827),
            Vec3A::new(2.3894, -0.8755, 3.8078),
        );

        let time = 60.; // 1 minute, lol
        let ball_prediction = ball.get_ball_prediction_struct_for_time(&game, time);
        assert_eq!(ball_prediction.len(), time as usize * 120);

        let iters = 200;
        let time = 10.; // 10 seconds
        let num_slices = time as usize * 120 * iters;
        let mut rng = rand::thread_rng();

        let mut x_locs = Vec::with_capacity(num_slices);
        let mut y_locs = Vec::with_capacity(num_slices);
        let mut z_locs = Vec::with_capacity(num_slices);

        dbg!(game.collision_mesh.global_box);

        for _ in 0..iters {
            ball.update(
                0.,
                Vec3A::new(rng.gen_range(-3900.0..3900.), rng.gen_range(-5000.0..5000.), rng.gen_range(100.0..1900.)),
                Vec3A::new(rng.gen_range(-2000.0..2000.), rng.gen_range(-2000.0..2000.), rng.gen_range(-2000.0..2000.)),
                Vec3A::new(rng.gen_range(-3.0..3.), rng.gen_range(-3.0..3.), rng.gen_range(-3.0..3.)),
            );

            let ball_prediction = ball.get_ball_prediction_struct(&game);

            for slice in ball_prediction {
                if slice.location.y.abs() > 5120. + slice.radius {
                    break;
                }

                x_locs.push(slice.location.x as isize);
                y_locs.push(slice.location.y as isize);
                z_locs.push(slice.location.z as isize);
            }
        }

        dbg!(*x_locs.iter().min().unwrap());
        dbg!(*x_locs.iter().max().unwrap());

        dbg!(*y_locs.iter().min().unwrap());
        dbg!(*y_locs.iter().max().unwrap());

        dbg!(*z_locs.iter().min().unwrap());
        dbg!(*z_locs.iter().max().unwrap());

        assert!(*z_locs.iter().min().unwrap() > game.collision_mesh.global_box.min().z as isize);
        assert!(*z_locs.iter().max().unwrap() < game.collision_mesh.global_box.max().z as isize);

        assert!(*y_locs.iter().min().unwrap() > game.collision_mesh.global_box.min().y as isize);
        assert!(*y_locs.iter().max().unwrap() < game.collision_mesh.global_box.max().y as isize);

        assert!(*x_locs.iter().min().unwrap() > game.collision_mesh.global_box.min().x as isize);
        assert!(*x_locs.iter().max().unwrap() < game.collision_mesh.global_box.max().x as isize);
    }

    #[test]
    fn gamemode_standard() {
        let (game, ball) = load_standard();

        // test all the default values to make sure they're proper

        assert_eq!(game.gravity.x as i64, 0);
        assert_eq!(game.gravity.y as i64, 0);
        assert_eq!(game.gravity.z as i64, -650);

        dbg!(game.collision_mesh.root.box_());

        assert_eq!(game.collision_mesh.num_leaves, 8028);

        assert_eq!(ball.time as i64, 0);
        assert_eq!(ball.location.x as i64, 0);
        assert_eq!(ball.location.y as i64, 0);
        assert_eq!(ball.location.z as i64, 102);
        assert_eq!(ball.velocity.x as i64, 0);
        assert_eq!(ball.velocity.y as i64, 0);
        assert_eq!(ball.velocity.z as i64, 0);
        assert_eq!(ball.angular_velocity.x as i64, 0);
        assert_eq!(ball.angular_velocity.y as i64, 0);
        assert_eq!(ball.angular_velocity.z as i64, 0);
        assert_eq!(ball.radius as i64, 91);
        assert_eq!(ball.collision_radius as i64, 93);
    }

    #[test]
    fn gamemode_hoops() {
        let (game, ball) = load_hoops();

        // test all the default values to make sure they're proper

        assert_eq!(game.gravity.x as i64, 0);
        assert_eq!(game.gravity.y as i64, 0);
        assert_eq!(game.gravity.z as i64, -650);

        dbg!(game.collision_mesh.root.box_());

        assert_eq!(game.collision_mesh.num_leaves, 15732);

        assert_eq!(ball.time as i64, 0);
        assert_eq!(ball.location.x as i64, 0);
        assert_eq!(ball.location.y as i64, 0);
        assert_eq!(ball.location.z as i64, 102);
        assert_eq!(ball.velocity.x as i64, 0);
        assert_eq!(ball.velocity.y as i64, 0);
        assert_eq!(ball.velocity.z as i64, 0);
        assert_eq!(ball.angular_velocity.x as i64, 0);
        assert_eq!(ball.angular_velocity.y as i64, 0);
        assert_eq!(ball.angular_velocity.z as i64, 0);
        assert_eq!(ball.radius as i64, 91);
        assert_eq!(ball.collision_radius as i64, 93);
    }

    #[test]
    fn gamemode_dropshot() {
        let (game, ball) = load_dropshot();

        // test all the default values to make sure they're proper

        assert_eq!(game.gravity.x as i64, 0);
        assert_eq!(game.gravity.y as i64, 0);
        assert_eq!(game.gravity.z as i64, -650);

        dbg!(game.collision_mesh.root.box_());

        assert_eq!(game.collision_mesh.num_leaves, 3616);

        assert_eq!(ball.time as i64, 0);
        assert_eq!(ball.location.x as i64, 0);
        assert_eq!(ball.location.y as i64, 0);
        assert_eq!(ball.location.z as i64, 113);
        assert_eq!(ball.velocity.x as i64, 0);
        assert_eq!(ball.velocity.y as i64, 0);
        assert_eq!(ball.velocity.z as i64, 0);
        assert_eq!(ball.angular_velocity.x as i64, 0);
        assert_eq!(ball.angular_velocity.y as i64, 0);
        assert_eq!(ball.angular_velocity.z as i64, 0);
        assert_eq!(ball.radius as i64, 100);
        assert_eq!(ball.collision_radius as i64, 103);
    }

    #[test]
    fn gamemode_throwback_standard() {
        let (game, ball) = load_standard_throwback();

        // test all the default values to make sure they're proper

        assert_eq!(game.gravity.x as i64, 0);
        assert_eq!(game.gravity.y as i64, 0);
        assert_eq!(game.gravity.z as i64, -650);

        dbg!(&game.collision_mesh.root.box_());
        if let BvhNode::Branch(branch) = &game.collision_mesh.root {
            dbg!(branch.left.box_());
            dbg!(branch.right.box_());
        }

        assert_eq!(game.collision_mesh.num_leaves, 9272);

        assert_eq!(ball.time as i64, 0);
        assert_eq!(ball.location.x as i64, 0);
        assert_eq!(ball.location.y as i64, 0);
        assert_eq!(ball.location.z as i64, 102);
        assert_eq!(ball.velocity.x as i64, 0);
        assert_eq!(ball.velocity.y as i64, 0);
        assert_eq!(ball.velocity.z as i64, 0);
        assert_eq!(ball.angular_velocity.x as i64, 0);
        assert_eq!(ball.angular_velocity.y as i64, 0);
        assert_eq!(ball.angular_velocity.z as i64, 0);
        assert_eq!(ball.radius as i64, 91);
        assert_eq!(ball.collision_radius as i64, 93);
    }

    fn recurse_bvhnode(node: &BvhNode, depth: usize, max_depth: &mut usize) {
        if depth > *max_depth {
            *max_depth = depth;
        }

        if let BvhNode::Branch(branch) = node {
            recurse_bvhnode(&branch.left, depth + 1, max_depth);
            recurse_bvhnode(&branch.right, depth + 1, max_depth);
        }
    }

    #[test]
    fn hierarchy_depth_throwback_standard() {
        let (game, _) = load_standard_throwback();
        let mut max_depth = 0;

        recurse_bvhnode(&game.collision_mesh.root, 0, &mut max_depth);
        assert_eq!(max_depth, 14);
    }

    #[test]
    fn hierarchy_depth_hoops() {
        let (game, _) = load_hoops();
        let mut max_depth = 0;

        recurse_bvhnode(&game.collision_mesh.root, 0, &mut max_depth);
        assert_eq!(max_depth, 14);
    }

    #[test]
    fn hierarchy_depth_standard() {
        let (game, _) = load_standard();
        let mut max_depth = 0;

        recurse_bvhnode(&game.collision_mesh.root, 0, &mut max_depth);
        assert_eq!(max_depth, 13);
    }

    #[test]
    fn hierarchy_depth_dropshot() {
        let (game, _) = load_dropshot();
        let mut max_depth = 0;

        recurse_bvhnode(&game.collision_mesh.root, 0, &mut max_depth);
        assert_eq!(max_depth, 12);
    }
}
