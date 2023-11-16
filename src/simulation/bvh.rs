//! Tools for calculating collisions between objects and the Rocket League field.

use super::{
    geometry::{Aabb, Ray, Sphere, Tri},
    morton::Morton,
};
use combo_vec::{rearr, ReArr};
use glam::Vec3A;
use std::{boxed::Box, convert::Into, ops::Add};

/// A leaf in the BVH.
#[derive(Clone, Copy, Debug)]
pub struct Leaf {
    /// The bounding box of this leaf.
    pub aabb: Aabb,
    /// The primitive that this leaf represents.
    pub primitive: Tri,
    /// The morton code of this leaf.
    pub morton: u64,
}

impl Leaf {
    #[must_use]
    #[inline]
    /// Create a new leaf.
    pub const fn new(primitive: Tri, aabb: Aabb, morton: u64) -> Self {
        Self { aabb, primitive, morton }
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

/// A bounding volume hierarchy.
#[derive(Clone, Debug)]
pub struct Bvh {
    /// The number of leaves that the BVH has; used in tests
    pub num_leaves: usize,
    /// The root of the BVH.
    pub root: Node,
}

#[inline]
fn global_aabb(boxes: &[Aabb]) -> Aabb {
    boxes.iter().skip(1).copied().fold(boxes[0], Add::add)
}

impl Bvh {
    #[must_use]
    /// Creates a new BVH from a list of primitives.
    pub fn from(primitives: &[Tri]) -> Self {
        let num_leaves = primitives.len();

        let boxes: Vec<Aabb> = primitives.iter().copied().map(Into::into).collect();
        let global_box = global_aabb(&boxes);
        let morton = Morton::from(global_box);

        let mut sorted_leaves: Vec<Leaf> = primitives
            .iter()
            .copied()
            .zip(boxes)
            .map(|(primitive, box_)| Leaf::new(primitive, box_, morton.get_code(box_)))
            .collect();
        radsort::sort_by_key(&mut sorted_leaves, |leaf| leaf.morton);

        let root = Self::generate_hierarchy(&sorted_leaves);

        Self { num_leaves, root }
    }

    fn generate_hierarchy(sorted_leaves: &[Leaf]) -> Node {
        // If we're dealing with a single object, return the leaf node
        if sorted_leaves.len() == 1 {
            return Node::Leaf(sorted_leaves[0]);
        }

        // Determine where to split the range
        let split = sorted_leaves.len() / 2;

        // Process the resulting sub-ranges recursively
        let right = Self::generate_hierarchy(&sorted_leaves[..split]);
        let left = Self::generate_hierarchy(&sorted_leaves[split..]);

        Node::branch(right, left)
    }

    #[must_use]
    /// Returns a Vec of the collision rays and triangle normals from the triangles intersecting with the `query_object`.
    pub fn collide(&self, query_object: Sphere) -> Vec<(Ray, Vec3A)> {
        const STACK: ReArr<&Branch, 8> = rearr![];

        // Traverse nodes starting from the root
        // If the node is a leaf, check for intersection
        let mut node = match &self.root {
            Node::Branch(branch) => branch,
            Node::Leaf(leaf) => {
                return leaf
                    .primitive
                    .intersect_sphere(query_object)
                    .map_or_else(Vec::new, |info| vec![info]);
            }
        };

        let query_box = Aabb::from(query_object);
        let mut hits = Vec::with_capacity(4);

        // Allocate traversal stack from thread-local memory
        let mut stack = STACK;

        // Check each child node for overlap.
        loop {
            let mut traverse_left = None;

            let left = node.left.as_ref();
            let right = node.right.as_ref();

            if left.aabb().intersect_self(&query_box) {
                match left {
                    Node::Leaf(left) => {
                        if let Some(info) = left.primitive.intersect_sphere(query_object) {
                            hits.push(info);
                        }
                    }
                    Node::Branch(left) => traverse_left = Some(left),
                }
            }

            if right.aabb().intersect_self(&query_box) {
                match right {
                    Node::Leaf(right) => {
                        if let Some(info) = right.primitive.intersect_sphere(query_object) {
                            hits.push(info);
                        }
                    }
                    Node::Branch(right) => {
                        if traverse_left.is_some() {
                            stack.push(right);
                        } else {
                            node = right;
                            continue;
                        }
                    }
                }
            }

            if let Some(branch) = traverse_left {
                node = branch;
                continue;
            }

            let Some(next_node) = stack.pop() else {
                break;
            };

            node = next_node;
        }

        hits
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
        VERT_MAP
            .iter()
            .map(|map| Tri::from_points(verts[map[0]], verts[map[1]], verts[map[2]]))
            .collect()
    }

    #[test]
    fn test_bvh_build() {
        let triangles = generate_tris();

        let _ = black_box(Bvh::from(&triangles));
    }

    #[test]
    fn test_bvh_collide_count() {
        let triangles = generate_tris();

        let bvh = Bvh::from(&triangles);
        {
            // Sphere hits nothing
            let sphere = Sphere::new(Vec3A::new(0., 0., 1022.), 100.);
            let hits = bvh.collide(sphere);
            assert_eq!(hits.len(), 0);
        }
        {
            // Sphere hits one Tri
            let sphere = Sphere::new(Vec3A::new(4096. / 2., 5120. / 2., 99.9), 100.);
            let hits = bvh.collide(sphere);

            assert_eq!(hits.len(), 1);
        }
        {
            // Middle of two Tris
            let sphere = Sphere::new(Vec3A::ZERO, 100.);
            let hits = bvh.collide(sphere);

            assert_eq!(hits.len(), 2);
        }
        {
            // Sphere is in a corner
            let sphere = Sphere::new(Vec3A::new(4096., 5120., 0.), 100.);
            let hits = bvh.collide(sphere);

            assert_eq!(hits.len(), 5);
        }
    }

    #[test]
    fn test_bvh_collide() {
        let triangles = generate_tris();

        let bvh = Bvh::from(&triangles);

        {
            // Sphere hits nothing
            let sphere = Sphere::new(Vec3A::new(0., 0., 1022.), 100.);

            let ray = bvh.collide(sphere);

            assert!(ray.is_empty());
        }
        {
            // Sphere hits one Tri
            let sphere = Sphere::new(Vec3A::new(4096. / 2., 5120. / 2., 99.), 100.);

            let rays = bvh.collide(sphere);
            assert!(!rays.is_empty());

            let ray = rays[0].0;

            assert!((ray.start.x - 2048.).abs() < f32::EPSILON);
            assert!((ray.start.y - 2560.).abs() < f32::EPSILON);
            assert!((ray.start.z - 0.).abs() < f32::EPSILON);
            assert!((ray.direction.x - 0.0).abs() < f32::EPSILON);
            assert!((ray.direction.y - 0.0).abs() < f32::EPSILON);
            assert!((ray.direction.z - 1.0).abs() < f32::EPSILON);
        }
        {
            // Middle of two Tris
            let sphere = Sphere::new(Vec3A::ZERO, 100.);

            let rays = bvh.collide(sphere);
            assert!(!rays.is_empty());

            let ray = rays[0].0;

            assert!((ray.start.x - 0.0).abs() < f32::EPSILON);
            assert!((ray.start.y - 0.0).abs() < f32::EPSILON);
            assert!((ray.start.z - 0.0).abs() < f32::EPSILON);
            assert!((ray.direction.x - 0.0).abs() < f32::EPSILON);
            assert!((ray.direction.y - 0.0).abs() < f32::EPSILON);
            assert!((ray.direction.z - 1.0).abs() < f32::EPSILON);
        }
        {
            // Sphere is in a corner
            let sphere = Sphere::new(Vec3A::new(4096., 5120., 0.), 100.);

            let rays = bvh.collide(sphere);
            assert!(!rays.is_empty());

            let ray = rays[0].0;

            assert!((ray.start.x - 4096.).abs() < f32::EPSILON);
            assert!((ray.start.y - 5120.).abs() < f32::EPSILON);
            assert!((ray.start.z - 0.0).abs() < f32::EPSILON);
            let mut ray = rays.iter().fold(Ray::default(), |mut acc, ray| {
                acc += ray.0;
                acc
            });
            ray.direction = ray.direction.normalize();
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
            let sphere = Sphere::new(Vec3A::new(0., 0., 92.15 - f32::EPSILON), 92.15);
            let ray = bvh.collide(sphere);

            assert_eq!(ray.len(), 2);
        }
    }

    #[test]
    fn basic_predict_standard() {
        let (game, mut ball) = load_standard();

        assert_eq!(ball.time as i64, 0);
        assert_eq!(ball.location.x as i64, 0);
        assert_eq!(ball.location.y as i64, 0);
        assert_eq!(ball.location.z as i64, 100);
        assert_eq!(ball.velocity.x as i64, 0);
        assert_eq!(ball.velocity.y as i64, 0);
        assert_eq!(ball.velocity.z as i64, 0);
        assert_eq!(ball.angular_velocity.x as i64, 0);
        assert_eq!(ball.angular_velocity.y as i64, 0);
        assert_eq!(ball.angular_velocity.z as i64, 0);
        assert_eq!(ball.radius as i64, 91);

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

        dbg!(game.collision_mesh.root.aabb());

        for _ in 0..iters {
            ball.update(
                0.,
                Vec3A::new(
                    rng.gen_range(-3200.0..3200.),
                    rng.gen_range(-4500.0..4500.),
                    rng.gen_range(100.0..1900.),
                ),
                Vec3A::new(
                    rng.gen_range(-2000.0..2000.),
                    rng.gen_range(-2000.0..2000.),
                    rng.gen_range(-2000.0..2000.),
                ),
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

        assert!(*z_locs.iter().min().unwrap() > game.collision_mesh.root.aabb().min().z as isize);
        assert!(*z_locs.iter().max().unwrap() < game.collision_mesh.root.aabb().max().z as isize);

        assert!(*y_locs.iter().min().unwrap() > game.collision_mesh.root.aabb().min().y as isize);
        assert!(*y_locs.iter().max().unwrap() < game.collision_mesh.root.aabb().max().y as isize);

        assert!(*x_locs.iter().min().unwrap() > game.collision_mesh.root.aabb().min().x as isize);
        assert!(*x_locs.iter().max().unwrap() < game.collision_mesh.root.aabb().max().x as isize);
    }

    #[test]
    fn basic_predict_throwback() {
        let (game, mut ball) = load_standard_throwback();

        assert_eq!(ball.time as i64, 0);
        assert_eq!(ball.location.x as i64, 0);
        assert_eq!(ball.location.y as i64, 0);
        assert_eq!(ball.location.z as i64, 100);
        assert_eq!(ball.velocity.x as i64, 0);
        assert_eq!(ball.velocity.y as i64, 0);
        assert_eq!(ball.velocity.z as i64, 0);
        assert_eq!(ball.angular_velocity.x as i64, 0);
        assert_eq!(ball.angular_velocity.y as i64, 0);
        assert_eq!(ball.angular_velocity.z as i64, 0);
        assert_eq!(ball.radius as i64, 91);

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

        dbg!(game.collision_mesh.root.aabb());

        for _ in 0..iters {
            ball.update(
                0.,
                Vec3A::new(
                    rng.gen_range(-3900.0..3900.),
                    rng.gen_range(-5000.0..5000.),
                    rng.gen_range(100.0..1900.),
                ),
                Vec3A::new(
                    rng.gen_range(-2000.0..2000.),
                    rng.gen_range(-2000.0..2000.),
                    rng.gen_range(-2000.0..2000.),
                ),
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

        assert!(*z_locs.iter().min().unwrap() > game.collision_mesh.root.aabb().min().z as isize);
        assert!(*z_locs.iter().max().unwrap() < game.collision_mesh.root.aabb().max().z as isize);

        assert!(*y_locs.iter().min().unwrap() > game.collision_mesh.root.aabb().min().y as isize);
        assert!(*y_locs.iter().max().unwrap() < game.collision_mesh.root.aabb().max().y as isize);

        assert!(*x_locs.iter().min().unwrap() > game.collision_mesh.root.aabb().min().x as isize);
        assert!(*x_locs.iter().max().unwrap() < game.collision_mesh.root.aabb().max().x as isize);
    }

    #[test]
    fn gamemode_standard() {
        let (game, ball) = load_standard();

        // test all the default values to make sure they're proper

        assert_eq!(game.gravity.x as i64, 0);
        assert_eq!(game.gravity.y as i64, 0);
        assert_eq!(game.gravity.z as i64, -650);

        dbg!(game.collision_mesh.root.aabb());

        assert_eq!(game.collision_mesh.num_leaves, 8028);

        assert_eq!(ball.time as i64, 0);
        assert_eq!(ball.location.x as i64, 0);
        assert_eq!(ball.location.y as i64, 0);
        assert_eq!(ball.location.z as i64, 100);
        assert_eq!(ball.velocity.x as i64, 0);
        assert_eq!(ball.velocity.y as i64, 0);
        assert_eq!(ball.velocity.z as i64, 0);
        assert_eq!(ball.angular_velocity.x as i64, 0);
        assert_eq!(ball.angular_velocity.y as i64, 0);
        assert_eq!(ball.angular_velocity.z as i64, 0);
        assert_eq!(ball.radius as i64, 91);
    }

    #[test]
    fn gamemode_hoops() {
        let (game, ball) = load_hoops();

        // test all the default values to make sure they're proper

        assert_eq!(game.gravity.x as i64, 0);
        assert_eq!(game.gravity.y as i64, 0);
        assert_eq!(game.gravity.z as i64, -650);

        dbg!(game.collision_mesh.root.aabb());

        assert_eq!(game.collision_mesh.num_leaves, 15732);

        assert_eq!(ball.time as i64, 0);
        assert_eq!(ball.location.x as i64, 0);
        assert_eq!(ball.location.y as i64, 0);
        assert_eq!(ball.location.z as i64, 106);
        assert_eq!(ball.velocity.x as i64, 0);
        assert_eq!(ball.velocity.y as i64, 0);
        assert_eq!(ball.velocity.z as i64, 0);
        assert_eq!(ball.angular_velocity.x as i64, 0);
        assert_eq!(ball.angular_velocity.y as i64, 0);
        assert_eq!(ball.angular_velocity.z as i64, 0);
        assert_eq!(ball.radius as i64, 96);
    }

    #[test]
    fn gamemode_dropshot() {
        let (game, ball) = load_dropshot();

        // test all the default values to make sure they're proper

        assert_eq!(game.gravity.x as i64, 0);
        assert_eq!(game.gravity.y as i64, 0);
        assert_eq!(game.gravity.z as i64, -650);

        dbg!(game.collision_mesh.root.aabb());

        assert_eq!(game.collision_mesh.num_leaves, 3616);

        assert_eq!(ball.time as i64, 0);
        assert_eq!(ball.location.x as i64, 0);
        assert_eq!(ball.location.y as i64, 0);
        assert_eq!(ball.location.z as i64, 110);
        assert_eq!(ball.velocity.x as i64, 0);
        assert_eq!(ball.velocity.y as i64, 0);
        assert_eq!(ball.velocity.z as i64, 0);
        assert_eq!(ball.angular_velocity.x as i64, 0);
        assert_eq!(ball.angular_velocity.y as i64, 0);
        assert_eq!(ball.angular_velocity.z as i64, 0);
        assert_eq!(ball.radius as i64, 100);
    }

    #[test]
    fn gamemode_throwback_standard() {
        let (game, ball) = load_standard_throwback();

        // test all the default values to make sure they're proper

        assert_eq!(game.gravity.x as i64, 0);
        assert_eq!(game.gravity.y as i64, 0);
        assert_eq!(game.gravity.z as i64, -650);

        dbg!(&game.collision_mesh.root.aabb());
        if let Node::Branch(branch) = &game.collision_mesh.root {
            dbg!(branch.left.aabb());
            dbg!(branch.right.aabb());
        }

        assert_eq!(game.collision_mesh.num_leaves, 9272);

        assert_eq!(ball.time as i64, 0);
        assert_eq!(ball.location.x as i64, 0);
        assert_eq!(ball.location.y as i64, 0);
        assert_eq!(ball.location.z as i64, 100);
        assert_eq!(ball.velocity.x as i64, 0);
        assert_eq!(ball.velocity.y as i64, 0);
        assert_eq!(ball.velocity.z as i64, 0);
        assert_eq!(ball.angular_velocity.x as i64, 0);
        assert_eq!(ball.angular_velocity.y as i64, 0);
        assert_eq!(ball.angular_velocity.z as i64, 0);
        assert_eq!(ball.radius as i64, 91);
    }

    fn recurse_bvhnode(node: &Node, depth: usize, max_depth: &mut usize) {
        if depth > *max_depth {
            *max_depth = depth;
        }

        if let Node::Branch(branch) = node {
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
