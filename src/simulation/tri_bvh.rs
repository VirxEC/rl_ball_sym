//! Tools for calculating collisions between objects and the Rocket League field.

use super::{
    bvh::{global_aabb, Branch, Leaf, Node},
    geometry::{Aabb, Ray, Sphere, Tri},
    morton::Morton,
};
use combo_vec::{re_arr, ReArr};
use std::convert::Into;

/// A bounding volume hierarchy.
#[derive(Clone, Debug)]
pub struct TriangleBvh {
    /// The root of the BVH.
    root: Node,
    /// The primitive triangles that the BVH is built from.
    primitives: Vec<Tri>,
}

impl TriangleBvh {
    #[must_use]
    /// Creates a new BVH from a list of primitives.
    pub fn new(primitives: Vec<Tri>) -> Self {
        let aabbs: Vec<Aabb> = primitives.iter().copied().map(Into::into).collect();
        let morton = Morton::new(global_aabb(&aabbs));

        let mut sorted_leaves: Vec<_> = aabbs.iter().map(|aabb| morton.get_code(aabb)).enumerate().collect();
        radsort::sort_by_key(&mut sorted_leaves, |leaf| leaf.1);

        let root = Self::generate_hierarchy(&sorted_leaves, &aabbs);

        Self { root, primitives }
    }

    fn generate_hierarchy(sorted_leaves: &[(usize, u64)], aabbs: &[Aabb]) -> Node {
        // If we're dealing with a single object, return the leaf node
        if sorted_leaves.len() == 1 {
            let idx = sorted_leaves[0].0;
            let leaf = Leaf::new(aabbs[idx], idx);
            return Node::Leaf(leaf);
        }

        // Determine where to split the range
        let mid = sorted_leaves.len() / 2;
        let (first_half, last_half) = sorted_leaves.split_at(mid);

        // Process the resulting sub-ranges recursively
        let right = Self::generate_hierarchy(first_half, aabbs);
        let left = Self::generate_hierarchy(last_half, aabbs);

        Node::branch(right, left)
    }

    #[must_use]
    /// Returns a Vec of the collision triangles intersecting with the `query_object`.
    pub fn intersect(&self, query_object: Sphere) -> Vec<Ray> {
        const STACK: ReArr<&Branch, 12> = re_arr![];

        let mut hits = Vec::with_capacity(4);

        // Traverse nodes starting from the root
        // If the node is a leaf, check for intersection
        let mut node = match &self.root {
            Node::Branch(branch) => branch,
            Node::Leaf(leaf) => {
                if let Some(contact) = self.primitives[leaf.idx].intersect_sphere(query_object) {
                    hits.push(contact);
                }
                return hits;
            }
        };

        let query_box = Aabb::from(query_object);

        // Allocate traversal stack from thread-local memory
        let mut stack = STACK;

        // Check each child node for overlap.
        loop {
            let mut traverse_left = None;

            let left = node.left.as_ref();
            if left.aabb().intersect_self(&query_box) {
                match left {
                    Node::Leaf(left) => {
                        if let Some(contact) = self.primitives[left.idx].intersect_sphere(query_object) {
                            hits.push(contact);
                        }
                    }
                    Node::Branch(left) => traverse_left = Some(left),
                }
            }

            let right = node.right.as_ref();
            if right.aabb().intersect_self(&query_box) {
                match right {
                    Node::Leaf(right) => {
                        if let Some(contact) = self.primitives[right.idx].intersect_sphere(query_object) {
                            hits.push(contact);
                        }
                    }
                    Node::Branch(right) => {
                        node = traverse_left.map_or(right, |branch| {
                            stack.push(right);
                            branch
                        });
                        continue;
                    }
                }
            }

            node = if let Some(branch) = traverse_left {
                branch
            } else {
                let Some(next_node) = stack.pop() else {
                    break;
                };

                next_node
            };
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

        let mut contact_points = tris_hit.into_iter();

        let Some(mut contact_point) = contact_points.next() else {
            return None;
        };

        let mut count = 1.;

        for point in contact_points {
            count += 1.;
            contact_point += point;
        }

        contact_point.start /= count;
        contact_point.direction = contact_point.direction.normalize_or_zero();

        Some(contact_point)
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
            .map(|map| Tri::new([verts[map[0]], verts[map[1]], verts[map[2]]]))
            .collect()
    }

    #[test]
    fn test_bvh_build() {
        let triangles = generate_tris();

        let _ = black_box(TriangleBvh::new(triangles));
    }

    #[test]
    fn test_bvh_collide_count() {
        let triangles = generate_tris();

        let bvh = TriangleBvh::new(triangles);
        {
            // Sphere hits nothing
            let sphere = Sphere::new(Vec3A::new(0., 0., 1022.), 100.);
            let hits = bvh.intersect(sphere);
            assert_eq!(hits.len(), 0);
        }
        {
            // Sphere hits one Tri
            let sphere = Sphere::new(Vec3A::new(4096. / 2., 5120. / 2., 99.9), 100.);
            let hits = bvh.intersect(sphere);

            assert_eq!(hits.len(), 1);
        }
        {
            // Middle of two Tris
            let sphere = Sphere::new(Vec3A::ZERO, 100.);
            let hits = bvh.intersect(sphere);

            assert_eq!(hits.len(), 2);
        }
        {
            // Sphere is in a corner
            let sphere = Sphere::new(Vec3A::new(4096., 5120., 0.), 100.);
            let hits = bvh.intersect(sphere);

            assert_eq!(hits.len(), 5);
        }
    }

    #[test]
    fn test_bvh_collide() {
        let triangles = generate_tris();

        let bvh = TriangleBvh::new(triangles);

        {
            // Sphere hits nothing
            let sphere = Sphere::new(Vec3A::new(0., 0., 1022.), 100.);

            let ray = bvh.intersect(sphere);

            assert!(ray.is_empty());
        }
        {
            // Sphere hits one Tri
            let sphere = Sphere::new(Vec3A::new(4096. / 2., 5120. / 2., 99.), 100.);

            let rays = bvh.intersect(sphere);
            assert!(!rays.is_empty());

            let ray = rays[0];

            assert!((ray.start.x - 2048.).abs() < f32::EPSILON);
            assert!((ray.start.y - 2560.).abs() < f32::EPSILON);
            assert!((ray.start.z - 0.).abs() < f32::EPSILON);
            assert!((ray.direction.x - 0.0).abs() < f32::EPSILON);
            assert!((ray.direction.y - 0.0).abs() < f32::EPSILON);
            assert!((ray.direction.z - 1.0).abs() < f32::EPSILON);
        }
        {
            // Middle of two Tris
            let sphere = Sphere::new(Vec3A::Z, 100.);

            let rays = bvh.intersect(sphere);
            assert!(!rays.is_empty());

            let ray = rays[0];

            assert!((ray.start.x - 0.0).abs() < f32::EPSILON);
            assert!((ray.start.y - 0.0).abs() < f32::EPSILON);
            assert!((ray.start.z - 0.0).abs() < f32::EPSILON);
            assert!((ray.direction.x - 0.0).abs() < f32::EPSILON);
            assert!((ray.direction.y - 0.0).abs() < f32::EPSILON);
            assert!((ray.direction.z - 1.0).abs() < f32::EPSILON);
        }
        {
            // Sphere is in a corner
            let sphere = Sphere::new(Vec3A::new(4095., 5119., 5.), 100.);

            let rays = bvh.intersect(sphere);
            assert!(!rays.is_empty());

            let first_ray = rays[0];
            dbg!(first_ray);
            assert!((first_ray.start.x - 4093.8838).abs() < f32::EPSILON);
            assert!((first_ray.start.y - 5120.).abs() < f32::EPSILON);
            assert!((first_ray.start.z - 0.528_076_2).abs() < f32::EPSILON);
        }
    }

    #[test]
    fn is_collision_ray_finite() {
        let triangles = generate_tris();

        let bvh = TriangleBvh::new(triangles);

        {
            let sphere = Sphere::new(Vec3A::new(0., 0., 92.15 - f32::EPSILON), 92.15);
            let ray = bvh.intersect(sphere);

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
        let num_slices = 6 * 120 * iters;
        let mut rng = rand::thread_rng();

        let mut x_locs = Vec::with_capacity(num_slices);
        let mut y_locs = Vec::with_capacity(num_slices);
        let mut z_locs = Vec::with_capacity(num_slices);

        dbg!(game.triangle_collisions.root.aabb());

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

        assert!(*z_locs.iter().min().unwrap() > game.triangle_collisions.root.aabb().min().z as isize);
        assert!(*z_locs.iter().max().unwrap() < game.triangle_collisions.root.aabb().max().z as isize);

        assert!(*y_locs.iter().min().unwrap() > game.triangle_collisions.root.aabb().min().y as isize);
        assert!(*y_locs.iter().max().unwrap() < game.triangle_collisions.root.aabb().max().y as isize);

        assert!(*x_locs.iter().min().unwrap() > game.triangle_collisions.root.aabb().min().x as isize);
        assert!(*x_locs.iter().max().unwrap() < game.triangle_collisions.root.aabb().max().x as isize);
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

        dbg!(game.triangle_collisions.root.aabb());

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

        assert!(*z_locs.iter().min().unwrap() > game.triangle_collisions.root.aabb().min().z as isize);
        assert!(*z_locs.iter().max().unwrap() < game.triangle_collisions.root.aabb().max().z as isize);

        assert!(*y_locs.iter().min().unwrap() > game.triangle_collisions.root.aabb().min().y as isize);
        assert!(*y_locs.iter().max().unwrap() < game.triangle_collisions.root.aabb().max().y as isize);

        assert!(*x_locs.iter().min().unwrap() > game.triangle_collisions.root.aabb().min().x as isize);
        assert!(*x_locs.iter().max().unwrap() < game.triangle_collisions.root.aabb().max().x as isize);
    }

    #[test]
    fn gamemode_standard() {
        let (game, ball) = load_standard();

        // test all the default values to make sure they're proper

        assert_eq!(game.gravity.x as i64, 0);
        assert_eq!(game.gravity.y as i64, 0);
        assert_eq!(game.gravity.z as i64, -650);

        dbg!(game.triangle_collisions.root.aabb());

        assert_eq!(game.triangle_collisions.primitives.len(), 8028);

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

        dbg!(game.triangle_collisions.root.aabb());

        assert_eq!(game.triangle_collisions.primitives.len(), 15732);

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

        dbg!(game.triangle_collisions.root.aabb());

        assert_eq!(game.triangle_collisions.primitives.len(), 3616);

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

        dbg!(&game.triangle_collisions.root.aabb());
        if let Node::Branch(branch) = &game.triangle_collisions.root {
            dbg!(branch.left.aabb());
            dbg!(branch.right.aabb());
        }

        assert_eq!(game.triangle_collisions.primitives.len(), 9272);

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

        recurse_bvhnode(&game.triangle_collisions.root, 0, &mut max_depth);
        assert_eq!(max_depth, 14);
    }

    #[test]
    fn hierarchy_depth_hoops() {
        let (game, _) = load_hoops();
        let mut max_depth = 0;

        recurse_bvhnode(&game.triangle_collisions.root, 0, &mut max_depth);
        assert_eq!(max_depth, 14);
    }

    #[test]
    fn hierarchy_depth_standard() {
        let (game, _) = load_standard();
        let mut max_depth = 0;

        recurse_bvhnode(&game.triangle_collisions.root, 0, &mut max_depth);
        assert_eq!(max_depth, 13);
    }

    #[test]
    fn hierarchy_depth_dropshot() {
        let (game, _) = load_dropshot();
        let mut max_depth = 0;

        recurse_bvhnode(&game.triangle_collisions.root, 0, &mut max_depth);
        assert_eq!(max_depth, 12);
    }
}
