//! Tools for calculating collisions between objects and the Rocket League field.

use super::{
    bvh::{Branch, Node, NodeType, global_aabb},
    game::Constraints,
    geometry::{Aabb, Contact, Hits, Sphere, Tri},
    morton::Morton,
};
use arrayvec::ArrayVec;

/// A bounding volume hierarchy built from only triangles.
#[derive(Clone, Debug)]
pub struct TriangleBvh {
    /// All the nodes in the bvh (branches & leaves)
    pub nodes: Box<[Node]>,
}

impl TriangleBvh {
    #[must_use]
    /// Creates a new BVH from a list of primitives.
    pub fn new(aabbs: &[Aabb]) -> Self {
        assert!(
            aabbs.len() > 1,
            "Cannot build a BVH with fewer than 2 triangles."
        );
        assert!(
            u16::try_from(aabbs.len()).is_ok(),
            "The number of triangles exceeds the maximum supported by the BVH ({}).",
            u16::MAX
        );

        let morton = Morton::new(global_aabb(aabbs));
        let codes: Vec<_> = aabbs.iter().map(|aabb| morton.get_code(aabb)).collect();

        // The assert at the beginning of this function ensures that the number of triangles
        // fits within the u16 range, so we can disable the clippy lint for casting truncation here.
        #[allow(clippy::cast_possible_truncation)]
        let mut sorted_leaves: Vec<_> = (0..codes.len()).map(|i| i as u16).collect();
        radsort::sort_by_key(&mut sorted_leaves, |idx| codes[*idx as usize]);

        let mut nodes = Vec::with_capacity(2 * aabbs.len() + 1);
        nodes.extend(
            aabbs
                .iter()
                .enumerate()
                .map(|(idx, aabb)| Node::leaf(*aabb, idx)),
        );
        Self::generate_hierarchy(&mut nodes, &sorted_leaves);

        Self {
            nodes: nodes.into_boxed_slice(),
        }
    }

    fn generate_hierarchy(nodes: &mut Vec<Node>, sorted_leaves: &[u16]) -> usize {
        debug_assert!(!sorted_leaves.is_empty());
        match sorted_leaves.len() {
            1 => {
                // If we're dealing with a single object, return the leaf node
                sorted_leaves[0] as usize
            }
            2 => {
                // Special case for two leaves, create a branch directly
                let right = sorted_leaves[0] as usize;
                let left = sorted_leaves[1] as usize;

                let num_nodes = nodes.len();
                let aabb = nodes[right].aabb + nodes[left].aabb;
                nodes.push(Node::branch(aabb, right, left));

                num_nodes
            }
            i => {
                let (first_half, last_half) = sorted_leaves.split_at(i / 2);

                // Process the resulting sub-ranges recursively
                let right = Self::generate_hierarchy(nodes, first_half);
                let left = Self::generate_hierarchy(nodes, last_half);

                let num_nodes = nodes.len();
                let aabb = nodes[right].aabb + nodes[left].aabb;
                nodes.push(Node::branch(aabb, right, left));

                num_nodes
            }
        }
    }

    #[must_use]
    pub fn get_child_bvh(&self, aabbs: &[Aabb], query_box: Aabb) -> Option<Self> {
        let mut stack = ArrayVec::<&Branch, 16>::new();
        match &self.nodes[self.nodes.len() - 1].node_type {
            NodeType::Branch(node) => stack.push(node),
            NodeType::Leaf { idx: _ } => unreachable!(),
        }

        let mut leaves = Vec::with_capacity(64);

        // Check each child node for overlap.
        while let Some(node) = stack.pop() {
            let right = &self.nodes[node.right];
            let left = &self.nodes[node.left];

            if right.aabb.intersect_self(&query_box) {
                match &right.node_type {
                    NodeType::Leaf { idx } => leaves.push(*idx),
                    NodeType::Branch(right) => stack.push(right),
                }
            }

            if left.aabb.intersect_self(&query_box) {
                match &left.node_type {
                    NodeType::Leaf { idx } => leaves.push(*idx),
                    NodeType::Branch(left) => stack.push(left),
                }
            }
        }

        if leaves.is_empty() {
            None
        } else {
            let mut nodes = Vec::with_capacity(2 * leaves.len() + 1);
            nodes.extend(leaves.into_iter().map(|idx| Node::leaf(aabbs[idx], idx)));

            // due to the nature of our traversal,
            // the leaves should already be sorted
            #[allow(clippy::cast_possible_truncation)]
            let leaves: Vec<_> = (0..nodes.len() as u16).collect();
            Self::generate_hierarchy(&mut nodes, &leaves);

            Some(Self {
                nodes: nodes.into_boxed_slice(),
            })
        }
    }

    #[must_use]
    /// Returns a Vec of the collision rays and triangle normals from the triangles intersecting with the `query_object`.
    pub fn collide(
        &self,
        primitives: &[Tri],
        query_object: Sphere,
    ) -> Option<ArrayVec<Contact, { Constraints::MAX_CONTACTS }>> {
        let query_box = Aabb::from(query_object);

        let mut hits = const { Hits::new() };
        let mut stack = ArrayVec::<&Branch, 16>::new();
        match &self.nodes[self.nodes.len() - 1].node_type {
            NodeType::Branch(node) => stack.push(node),
            NodeType::Leaf { idx } => {
                return if self.nodes[self.nodes.len() - 1]
                    .aabb
                    .intersect_self(&query_box)
                {
                    primitives[*idx].intersect_sphere(query_object).map(|node| {
                        hits.push(node);
                        hits.inner()
                    })
                } else {
                    None
                };
            }
        }

        // Check each child node for overlap.
        while let Some(node) = stack.pop() {
            let right = &self.nodes[node.right];
            let left = &self.nodes[node.left];

            if right.aabb.intersect_self(&query_box) {
                match &right.node_type {
                    NodeType::Leaf { idx } => {
                        if let Some(info) = primitives[*idx].intersect_sphere(query_object) {
                            hits.push(info);
                        }
                    }
                    NodeType::Branch(right) => stack.push(right),
                }
            }

            if left.aabb.intersect_self(&query_box) {
                match &left.node_type {
                    NodeType::Leaf { idx } => {
                        if let Some(info) = primitives[*idx].intersect_sphere(query_object) {
                            hits.push(info);
                        }
                    }
                    NodeType::Branch(left) => stack.push(left),
                }
            }
        }

        let hits = hits.inner();
        if hits.is_empty() { None } else { Some(hits) }
    }
}

#[cfg(test)]
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
mod test {
    use super::*;
    use crate::{load_dropshot, load_hoops, load_standard, load_standard_throwback};
    use glam::Vec3A;
    use rand::Rng;
    use std::hint::black_box;

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

    fn generate_tris() -> Box<[Tri]> {
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
        let aabbs: Vec<Aabb> = triangles.iter().map(Into::into).collect();

        let _ = black_box(TriangleBvh::new(&aabbs));
    }

    #[test]
    fn test_bvh_collide_count() {
        let triangles = generate_tris();
        let aabbs: Vec<Aabb> = triangles.iter().map(Into::into).collect();
        let bvh = TriangleBvh::new(&aabbs);

        {
            // Sphere hits nothing
            let sphere = Sphere::new(Vec3A::new(0., 0., 1022.), 100.);
            let hits = bvh.collide(&triangles, sphere);
            assert!(hits.is_none());
        }
        {
            // Sphere hits one Tri
            let sphere = Sphere::new(Vec3A::new(4096. / 2., 5120. / 2., 99.9), 100.);
            let hits = bvh.collide(&triangles, sphere);

            assert_eq!(hits.unwrap().len(), 1);
        }
        {
            // Middle of two Tris
            let sphere = Sphere::new(Vec3A::ZERO, 100.);
            let hits = bvh.collide(&triangles, sphere);

            assert_eq!(hits.unwrap().len(), 2);
        }
        {
            // Sphere is in a corner
            let sphere = Sphere::new(Vec3A::new(4096., 5120., 0.), 100.);
            let hits = bvh.collide(&triangles, sphere);

            assert_eq!(hits.unwrap().len(), 4);
        }
    }

    #[test]
    fn test_bvh_collide() {
        let triangles = generate_tris();
        let aabbs: Vec<Aabb> = triangles.iter().map(Into::into).collect();
        let bvh = TriangleBvh::new(&aabbs);

        {
            // Sphere hits nothing
            let sphere = Sphere::new(Vec3A::new(0., 0., 1022.), 100.);

            let ray = bvh.collide(&triangles, sphere);

            assert!(ray.is_none());
        }
        {
            // Sphere hits one Tri
            let center = Vec3A::new(4096. / 2., 5120. / 2., 99.);
            let sphere = Sphere::new(center, 100.);

            let rays = bvh.collide(&triangles, sphere).unwrap();
            assert!(!rays.is_empty());

            let position = rays[0].local_position + center;
            dbg!(position);

            assert!((position.x - 2048.).abs() < f32::EPSILON);
            assert!((position.y - 2560.).abs() < f32::EPSILON);
            assert!((position.z - -1.).abs() < f32::EPSILON);
            assert!((rays[0].triangle_normal.x - 0.0).abs() < f32::EPSILON);
            assert!((rays[0].triangle_normal.y - 0.0).abs() < f32::EPSILON);
            assert!((rays[0].triangle_normal.z - 1.0).abs() < f32::EPSILON);
        }
        {
            // Middle of two Tris
            let center = Vec3A::Z;
            let sphere = Sphere::new(center, 2.);

            let rays = bvh.collide(&triangles, sphere).unwrap();
            assert!(!rays.is_empty());

            let position = rays[0].local_position + center;
            dbg!(position);

            assert!((position.x - 0.0).abs() < f32::EPSILON);
            assert!((position.y - 0.0).abs() < f32::EPSILON);
            assert!((position.z - -1.0).abs() < f32::EPSILON);
            assert!((rays[0].triangle_normal.x - 0.0).abs() < f32::EPSILON);
            assert!((rays[0].triangle_normal.y - 0.0).abs() < f32::EPSILON);
            assert!((rays[0].triangle_normal.z - 1.0).abs() < f32::EPSILON);
        }
        {
            // Sphere is in a corner
            let center = Vec3A::new(4095., 5119., 5.);
            let sphere = Sphere::new(center, 6.);

            let rays = bvh.collide(&triangles, sphere).unwrap();
            assert!(!rays.is_empty());

            let position = rays[0].local_position + center;
            dbg!(position);

            assert!((position.x - 4095.).abs() < f32::EPSILON);
            assert!((position.y - 5119.).abs() < f32::EPSILON);
            assert!((position.z - -1.).abs() < f32::EPSILON);

            let ray_normal = rays
                .iter()
                .skip(1)
                .fold(rays[0].triangle_normal, |mut acc, ray| {
                    acc += ray.triangle_normal;
                    acc
                });
            let direction = ray_normal.normalize();
            dbg!(direction);

            assert!((direction.x - -0.816_496_55).abs() < f32::EPSILON);
            assert!((direction.y - -0.408_248_28).abs() < f32::EPSILON);
            assert!((direction.z - 0.408_248_28).abs() < f32::EPSILON);
        }
    }

    #[test]
    fn is_collision_ray_finite() {
        let triangles = generate_tris();
        let aabbs: Vec<Aabb> = triangles.iter().map(Into::into).collect();
        let bvh = TriangleBvh::new(&aabbs);

        {
            let sphere = Sphere::new(Vec3A::new(0., 0., 92.15 - f32::EPSILON), 92.15);
            let ray = bvh.collide(&triangles, sphere).unwrap();

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
        assert_eq!(ball.radius() as i64, 91);

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
        let mut rng = rand::rng();

        let mut x_locs = Vec::with_capacity(num_slices);
        let mut y_locs = Vec::with_capacity(num_slices);
        let mut z_locs = Vec::with_capacity(num_slices);

        for _ in 0..iters {
            ball.update(
                0.,
                Vec3A::new(
                    rng.random_range(-3200.0..3200.),
                    rng.random_range(-4500.0..4500.),
                    rng.random_range(100.0..1900.),
                ),
                Vec3A::new(
                    rng.random_range(-2000.0..2000.),
                    rng.random_range(-2000.0..2000.),
                    rng.random_range(-2000.0..2000.),
                ),
                Vec3A::new(
                    rng.random_range(-3.0..3.),
                    rng.random_range(-3.0..3.),
                    rng.random_range(-3.0..3.),
                ),
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

        let root_aabb = game.collider.aabb;
        dbg!(root_aabb);

        assert!(*z_locs.iter().min().unwrap() > root_aabb.min().z as isize);
        assert!(*z_locs.iter().max().unwrap() < root_aabb.max().z as isize);

        assert!(*y_locs.iter().min().unwrap() > root_aabb.min().y as isize);
        assert!(*y_locs.iter().max().unwrap() < root_aabb.max().y as isize);

        assert!(*x_locs.iter().min().unwrap() > root_aabb.min().x as isize);
        assert!(*x_locs.iter().max().unwrap() < root_aabb.max().x as isize);
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
        assert_eq!(ball.radius() as i64, 91);

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
        let mut rng = rand::rng();

        let mut x_locs = Vec::with_capacity(num_slices);
        let mut y_locs = Vec::with_capacity(num_slices);
        let mut z_locs = Vec::with_capacity(num_slices);

        for _ in 0..iters {
            ball.update(
                0.,
                Vec3A::new(
                    rng.random_range(-3900.0..3900.),
                    rng.random_range(-5000.0..5000.),
                    rng.random_range(100.0..1900.),
                ),
                Vec3A::new(
                    rng.random_range(-2000.0..2000.),
                    rng.random_range(-2000.0..2000.),
                    rng.random_range(-2000.0..2000.),
                ),
                Vec3A::new(
                    rng.random_range(-3.0..3.),
                    rng.random_range(-3.0..3.),
                    rng.random_range(-3.0..3.),
                ),
            );

            let ball_prediction = ball.get_ball_prediction_struct(&game);

            for slice in ball_prediction {
                if slice.location.y.abs() > 5120. + slice.radius() {
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

        let root_aabb = game.collider.aabb;
        dbg!(root_aabb);

        assert!(*z_locs.iter().min().unwrap() > root_aabb.min().z as isize);
        assert!(*z_locs.iter().max().unwrap() < root_aabb.max().z as isize);

        assert!(*y_locs.iter().min().unwrap() > root_aabb.min().y as isize);
        assert!(*y_locs.iter().max().unwrap() < root_aabb.max().y as isize);

        assert!(*x_locs.iter().min().unwrap() > root_aabb.min().x as isize);
        assert!(*x_locs.iter().max().unwrap() < root_aabb.max().x as isize);
    }

    #[test]
    fn gamemode_standard() {
        let (game, ball) = load_standard();

        // test all the default values to make sure they're proper

        assert_eq!(game.gravity.x as i64, 0);
        assert_eq!(game.gravity.y as i64, 0);
        assert_eq!(game.gravity.z as i64, -650);

        dbg!(game.collider.aabb);

        assert_eq!(game.collider.primitives.len(), 8028);

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
        assert_eq!(ball.radius() as i64, 91);
    }

    #[test]
    fn gamemode_hoops() {
        let (game, ball) = load_hoops();

        // test all the default values to make sure they're proper

        assert_eq!(game.gravity.x as i64, 0);
        assert_eq!(game.gravity.y as i64, 0);
        assert_eq!(game.gravity.z as i64, -650);

        dbg!(game.collider.aabb);

        assert_eq!(game.collider.primitives.len(), 15732);

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
        assert_eq!(ball.radius() as i64, 96);
    }

    #[test]
    fn gamemode_dropshot() {
        let (game, ball) = load_dropshot();

        // test all the default values to make sure they're proper

        assert_eq!(game.gravity.x as i64, 0);
        assert_eq!(game.gravity.y as i64, 0);
        assert_eq!(game.gravity.z as i64, -650);

        dbg!(game.collider.aabb);

        assert_eq!(game.collider.primitives.len(), 3616);

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
        assert_eq!(ball.radius() as i64, 100);
    }

    #[test]
    fn gamemode_throwback_standard() {
        let (game, ball) = load_standard_throwback();

        // test all the default values to make sure they're proper

        assert_eq!(game.gravity.x as i64, 0);
        assert_eq!(game.gravity.y as i64, 0);
        assert_eq!(game.gravity.z as i64, -650);

        dbg!(game.collider.aabb);

        assert_eq!(game.collider.primitives.len(), 9272);

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
        assert_eq!(ball.radius() as i64, 91);
    }
}
