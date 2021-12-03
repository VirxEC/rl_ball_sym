use super::geometry::{Aabb, Tri};
use super::geometry::{Ray, Sphere};
use super::morton::Morton;
use std::boxed::Box;

#[derive(Clone)]
pub struct BvhNode {
    pub is_terminal: bool,
    pub box_: Aabb,
    pub right: Option<Box<BvhNode>>,
    pub left: Option<Box<BvhNode>>,
    pub primitive: Option<Tri>,
    pub morton: Option<u64>,
}

impl Default for BvhNode {
    fn default() -> Self {
        Self {
            is_terminal: false,
            box_: Aabb::default(),
            right: None,
            left: None,
            primitive: None,
            morton: None,
        }
    }
}

impl BvhNode {
    pub fn branch(right: Box<BvhNode>, left: Box<BvhNode>) -> Box<Self> {
        Box::new(Self {
            is_terminal: false,
            box_: right.box_.add(&left.box_),
            right: Some(right),
            left: Some(left),
            primitive: None,
            morton: None,
        })
    }

    pub fn leaf(primitive: Tri, box_: Aabb, morton_code: u64) -> Box<Self> {
        Box::new(Self {
            is_terminal: true,
            box_,
            right: None,
            left: None,
            primitive: Some(primitive),
            morton: Some(morton_code),
        })
    }
}

// BVH stands for "Bounding Volume Hierarchy"
#[derive(Clone)]
pub struct Bvh {
    pub global_box: Aabb,
    pub num_leaves: u64,
    pub root: Box<BvhNode>,
}

fn global_aabb(boxes: &[Aabb]) -> Aabb {
    let mut global_box = boxes[0];

    for b in boxes {
        global_box = global_box.add(b);
    }

    global_box
}

impl Default for Bvh {
    fn default() -> Self {
        Self {
            global_box: Aabb::default(),
            num_leaves: 0,
            root: Box::new(BvhNode::default()),
        }
    }
}

impl Bvh {
    pub fn from(primitives: &[Tri]) -> Self {
        let num_leaves = primitives.len();

        let mut boxes: Vec<Aabb> = Vec::with_capacity(num_leaves);
        for primitive in primitives {
            boxes.push(Aabb::from_tri(primitive));
        }

        let global_box = global_aabb(&boxes);

        let morton = Morton::from(&global_box);
        let mut sorted_leaves: Vec<Box<BvhNode>> = Vec::with_capacity(num_leaves);

        for (i, box_) in boxes.iter().enumerate() {
            sorted_leaves.push(BvhNode::leaf(primitives[i], *box_, morton.get_code(box_)));
        }

        sorted_leaves.sort_unstable_by_key(|leaf| leaf.morton);

        let root = Bvh::generate_hierarchy(&sorted_leaves, 0, num_leaves - 1);

        Self {
            global_box,
            num_leaves: num_leaves as u64,
            root,
        }
    }

    fn generate_hierarchy(sorted_leaves: &[Box<BvhNode>], first: usize, last: usize) -> Box<BvhNode> {
        // If we're dealing with a single object, return the leaf node
        if first == last {
            return sorted_leaves[first].clone();
        }

        // Determine where to split the range

        let split = first + ((last - first) / 2);

        // Process the resulting sub-ranges recursively

        let right = Bvh::generate_hierarchy(sorted_leaves, first, split);
        let left = Bvh::generate_hierarchy(sorted_leaves, split + 1, last);

        BvhNode::branch(right, left)
    }

    pub fn intersect(&self, query_object: &Sphere) -> Vec<Tri> {
        let query_box: Aabb = Aabb::from_sphere(query_object);

        let mut hits = Vec::with_capacity(16);

        // Allocate traversal stack from thread-local memory,
        // and push NULL to indicate that there are no postponed nodes.
        let mut stack: Vec<&BvhNode> = Vec::with_capacity(32);

        // Traverse nodes starting from the root.
        let mut node = &*self.root;
        // Check each child node for overlap.
        loop {
            // We must save the right node to a variable
            // There's the potential for node to be overwritten
            let right_og = node.right.as_deref();
            
            let mut traverse_left = false;
            if let Some(left) = node.left.as_deref() {
                if left.box_.intersect_self(&query_box) {
                    match left.primitive {
                        Some(left_tri) => {
                            if left_tri.intersect_sphere(query_object) {
                                hits.push(left_tri);
                            }
                        }
                        None => {
                            traverse_left = true;
                            node = left;
                        }
                    }
                }
            }

            let mut traverse_right = false;
            if let Some(right) = right_og {
                if right.box_.intersect_self(&query_box) {
                    match right.primitive {
                        Some(right_tri) => {
                            if right_tri.intersect_sphere(query_object) {
                                hits.push(right_tri);
                            }
                        }
                        None => {
                            traverse_right = true;

                            if traverse_left {
                                stack.push(right);
                            } else {
                                node = right;
                            }
                        }
                    }
                }
            }

            if !(traverse_left || traverse_right) {
                match stack.pop() {
                    Some(n) => node = n,
                    None => break,
                }
            }
        }

        hits
    }

    pub fn collide(&self, s: &Sphere) -> Option<Ray> {
        let mut contact_point = Ray::default();
        let mut count = 0;

        let tris_hit = self.intersect(s);

        for tri in tris_hit {
            let p = tri.center();
            let n = tri.unit_normal();

            let separation = (s.center - p).dot(&n);
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
        contact_point.direction = contact_point.direction.normalize();

        Some(contact_point)
    }
}

#[cfg(test)]
mod test {
    use criterion::black_box;
    use vvec3::Vec3;

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
            Aabb {
                min: Vec3::new(MIN_X, 0.0, 0.0),
                max: Vec3::new(0.0, 0.0, 0.0),
            },
            Aabb {
                min: Vec3::new(0.0, MIN_Y, 0.0),
                max: Vec3::new(0.0, 0.0, 0.0),
            },
            Aabb {
                min: Vec3::new(0.0, 0.0, MIN_Z),
                max: Vec3::new(0.0, 0.0, 0.0),
            },
            Aabb {
                min: Vec3::new(0.0, 0.0, 0.0),
                max: Vec3::new(MAX_X, 0.0, 0.0),
            },
            Aabb {
                min: Vec3::new(0.0, 0.0, 0.0),
                max: Vec3::new(0.0, MAX_Y, 0.0),
            },
            Aabb {
                min: Vec3::new(0.0, 0.0, 0.0),
                max: Vec3::new(0.0, 0.0, MAX_Z),
            },
        ];

        let global = global_aabb(&bounding_boxes);

        assert!((global.min.x - MIN_X).abs() < f32::EPSILON);
        assert!((global.min.y - MIN_Y).abs() < f32::EPSILON);
        assert!((global.min.z - MIN_Z).abs() < f32::EPSILON);
        assert!((global.max.x - MAX_X).abs() < f32::EPSILON);
        assert!((global.max.y - MAX_Y).abs() < f32::EPSILON);
        assert!((global.max.z - MAX_Z).abs() < f32::EPSILON);
    }

    #[test]
    fn global_bounding_box_min() {
        let bounding_boxes = vec![
            Aabb {
                min: Vec3::new(MIN_X, MIN_Y, MIN_Z),
                max: Vec3::new(MIN_X, MIN_Y, MIN_Z),
            },
            Aabb {
                min: Vec3::new(MIN_X, MIN_Y, MIN_Z),
                max: Vec3::new(MIN_X, MIN_Y, MIN_Z),
            },
        ];
        let global = global_aabb(&bounding_boxes);

        assert!((global.min.x - MIN_X).abs() < f32::EPSILON);
        assert!((global.min.y - MIN_Y).abs() < f32::EPSILON);
        assert!((global.min.z - MIN_Z).abs() < f32::EPSILON);
        assert!((global.max.x - MIN_X).abs() < f32::EPSILON);
        assert!((global.max.y - MIN_Y).abs() < f32::EPSILON);
        assert!((global.max.z - MIN_Z).abs() < f32::EPSILON);
    }

    #[test]
    fn global_bounding_box_max() {
        let bounding_boxes = vec![
            Aabb {
                min: Vec3::new(MAX_X, MAX_Y, MAX_Z),
                max: Vec3::new(MAX_X, MAX_Y, MAX_Z),
            },
            Aabb {
                min: Vec3::new(MAX_X, MAX_Y, MAX_Z),
                max: Vec3::new(MAX_X, MAX_Y, MAX_Z),
            },
        ];
        let global = global_aabb(&bounding_boxes);

        assert!((global.min.x - MAX_X).abs() < f32::EPSILON);
        assert!((global.min.y - MAX_Y).abs() < f32::EPSILON);
        assert!((global.min.z - MAX_Z).abs() < f32::EPSILON);
        assert!((global.max.x - MAX_X).abs() < f32::EPSILON);
        assert!((global.max.y - MAX_Y).abs() < f32::EPSILON);
        assert!((global.max.z - MAX_Z).abs() < f32::EPSILON);
    }

    fn generate_tris() -> Vec<Tri> {
        static VERT_MAP: &[[usize; 3]; 12] = &[[0, 1, 2], [3, 2, 1], [7, 5, 6], [4, 6, 5], [0, 2, 4], [6, 4, 2], [7, 3, 5], [1, 5, 3], [0, 4, 1], [5, 1, 4], [7, 6, 3], [2, 3, 6]];
        let verts = &[Vec3::new(-5000.0, -5000.0, -5000.0), Vec3::new(-5000.0, -5000.0, 5000.0), Vec3::new(-5000.0, 5000.0, -5000.0), Vec3::new(-5000.0, 5000.0, 5000.0), Vec3::new(5000.0, -5000.0, -5000.0), Vec3::new(5000.0, -5000.0, 5000.0), Vec3::new(5000.0, 5000.0, -5000.0), Vec3::new(5000.0, 5000.0, 5000.0)];
        VERT_MAP
            .iter()
            .map(|map| {
                let p = [verts[map[0]], verts[map[1]], verts[map[2]]];
                Tri {
                    p,
                }
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
                center: Vec3::default(),
                radius: 100.,
            };
            let hits = bvh.intersect(&sphere);
            assert_eq!(hits.len(), 0);
        }
        {
            // Sphere hits one Tri
            let sphere = Sphere {
                center: Vec3 {
                    x: 4900.,
                    y: 2500.,
                    z: 2500.,
                },
                radius: 100.,
            };
            let hits = bvh.intersect(&sphere);

            assert_eq!(hits.len(), 1);
            let p0 = hits[0].p[0];
            assert!((p0.x - 5000.).abs() < f32::EPSILON);
            assert!((p0.y - 5000.).abs() < f32::EPSILON);
            assert!((p0.z - 5000.).abs() < f32::EPSILON);
            let p1 = hits[0].p[1];
            assert!((p1.x - 5000.).abs() < f32::EPSILON);
            assert!((p1.y - -5000.).abs() < f32::EPSILON);
            assert!((p1.z - 5000.).abs() < f32::EPSILON);
            let p2 = hits[0].p[2];
            assert!((p2.x - 5000.).abs() < f32::EPSILON);
            assert!((p2.y - 5000.).abs() < f32::EPSILON);
            assert!((p2.z - -5000.).abs() < f32::EPSILON);
        }
        {
            // Middle of two Tris
            let sphere = Sphere {
                center: Vec3 {
                    x: 4900.,
                    ..Default::default()
                },
                radius: 100.,
            };
            let hits = bvh.intersect(&sphere);

            assert_eq!(hits.len(), 2);
        }
        {
            // Sphere is in a corner
            let sphere = Sphere {
                center: Vec3 {
                    x: 4900.,
                    y: 4900.,
                    z: -4900.,
                },
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
                center: Vec3::default(),
                radius: 100.,
            };

            let ray = bvh.collide(&sphere);

            assert!(ray.is_none());
        }
        {
            // Sphere hits one Tri
            let sphere = Sphere {
                center: Vec3 {
                    x: 4900.,
                    y: 2500.,
                    z: 2500.,
                },
                radius: 100.,
            };

            let ray = bvh.collide(&sphere);

            assert!(ray.is_some());
            let ray = ray.unwrap();
            assert!((ray.start.x - 5000.).abs() < f32::EPSILON);
            assert!((ray.start.y - 2500.).abs() < f32::EPSILON);
            assert!((ray.start.z - 2500.).abs() < f32::EPSILON);
            assert!((ray.direction.x - 1.0).abs() < f32::EPSILON);
            assert!((ray.direction.y - 0.0).abs() < f32::EPSILON);
            assert!((ray.direction.z - 0.0).abs() < f32::EPSILON);
        }
        {
            // Middle of two Tris
            let sphere = Sphere {
                center: Vec3 {
                    x: 4900.,
                    ..Default::default()
                },
                radius: 100.,
            };

            let ray = bvh.collide(&sphere);

            assert!(ray.is_some());
            let ray = ray.unwrap();
            assert!((ray.start.x - 5000.).abs() < f32::EPSILON);
            assert!((ray.start.y - 0.0).abs() < f32::EPSILON);
            assert!((ray.start.z - 0.0).abs() < f32::EPSILON);
            assert!((ray.direction.x - 1.0).abs() < f32::EPSILON);
            assert!((ray.direction.y - 0.0).abs() < f32::EPSILON);
            assert!((ray.direction.z - 0.0).abs() < f32::EPSILON);
        }
        {
            // Sphere is in a corner
            let sphere = Sphere {
                center: Vec3 {
                    x: 4900.,
                    y: 4900.,
                    z: -4900.,
                },
                radius: 100.,
            };

            let ray = bvh.collide(&sphere);

            assert!(ray.is_some());
            let ray = ray.unwrap();
            assert!((ray.start.x - 4940.).abs() < f32::EPSILON);
            assert!((ray.start.y - 4940.).abs() < f32::EPSILON);
            assert!((ray.start.z - -4920.).abs() < f32::EPSILON);
            assert!((ray.direction.x - 0.6666667).abs() < f32::EPSILON);
            assert!((ray.direction.y - 0.6666667).abs() < f32::EPSILON);
            assert!((ray.direction.z - -0.33333334).abs() < f32::EPSILON);
        }
    }
}
