use crate::simulation::geometry::Sphere;
use crate::simulation::geometry::{Aabb, Tri};
use crate::simulation::morton::Morton;
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

fn global_aabb(boxes: &Vec<Aabb>) -> Aabb {
    let mut global_box = boxes[0];
    for i in 0..boxes.len() {
        global_box = global_box.add(&boxes[i]);
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
    pub fn from(primitives: &Vec<Tri>) -> Self {
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

    fn generate_hierarchy(sorted_leaves: &Vec<Box<BvhNode>>, first: usize, last: usize) -> Box<BvhNode> {
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
        let query_box: Aabb = Aabb::from_sphere(&query_object);

        let mut hits = Vec::with_capacity(8);

        // Allocate traversal stack from thread-local memory,
        // and push NULL to indicate that there are no postponed nodes.
        let mut stack: Vec<&BvhNode> = Vec::with_capacity(32);

        // Traverse nodes starting from the root.
        let mut node = &*self.root;
        loop {
            // Check each child node for overlap.
            let left = node.left.as_deref();
            let right = node.right.as_deref();
            let mut traverse_left = false;
            let mut traverse_right = false;

            match left {
                Some(left) => {
                    if left.box_.intersect_self(&query_box) {
                        match left.primitive {
                            Some(left_tri) => {
                                if left_tri.intersect_sphere(&query_object) {
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
                None => (),
            }

            match right {
                Some(right) => {
                    if right.box_.intersect_self(&query_box) {
                        match right.primitive {
                            Some(right_tri) => {
                                if right_tri.intersect_sphere(&query_object) {
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
                None => (),
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
}
