use crate::simulation::bit_packing::bits_needed;
use crate::simulation::geometry::Sphere;
use crate::simulation::geometry::{Aabb, Tri};
use crate::simulation::morton::Morton;
use std::boxed::Box;

#[derive(Clone)]
pub struct BvhNode {
    pub is_terminal: bool,
    pub right: Option<Box<BvhNode>>,
    pub left: Option<Box<BvhNode>>,
    pub code: Option<u64>,
    pub primitive: Option<Tri>,
    pub box_: Option<Aabb>,
    pub morton: Option<u64>,
}

impl Default for BvhNode {
    fn default() -> Self {
        Self {
            is_terminal: false,
            right: None,
            left: None,
            code: None,
            primitive: None,
            box_: None,
            morton: None,
        }
    }
}

impl BvhNode {
    pub fn branch(right: BvhNode, left: BvhNode) -> Self {
        Self {
            is_terminal: false,
            right: Some(Box::new(right)),
            left: Some(Box::new(left)),
            code: None,
            primitive: None,
            box_: None,
            morton: None,
        }
    }

    pub fn leaf(code: u64, primitive: Tri, box_: Aabb, morton_code: u64) -> Self {
        Self {
            is_terminal: true,
            right: None,
            left: None,
            code: Some(code),
            primitive: Some(primitive),
            box_: Some(box_),
            morton: Some(morton_code),
        }
    }

    pub fn generate_aabbs(&mut self) {
        // TODO: generate aabbs of each branch in the tree, in a bottom-up fashion
    }
}

pub struct Bvh {
    pub global_box: Aabb,
    pub mask: u64,
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

impl Bvh {
    pub fn from(primitives: &Vec<Tri>) -> Self {
        let num_leaves = primitives.len();
        let mask = ((1 as u64) << bits_needed(num_leaves as u32)) - (1 as u64);

        let mut boxes: Vec<Aabb> = Vec::with_capacity(num_leaves);
        for primitive in primitives {
            boxes.push(Aabb::from_tri(primitive));
        }

        let global_box = global_aabb(&boxes);

        let morton = Morton::from(&global_box, num_leaves as u32);
        let mut leaves: Vec<BvhNode> = Vec::with_capacity(num_leaves);

        let mut morton_codes: Vec<u64> = Vec::with_capacity(num_leaves);
        for (i, box_) in boxes.iter().enumerate() {
            morton_codes.push(morton.get_code(box_, i as u64));
        }
        morton_codes.sort();

        for i in 0..num_leaves {
            let code = morton_codes[i] & mask;
            let code_us = code as usize;
            leaves.push(BvhNode::leaf(code, primitives[code_us], boxes[code_us], morton_codes[i]));
        }

        println!("HELLO");
        let mut root = Bvh::generate_hierarchy(&morton_codes, &leaves, 0, num_leaves - 1);
        println!("HELLO");
        root.generate_aabbs();
        println!("HELLO");

        Self {
            global_box,
            mask,
            num_leaves: num_leaves as u64,
            root: Box::new(root),
        }
    }

    fn generate_hierarchy(sorted_morton_codes: &Vec<u64>, sorted_leaves: &Vec<BvhNode>, first: usize, last: usize) -> BvhNode {
        // If we're dealing with a single object, return the leaf node
        if first == last {
            return sorted_leaves[first].clone();
        }

        // Determine where to split the range

        let split = Bvh::find_split(sorted_morton_codes, first, last);

        // Process the resulting sub-ranges recursively

        let right = Bvh::generate_hierarchy(sorted_morton_codes, sorted_leaves, first, split);
        let left = Bvh::generate_hierarchy(sorted_morton_codes, sorted_leaves, split + 1, last);

        BvhNode::branch(right, left)
    }

    fn find_split(sorted_morton_codes: &Vec<u64>, first: usize, last: usize) -> usize {
        // If the first and last codes are the same, split the range in the middle

        let first_code = sorted_morton_codes[first];
        let last_code = sorted_morton_codes[last];

        if first_code == last_code {
            return (first + last) >> 1;
        }

        // Calculate the number of highest bits that are the same for all objects by counting the leading zeros
        let common_prefix = (first_code ^ last_code).leading_zeros();

        // We don't need last_code now, just first_code
        drop(last_code);

        let mut split = first;
        let mut step = last - first;

        loop {
            step = (step + 1) >> 1; // exponential decrease
            let new_split = split + step; // proposed new position

            if new_split < last {
                let split_code = sorted_morton_codes[new_split];
                let split_prefix = (first_code ^ split_code).leading_zeros();
                if split_prefix > common_prefix {
                    split = new_split; // accept proposal
                }
            }

            if !(step > 1) {
                break;
            }
        }

        step
    }

    pub fn intersect(&self, query_object: &Sphere) -> Vec<i32> {
        let query_box: Aabb = Aabb::from_sphere(&query_object);

        let mut hits = Vec::new();

        // Allocate traversal stack from thread-local memory,
        // and push NULL to indicate that there are no postponed nodes.
        let bvh_default = BvhNode::default();
        let mut stack: Vec<&BvhNode> = vec![&bvh_default; 32];
        let mut stack_ptr = 1;

        // Traverse nodes starting from the root.
        let mut node = &*self.root;
        loop {
            // Check each child node for overlap.
            let left = node.left.as_deref();
            let right = node.right.as_deref();
            let mut traverse_left = false;
            let mut traverse_right = false;

            if left.is_some() {
                let left = left.unwrap();
                if left.box_.unwrap().intersect_self(&query_box) {
                    if left.is_terminal {
                        let left_id = (left.code.unwrap() & self.mask) as i32;
                        if left.primitive.unwrap().intersect_sphere(&query_object) {
                            hits.push(left_id);
                        }
                    } else {
                        // traverse when a query overlaps with an internal node
                        traverse_left = true;
                    }
                }
            }

            if right.is_some() {
                let right = right.as_deref().unwrap();
                if right.box_.unwrap().intersect_self(&query_box) {
                    if right.is_terminal {
                        let right_id = (right.code.unwrap() & self.mask) as i32;
                        if right.primitive.unwrap().intersect_sphere(&query_object) {
                            hits.push(right_id);
                        }
                    } else {
                        // traverse when a query overlaps with an internal node
                        traverse_right = true;
                    }
                }
            }

            if !(traverse_left || traverse_right) {
                // pop
                stack_ptr -= 1;

                if stack_ptr == 0 {
                    break;
                }

                node = &stack[stack_ptr];
            } else if traverse_left {
                node = left.unwrap();
            } else if traverse_right {
                // push
                stack[stack_ptr] = right.unwrap();
                stack_ptr += 1;
            } else {
                if stack_ptr == 0 {
                    break;
                }

                node = right.unwrap()
            }
        }

        hits
    }
}
