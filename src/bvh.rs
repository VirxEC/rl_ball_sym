use crate::geometry::{Aabb, Tri};
use crate::bit_packing::bits_needed;

pub struct BvhNode {
    pub box_: Aabb,
    pub code: u32
}

pub struct Bvh {
    pub global: Aabb,
    pub mask: u64,
    pub num_leaves: u64,

    pub nodes: Vec<BvhNode>,

    pub ranges: Vec<i8>,
    pub ready: Vec<i32>,
    pub parents: Vec<i32>,
    pub siblings: Vec<i32>,
    pub code_ids: Vec<u64>,
    pub primitives: Vec<Tri>,
}

/*
impl Bvh {
    pub fn from(_primitives: &Vec<Tri>) -> Self {
        let num_leaves = _primitives.len();

        let primitives: Vec<Tri> = vec![Tri::default(); num_leaves];

        // nodes.resize(2 * num_leaves - 1);
        let mut ranges: Vec<i8> = Vec::new();
        ranges.reserve(2 * num_leaves - 1);
        let mut ready: Vec<i32> = Vec::new();
        ready.reserve(2 * num_leaves - 1);
        let mut parents: Vec<i32> = Vec::new();
        parents.reserve(2 * num_leaves - 1);
        let mut siblings: Vec<i32> = Vec::new();
        siblings.reserve(2 * num_leaves - 1);

        let mask: u64 = ((1 as u64) << bits_needed(num_leaves as u32)) - (1 as u64);

        // std::vector < aabb > boxes(num_leaves);
        let boxes: Aabb;
        // for (int i = 0; i < num_leaves; i++) {
        //     boxes[i] = aabb(_primitives[i]);
        // }

        // global = global_aabb(boxes);

        // code_ids = morton_sort(boxes, global);

        // for (int i = 0; i < num_leaves; i++) {
        //     uint32_t id = uint32_t(code_ids[i] & mask);
        //     primitives[i] = _primitives[id];
        //     nodes[i] = bvh_node{ boxes[id], code_ids[i] };
        // }

        // build_radix_tree();
        // fit_bounding_boxes();

        Bvh {
            primitives,
            ranges,
            ready,
            parents,
            siblings,
            mask,
            num_leaves
        }
    }
}
*/