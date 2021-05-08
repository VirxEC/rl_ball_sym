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

fn global_aabb(boxes: &Vec<Aabb>) -> Aabb {
    let mut global_box = boxes[0];
    for i in 0..boxes.len() {
        global_box = global_box.add(boxes[i]);
    }

    global_box
}

/*
impl Bvh {
    pub fn from(_primitives: &Vec<Tri>) -> Self {
        let num_leaves = _primitives.len();

        let primitives: Vec<Tri> = vec![Tri::default(); num_leaves as usize];

        let capacity = 2 * num_leaves - 1;
        let mut nodes: Vec<BvhNode> = Vec::with_capacity(capacity);
        let mut ranges: Vec<i8> = Vec::with_capacity(capacity);
        let mut ready: Vec<i32> = Vec::with_capacity(capacity);
        let mut parents: Vec<i32> = Vec::with_capacity(capacity);
        let mut siblings: Vec<i32> = Vec::with_capacity(capacity);

        let mask: u64 = ((1 as u64) << bits_needed(num_leaves as u32)) - (1 as u64);

        let boxes: Vec<Aabb> = Vec::with_capacity(num_leaves);
        for i in 0..num_leaves {
            boxes.push(Aabb::from(_primitives[i]));
        }

        let global = global_aabb(&boxes);

        // code_ids = morton_sort(boxes, global);

        // for (int i = 0; i < num_leaves; i++) {
        //     uint32_t id = uint32_t(code_ids[i] & mask);
        //     primitives[i] = _primitives[id];
        //     nodes[i] = bvh_node{ boxes[id], code_ids[i] };
        // }

        // build_radix_tree();
        // fit_bounding_boxes();

        let num_leaves = num_leaves as u64;

        Bvh {
            primitives,
            ranges,
            ready,
            parents,
            siblings,
            mask,
            num_leaves,
            global
        }
    }
}
*/
