use crate::geometry::{Aabb, Tri};
use crate::bit_packing::bits_needed;
use crate::morton;

pub struct BvhNode {
    pub box_: Aabb,
    pub code: u64
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

fn morton_sort(boxes: &Vec<Aabb>, global: &Aabb) -> Vec<u64> {
    const DIM: u32 = 3;
  
    let num_boxes = boxes.len();
    let x_offset = global.min_x;
    let y_offset = global.min_y;
    let z_offset = global.min_z;
  
    let b = bits_needed(num_boxes as u32);
    let bits_per_dimension = ((64 - b) / DIM) as i32;
    let divisions_per_dimension = 1 << bits_per_dimension;
  
    let scale = (divisions_per_dimension - 1) as f32;
  
    let x_scale = scale / (global.max_x - global.min_x);
    let y_scale = scale / (global.max_y - global.min_y);
    let z_scale = scale / (global.max_z - global.min_z);
  
    let mut code_ids: Vec<u64> = Vec::with_capacity(num_boxes);
  
    for i in 0..num_boxes {
      let box_ = boxes[i];
  
      // get the centroid of the ith bounding box
      let cx = 0.5 * (box_.min_x + box_.max_x);
      let cy = 0.5 * (box_.min_y + box_.max_y);
      let cz = 0.5 * (box_.min_z + box_.max_z);
  
      let ux = ((cx - x_offset) * x_scale) as u64;
      let uy = ((cy - y_offset) * y_scale) as u64;
      let uz = ((cz - z_offset) * z_scale) as u64;
  
      let code: u64 = morton::encode(ux, uy, uz);
  
      code_ids.push((code << b) + (i as u64));
    }
  
    code_ids.sort();
  
    code_ids
}

impl Bvh {
    pub fn from(_primitives: &Vec<Tri>) -> Self {
        let num_leaves = _primitives.len();

        let mut primitives: Vec<Tri> = Vec::with_capacity(num_leaves);

        let capacity = 2 * num_leaves - 1;
        let mut nodes: Vec<BvhNode> = Vec::with_capacity(capacity);
        let ranges: Vec<i8> = Vec::with_capacity(capacity);
        let ready: Vec<i32> = Vec::with_capacity(capacity);
        let parents: Vec<i32> = Vec::with_capacity(capacity);
        let siblings: Vec<i32> = Vec::with_capacity(capacity);

        let mask: u64 = ((1 as u64) << bits_needed(num_leaves as u32)) - (1 as u64);

        let mut boxes: Vec<Aabb> = Vec::with_capacity(num_leaves);
        for i in 0..num_leaves {
            boxes.push(Aabb::from(_primitives[i]));
        }

        let global = global_aabb(&boxes);

        let code_ids = morton_sort(&boxes, &global);

        for i in 0..num_leaves {
            let id = (code_ids[i] & mask) as usize;
            primitives[i] = _primitives[id];
            nodes[i] = BvhNode{
                box_: boxes[id],
                code: code_ids[i]
            };
        }

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
            global,
            code_ids,
            nodes
        }
    }
}
