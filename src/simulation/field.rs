use super::{bvh::Bvh, mesh::Mesh};
use crate::linear_algebra::{
    mat::MatrixExt,
    math::{axis_to_rotation, dot},
};
use glam::{Mat3A, Vec3A};
use std::f32::consts::{FRAC_PI_3, FRAC_PI_6};

const FLIP_X: Mat3A = Mat3A::from_cols_array_2d(&[[-1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]);
const FLIP_Y: Mat3A = Mat3A::from_cols_array_2d(&[[1., 0., 0.], [0., -1., 0.], [0., 0., 1.]]);

#[inline]
fn quad(p: Vec3A, e1: Vec3A, e2: Vec3A) -> Mesh {
    Mesh::from(
        vec![0, 1, 3, 1, 2, 3],
        [p + e1 + e2, p - e1 + e2, p - e1 - e2, p + e1 - e2].iter().flat_map(Vec3A::to_array).collect(),
    )
}

#[must_use]
/// Get a BVH generated from the given soccar field meshes.
pub(crate) fn initialize_soccer(soccer_corner: &Mesh, soccer_goal: &Mesh, soccer_ramps_0: &Mesh, soccer_ramps_1: &Mesh) -> Bvh {
    const SCALE: f32 = 100.;

    // NOTE: Use Mat3A::from_diagonal(Vec3A::splat(SCALE)) here, when when from_diagonal is const-ified.
    const S: Mat3A = Mat3A::from_cols_array(&[SCALE, 0., 0., 0., SCALE, 0., 0., 0., SCALE]);

    let soccar_corner = soccer_corner.transform(S);
    let soccar_goal = soccer_goal.transform(S);
    let soccar_ramps_0 = soccer_ramps_0.transform(S);
    let soccar_ramps_1 = soccer_ramps_1.transform(S);

    let floor = quad(Vec3A::ZERO, Vec3A::new(4096., 0., 0.), Vec3A::new(0., 5500., 0.));
    let ceiling = quad(Vec3A::new(0., 0., 2048.), Vec3A::new(-4096., 0., 0.), Vec3A::new(0., 5120., 0.));
    let side_walls = [
        quad(Vec3A::new(4096., 0., 1024.), Vec3A::new(0., -5120., 0.), Vec3A::new(0., 0., 1024.)),
        quad(Vec3A::new(-4096., 0., 1024.), Vec3A::new(0., 5120., 0.), Vec3A::new(0., 0., 1024.)),
    ];

    let field_mesh = Mesh::combine(&[
        &soccar_corner,
        &soccar_corner.transform(FLIP_X),
        &soccar_corner.transform(FLIP_Y),
        &soccar_corner.transform(FLIP_X.dot(FLIP_Y)),
        &soccar_goal.translate(Vec3A::new(0., -5120., 0.)),
        &soccar_goal.translate(Vec3A::new(0., -5120., 0.)).transform(FLIP_Y),
        &soccar_ramps_0,
        &soccar_ramps_0.transform(FLIP_X),
        &soccar_ramps_1,
        &soccar_ramps_1.transform(FLIP_X),
        &floor,
        &ceiling,
        &side_walls[0],
        &side_walls[1],
    ]);

    Bvh::from(&field_mesh.to_triangles())
}

#[must_use]
/// Get a BVH generated from the given hoops field meshes.
pub(crate) fn initialize_hoops(hoops_corner: &Mesh, hoops_net: &Mesh, hoops_rim: &Mesh, hoops_ramps_0: &Mesh, hoops_ramps_1: &Mesh) -> Bvh {
    const SCALE: f32 = 0.9;
    const Y_OFFSET: f32 = 431.664;

    // NOTE: Use Mat3A::from_diagonal(Vec3A::splat(SCALE)) here, when when from_diagonal is const-ified.
    const S: Mat3A = Mat3A::from_cols_array(&[SCALE, 0., 0., 0., SCALE, 0., 0., 0., SCALE]);

    const DY: Vec3A = Vec3A::new(0., Y_OFFSET, 0.);

    let transformed_hoops_net = hoops_net.transform(S).translate(DY);
    let transformed_hoops_rim = hoops_rim.transform(S).translate(DY);

    let floor = quad(Vec3A::ZERO, Vec3A::new(2966., 0., 0.), Vec3A::new(0., 3581., 0.));

    let ceiling = quad(Vec3A::new(0., 0., 1820.), Vec3A::new(-2966., 0., 0.), Vec3A::new(0., 3581., 0.));

    let side_walls = [
        quad(Vec3A::new(2966., 0., 910.), Vec3A::new(0., -3581., 0.), Vec3A::new(0., 0., 910.)),
        quad(Vec3A::new(-2966., 0., 910.), Vec3A::new(0., 3581., 0.), Vec3A::new(0., 0., 910.)),
    ];

    let back_walls = [
        quad(Vec3A::new(0., 0., 1024.), Vec3A::new(0., -5120., 0.), Vec3A::new(0., 0., 1024.)),
        quad(Vec3A::new(0., 0., 1024.), Vec3A::new(0., 5120., 0.), Vec3A::new(0., 0., 1024.)),
    ];

    let field_mesh = Mesh::combine(&[
        hoops_corner,
        &hoops_corner.transform(FLIP_X),
        &hoops_corner.transform(FLIP_Y),
        &hoops_corner.transform(FLIP_X.dot(FLIP_Y)),
        &transformed_hoops_net,
        &transformed_hoops_net.transform(FLIP_Y),
        &transformed_hoops_rim,
        &transformed_hoops_rim.transform(FLIP_Y),
        hoops_ramps_0,
        &hoops_ramps_0.transform(FLIP_X),
        hoops_ramps_1,
        &hoops_ramps_1.transform(FLIP_Y),
        &floor,
        &ceiling,
        &side_walls[0],
        &side_walls[1],
        &back_walls[0],
        &back_walls[1],
    ]);

    Bvh::from(&field_mesh.to_triangles())
}

#[must_use]
/// Get a BVH generated from the given dropshot field meshes.
pub(crate) fn initialize_dropshot(dropshot: &Mesh) -> Bvh {
    const SCALE: f32 = 0.393;
    const Z_OFFSET: f32 = -207.565;

    let q = axis_to_rotation(Vec3A::new(0., 0., FRAC_PI_6));

    // NOTE: Use Mat3A::from_diagonal(Vec3A::splat(SCALE)) here, when when from_diagonal is const-ified.
    const S: Mat3A = Mat3A::from_cols_array(&[SCALE, 0., 0., 0., SCALE, 0., 0., 0., SCALE]);

    const DZ: Vec3A = Vec3A::new(0., 0., Z_OFFSET);

    let floor = quad(Vec3A::new(0., 0., 2.), Vec3A::new(10000., 0., 0.), Vec3A::new(0., 7000., 0.));
    let ceiling = quad(Vec3A::new(0., 0., 2020.), Vec3A::new(-10000., 0., 0.), Vec3A::new(0., 7000., 0.));

    let mut p = Vec3A::new(0., 11683.6 * SCALE, 2768.64 * SCALE - Z_OFFSET);
    let mut x = Vec3A::new(5000., 0., 0.);
    const Z: Vec3A = Vec3A::new(0., 0., 1010.);
    let r = axis_to_rotation(Vec3A::new(0., 0., FRAC_PI_3));

    let walls = (0..6)
        .map(|_| {
            let result = quad(p, x, Z);
            p = dot(r, p);
            x = dot(r, x);
            result
        })
        .collect::<Vec<_>>();

    let field_mesh = Mesh::combine(&[
        &dropshot.transform(q.dot(S)).translate(DZ),
        &floor,
        &ceiling,
        &walls[0],
        &walls[1],
        &walls[2],
        &walls[3],
        &walls[4],
        &walls[5],
    ]);

    Bvh::from(&field_mesh.to_triangles())
}

pub(crate) struct InitializeThrowbackParams<'a> {
    pub back_ramps_lower: &'a Mesh,
    pub back_ramps_upper: &'a Mesh,
    pub corner_ramps_lower: &'a Mesh,
    pub corner_ramps_upper: &'a Mesh,
    pub corner_wall_0: &'a Mesh,
    pub corner_wall_1: &'a Mesh,
    pub corner_wall_2: &'a Mesh,
    pub goal: &'a Mesh,
    pub side_ramps_lower: &'a Mesh,
    pub side_ramps_upper: &'a Mesh,
}

#[must_use]
/// Get a BVH generated from the given throwback stadium meshes.
pub(crate) fn initialize_throwback(
    InitializeThrowbackParams {
        back_ramps_lower,
        back_ramps_upper,
        corner_ramps_lower,
        corner_ramps_upper,
        corner_wall_0,
        corner_wall_1,
        corner_wall_2,
        goal,
        side_ramps_lower,
        side_ramps_upper,
    }: InitializeThrowbackParams<'_>,
) -> Bvh {
    const SCALE: f32 = 100.;

    // NOTE: Use Mat3A::from_diagonal(Vec3A::splat(SCALE)) here, when when from_diagonal is const-ified.
    const S: Mat3A = Mat3A::from_cols_array(&[SCALE, 0., 0., 0., SCALE, 0., 0., 0., SCALE]);

    let floor = quad(Vec3A::ZERO, Vec3A::new(4096.6, 0., 0.), Vec3A::new(0., 6910., 0.));
    let ceiling = quad(Vec3A::new(0., 0., 2048.), Vec3A::new(-4096.6, 0., 0.), Vec3A::new(0., 6910., 0.));
    let side_walls: [Mesh; 2] = [
        quad(Vec3A::new(4096.6, 0., 1024.), Vec3A::new(0., -6910., 0.), Vec3A::new(0., 0., 1024.)),
        quad(Vec3A::new(-4096.6, 0., 1024.), Vec3A::new(0., 6910., 0.), Vec3A::new(0., 0., 1024.)),
    ];
    let back_walls: [Mesh; 2] = [
        quad(Vec3A::new(0., 6910., 1024.), Vec3A::new(4096., 0., 0.), Vec3A::new(0., 0., 1024.)),
        quad(Vec3A::new(0., -6910., 1024.), Vec3A::new(-4096., 0., 0.), Vec3A::new(0., 0., 1024.)),
    ];

    let throwback_goal = goal.transform(S);
    let throwback_side_ramps_lower = side_ramps_lower.transform(S);
    let throwback_side_ramps_upper = side_ramps_upper.transform(S);
    let throwback_back_ramps_lower = back_ramps_lower.transform(S);
    let throwback_back_ramps_upper = back_ramps_upper.transform(S);
    let throwback_corner_ramps_lower = corner_ramps_lower.transform(S);
    let throwback_corner_ramps_upper = corner_ramps_upper.transform(S);
    let throwback_corner_wall_0 = corner_wall_0.transform(S);
    let throwback_corner_wall_1 = corner_wall_1.transform(S);
    let throwback_corner_wall_2 = corner_wall_2.transform(S);

    let field_mesh = Mesh::combine(&[
        &throwback_corner_ramps_lower,
        &throwback_corner_ramps_lower.transform(FLIP_X),
        &throwback_corner_ramps_lower.transform(FLIP_Y),
        &throwback_corner_ramps_lower.transform(FLIP_Y).transform(FLIP_X),
        &throwback_corner_ramps_upper,
        &throwback_corner_ramps_upper.transform(FLIP_X),
        &throwback_corner_ramps_upper.transform(FLIP_Y),
        &throwback_corner_ramps_upper.transform(FLIP_Y).transform(FLIP_X),
        &throwback_goal,
        &throwback_goal.transform(FLIP_Y),
        &throwback_side_ramps_lower,
        &throwback_side_ramps_lower.transform(FLIP_X),
        &throwback_side_ramps_upper,
        &throwback_side_ramps_upper.transform(FLIP_X),
        &throwback_back_ramps_lower,
        &throwback_back_ramps_lower.transform(FLIP_Y),
        &throwback_back_ramps_upper,
        &throwback_back_ramps_upper.transform(FLIP_Y),
        &throwback_corner_wall_0,
        &throwback_corner_wall_0.transform(FLIP_X),
        &throwback_corner_wall_0.transform(FLIP_Y),
        &throwback_corner_wall_0.transform(FLIP_Y).transform(FLIP_X),
        &throwback_corner_wall_1,
        &throwback_corner_wall_1.transform(FLIP_X),
        &throwback_corner_wall_1.transform(FLIP_Y),
        &throwback_corner_wall_1.transform(FLIP_Y).transform(FLIP_X),
        &throwback_corner_wall_2,
        &throwback_corner_wall_2.transform(FLIP_X),
        &throwback_corner_wall_2.transform(FLIP_Y),
        &throwback_corner_wall_2.transform(FLIP_Y).transform(FLIP_X),
        &floor,
        &ceiling,
        &side_walls[0],
        &side_walls[1],
        &back_walls[0],
        &back_walls[1],
    ]);

    let triangles = field_mesh.to_triangles();

    Bvh::from(&triangles)
}
