use super::{mesh::Mesh, tri_bvh::TriangleBvh};
use glam::{Mat3A, Vec3A};

#[cfg(feature = "dropshot")]
use std::f32::consts::{FRAC_PI_3, FRAC_PI_6};

const FLIP_X: Mat3A = Mat3A::from_cols(Vec3A::NEG_X, Vec3A::Y, Vec3A::Z);
const FLIP_Y: Mat3A = Mat3A::from_cols(Vec3A::X, Vec3A::NEG_Y, Vec3A::Z);

#[inline]
#[cfg(feature = "dropshot")]
/// Convert an axis-angle vector to a rotation matrix.
fn z_axis_to_rotation(axis: f32) -> Mat3A {
    Mat3A::from_axis_angle(glam::Vec3::new(0., 0., axis), axis)
}

#[inline]
fn quad(p: Vec3A, e1: Vec3A, e2: Vec3A) -> Mesh {
    Mesh::new(
        Box::from([0, 1, 3, 1, 2, 3]),
        Box::from([p + e1 + e2, p - e1 + e2, p - e1 - e2, p + e1 - e2]),
    )
}

#[inline]
#[must_use]
#[cfg(feature = "standard")]
pub fn get_standard_walls() -> [Mesh; 4] {
    [
        quad(Vec3A::ZERO, Vec3A::new(4096., 0., 0.), Vec3A::new(0., 5500., 0.)),
        quad(
            Vec3A::new(0., 0., 2048.),
            Vec3A::new(-4096., 0., 0.),
            Vec3A::new(0., 5120., 0.),
        ),
        quad(
            Vec3A::new(4096., 0., 1024.),
            Vec3A::new(0., -5120., 0.),
            Vec3A::new(0., 0., 1024.),
        ),
        quad(
            Vec3A::new(-4096., 0., 1024.),
            Vec3A::new(0., 5120., 0.),
            Vec3A::new(0., 0., 1024.),
        ),
    ]
}

#[must_use]
#[cfg(feature = "standard")]
/// Get a BVH generated from the given standard field meshes.
pub fn initialize_standard(
    standard_corner: Mesh,
    standard_goal: Mesh,
    standard_ramps_0: Mesh,
    standard_ramps_1: Mesh,
) -> TriangleBvh {
    const Y_OFFSET: f32 = -5120.;

    let standard_goal_tf = standard_goal.translate_y(Y_OFFSET);

    let [floor, ceiling, side_wall_0, side_wall_1] = get_standard_walls();

    let field_mesh = Mesh::combine([
        standard_corner.clone().transform(FLIP_X),
        standard_corner.clone().transform(FLIP_Y),
        standard_corner.clone().transform(FLIP_X * FLIP_Y),
        standard_corner,
        standard_goal_tf.clone().transform(FLIP_X),
        standard_goal_tf.clone().transform(FLIP_Y),
        standard_goal_tf.clone().transform(FLIP_X * FLIP_Y),
        standard_goal_tf,
        standard_ramps_0.clone().transform(FLIP_X),
        standard_ramps_0.clone().transform(FLIP_Y),
        standard_ramps_0.clone().transform(FLIP_X * FLIP_Y),
        standard_ramps_0,
        standard_ramps_1.clone().transform(FLIP_X),
        standard_ramps_1.clone().transform(FLIP_Y),
        standard_ramps_1.clone().transform(FLIP_X * FLIP_Y),
        standard_ramps_1,
        floor,
        ceiling,
        side_wall_0,
        side_wall_1,
    ]);

    TriangleBvh::new(field_mesh.into_triangles())
}

#[inline]
#[must_use]
#[cfg(feature = "hoops")]
pub fn get_hoops_walls() -> [Mesh; 6] {
    [
        quad(Vec3A::ZERO, Vec3A::new(2966., 0., 0.), Vec3A::new(0., 3581., 0.)),
        quad(
            Vec3A::new(0., 0., 1820.),
            Vec3A::new(-2966., 0., 0.),
            Vec3A::new(0., 3581., 0.),
        ),
        quad(
            Vec3A::new(2966., 0., 910.),
            Vec3A::new(0., -3581., 0.),
            Vec3A::new(0., 0., 910.),
        ),
        quad(
            Vec3A::new(-2966., 0., 910.),
            Vec3A::new(0., 3581., 0.),
            Vec3A::new(0., 0., 910.),
        ),
        quad(
            Vec3A::new(0., -3581., 910.),
            Vec3A::new(2966., 0., 0.),
            Vec3A::new(0., 0., 910.),
        ),
        quad(
            Vec3A::new(0., 3581., 910.),
            Vec3A::new(-2966., 0., 0.),
            Vec3A::new(0., 0., 910.),
        ),
    ]
}

#[must_use]
#[cfg(feature = "hoops")]
/// Get a BVH generated from the given hoops field meshes.
pub fn initialize_hoops(
    hoops_corner: Mesh,
    hoops_net: Mesh,
    hoops_rim: Mesh,
    hoops_ramps_0: Mesh,
    hoops_ramps_1: Mesh,
) -> TriangleBvh {
    const SCALE: f32 = 0.9;
    const S: Mat3A = Mat3A::from_diagonal(glam::Vec3::splat(SCALE));

    const Y_OFFSET: f32 = 431.664;

    let hoops_net_tf = hoops_net.transform(S).translate_y(Y_OFFSET);
    let hoops_rim_tf = hoops_rim.transform(S).translate_y(Y_OFFSET);

    let [floor, ceiling, side_wall_0, side_wall_1, back_wall_0, back_wall_1] = get_hoops_walls();

    let field_mesh = Mesh::combine([
        hoops_corner.clone().transform(FLIP_X),
        hoops_corner.clone().transform(FLIP_Y),
        hoops_corner.clone().transform(FLIP_X * FLIP_Y),
        hoops_corner,
        hoops_net_tf.clone().transform(FLIP_Y),
        hoops_net_tf,
        hoops_rim_tf.clone().transform(FLIP_Y),
        hoops_rim_tf,
        hoops_ramps_0.clone().transform(FLIP_X),
        hoops_ramps_0,
        hoops_ramps_1.clone().transform(FLIP_Y),
        hoops_ramps_1,
        floor,
        ceiling,
        side_wall_0,
        side_wall_1,
        back_wall_0,
        back_wall_1,
    ]);

    TriangleBvh::new(field_mesh.into_triangles())
}

#[must_use]
#[cfg(feature = "dropshot")]
/// Get a BVH generated from the given dropshot field meshes.
pub fn initialize_dropshot(dropshot: Mesh) -> TriangleBvh {
    const SCALE: f32 = 0.393;
    const S: Mat3A = Mat3A::from_diagonal(glam::Vec3::splat(SCALE));
    const Z_OFFSET: f32 = -207.565;
    const Z: Vec3A = Vec3A::new(0., 0., 1010.);

    let q = z_axis_to_rotation(FRAC_PI_6);

    let floor = quad(Vec3A::new(0., 0., 2.), Vec3A::new(10000., 0., 0.), Vec3A::new(0., 7000., 0.));
    let ceiling = quad(
        Vec3A::new(0., 0., 2020.),
        Vec3A::new(-10000., 0., 0.),
        Vec3A::new(0., 7000., 0.),
    );

    let mut p = Vec3A::new(0., 11683.6 * SCALE, 2768.64 * SCALE - Z_OFFSET);
    let mut x = Vec3A::new(5000., 0., 0.);
    let r = z_axis_to_rotation(FRAC_PI_3).transpose();

    let mut get_wall = || {
        let result = quad(p, x, Z);
        p = r * p;
        x = r * x;
        result
    };

    let field_mesh = Mesh::combine([
        dropshot.transform(q * S).translate_z(Z_OFFSET),
        floor,
        ceiling,
        // it is critical that this operation only happens 6 times
        // there are 6 walls in the dropshot arena
        get_wall(),
        get_wall(),
        get_wall(),
        get_wall(),
        get_wall(),
        get_wall(),
    ]);

    TriangleBvh::new(field_mesh.into_triangles())
}

#[cfg(feature = "throwback")]
pub struct InitializeThrowbackParams {
    pub back_ramps_lower: Mesh,
    pub back_ramps_upper: Mesh,
    pub corner_ramps_lower: Mesh,
    pub corner_ramps_upper: Mesh,
    pub corner_wall_0: Mesh,
    pub corner_wall_1: Mesh,
    pub corner_wall_2: Mesh,
    pub goal: Mesh,
    pub side_ramps_lower: Mesh,
    pub side_ramps_upper: Mesh,
}

#[must_use]
#[cfg(feature = "throwback")]
/// Get a BVH generated from the given throwback stadium meshes.
pub fn initialize_throwback(
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
    }: InitializeThrowbackParams,
) -> TriangleBvh {
    const SCALE: f32 = 100.;
    const S: Mat3A = Mat3A::from_diagonal(glam::Vec3::splat(SCALE));

    let floor = quad(Vec3A::ZERO, Vec3A::new(4096.6, 0., 0.), Vec3A::new(0., 6910., 0.));
    let ceiling = quad(
        Vec3A::new(0., 0., 2048.),
        Vec3A::new(-4096.6, 0., 0.),
        Vec3A::new(0., 6910., 0.),
    );
    let [side_wall_0, side_wall_1] = [
        quad(
            Vec3A::new(4096.6, 0., 1024.),
            Vec3A::new(0., -6910., 0.),
            Vec3A::new(0., 0., 1024.),
        ),
        quad(
            Vec3A::new(-4096.6, 0., 1024.),
            Vec3A::new(0., 6910., 0.),
            Vec3A::new(0., 0., 1024.),
        ),
    ];
    let [back_wall_0, back_wall_1] = [
        quad(
            Vec3A::new(0., 6910., 1024.),
            Vec3A::new(4096., 0., 0.),
            Vec3A::new(0., 0., 1024.),
        ),
        quad(
            Vec3A::new(0., -6910., 1024.),
            Vec3A::new(-4096., 0., 0.),
            Vec3A::new(0., 0., 1024.),
        ),
    ];

    let goal_tf = goal.transform(S);
    let side_ramps_lower_tf = side_ramps_lower.transform(S);
    let side_ramps_upper_tf = side_ramps_upper.transform(S);
    let back_ramps_lower_tf = back_ramps_lower.transform(S);
    let back_ramps_upper_tf = back_ramps_upper.transform(S);
    let corner_ramps_lower_tf = corner_ramps_lower.transform(S);
    let corner_ramps_upper_tf = corner_ramps_upper.transform(S);
    let corner_wall_0_tf = corner_wall_0.transform(S);
    let corner_wall_1_tf = corner_wall_1.transform(S);
    let corner_wall_2_tf = corner_wall_2.transform(S);

    let field_mesh = Mesh::combine([
        corner_ramps_lower_tf.clone().transform(FLIP_X),
        corner_ramps_lower_tf.clone().transform(FLIP_Y),
        corner_ramps_lower_tf.clone().transform(FLIP_X * FLIP_Y),
        corner_ramps_lower_tf,
        corner_ramps_upper_tf.clone().transform(FLIP_X),
        corner_ramps_upper_tf.clone().transform(FLIP_Y),
        corner_ramps_upper_tf.clone().transform(FLIP_X * FLIP_Y),
        corner_ramps_upper_tf,
        goal_tf.clone().transform(FLIP_Y),
        goal_tf,
        side_ramps_lower_tf.clone().transform(FLIP_X),
        side_ramps_lower_tf,
        side_ramps_upper_tf.clone().transform(FLIP_X),
        side_ramps_upper_tf,
        back_ramps_lower_tf.clone().transform(FLIP_Y),
        back_ramps_lower_tf,
        back_ramps_upper_tf.clone().transform(FLIP_Y),
        back_ramps_upper_tf,
        corner_wall_0_tf.clone().transform(FLIP_X),
        corner_wall_0_tf.clone().transform(FLIP_Y),
        corner_wall_0_tf.clone().transform(FLIP_X * FLIP_Y),
        corner_wall_0_tf,
        corner_wall_1_tf.clone().transform(FLIP_X),
        corner_wall_1_tf.clone().transform(FLIP_Y),
        corner_wall_1_tf.clone().transform(FLIP_X * FLIP_Y),
        corner_wall_1_tf,
        corner_wall_2_tf.clone().transform(FLIP_X),
        corner_wall_2_tf.clone().transform(FLIP_Y),
        corner_wall_2_tf.clone().transform(FLIP_X * FLIP_Y),
        corner_wall_2_tf,
        floor,
        ceiling,
        side_wall_0,
        side_wall_1,
        back_wall_0,
        back_wall_1,
    ]);

    TriangleBvh::new(field_mesh.into_triangles())
}
