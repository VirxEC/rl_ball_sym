use crate::linear_algebra::mat::{dot, Mat3};
use crate::linear_algebra::vector::Vec3;
use crate::simulation::bvh::Bvh;
use crate::simulation::mesh::Mesh;

pub struct Field {
    pub field_mesh: Mesh,
    pub collision_mesh: Bvh,
}

static FLIP_X: Mat3 = [[-1., 0., 0.], [0., 1., 0.], [0., 0., 1.]];

static FLIP_Y: Mat3 = [[1., 0., 0.], [0., -1., 0.], [0., 0., 1.]];

fn quad(p: Vec3, e1: Vec3, e2: Vec3) -> Mesh {
    let vertices: [Vec3; 4] = [p + e1 + e2, p - e1 + e2, p - e1 - e2, p + e1 - e2];

    return Mesh {
        ids: vec![0, 1, 3, 1, 2, 3],
        vertices: vec![vertices[0].x as f32, vertices[0].y as f32, vertices[0].z as f32, vertices[1].x as f32, vertices[1].y as f32, vertices[1].z as f32, vertices[2].x as f32, vertices[2].y as f32, vertices[2].z as f32, vertices[3].x as f32, vertices[3].y as f32, vertices[3].z as f32],
    };
}

impl Field {
    pub fn initialize_soccar(soccar_corner: &Mesh, soccar_goal: &Mesh, soccar_ramps_0: &Mesh, soccar_ramps_1: &Mesh) -> Field {
        let floor = quad(
            Vec3::default(),
            Vec3 {
                x: 4096.,
                y: 0.,
                z: 0.,
            },
            Vec3 {
                x: 0.,
                y: 5120.,
                z: 0.,
            },
        );

        let ceiling = quad(
            Vec3 {
                x: 0.,
                y: 0.,
                z: 2048.,
            },
            Vec3 {
                x: -4096.,
                y: 0.,
                z: 0.,
            },
            Vec3 {
                x: 0.,
                y: 5120.,
                z: 0.,
            },
        );

        let side_walls: [Mesh; 2] = [
            quad(
                Vec3 {
                    x: 4096.,
                    y: 0.,
                    z: 1024.,
                },
                Vec3 {
                    x: 0.,
                    y: -5120.,
                    z: 0.,
                },
                Vec3 {
                    x: 0.,
                    y: 0.,
                    z: 1024.,
                },
            ),
            quad(
                Vec3 {
                    x: -4096.,
                    y: 0.,
                    z: 1024.,
                },
                Vec3 {
                    x: 0.,
                    y: 5120.,
                    z: 0.,
                },
                Vec3 {
                    x: 0.,
                    y: 0.,
                    z: 1024.,
                },
            ),
        ];

        let field_mesh = Mesh::from(vec![
            soccar_corner,
            &soccar_corner.transform(&FLIP_X),
            &soccar_corner.transform(&FLIP_Y),
            &soccar_corner.transform(&dot(&FLIP_X, &FLIP_Y)),
            &soccar_goal.translate(&Vec3 {
                x: 0.,
                y: -5120.,
                z: 0.,
            }),
            &soccar_goal
                .translate(&Vec3 {
                    x: 0.,
                    y: -5120.,
                    z: 0.,
                })
                .transform(&FLIP_Y),
            soccar_ramps_0,
            &soccar_ramps_0.transform(&FLIP_X),
            soccar_ramps_1,
            &soccar_ramps_1.transform(&FLIP_X),
            &floor,
            &ceiling,
            &side_walls[0],
            &side_walls[1],
        ]);

        let triangles = field_mesh.to_triangles();
        let collision_mesh = Bvh::from(&triangles);

        Field {
            field_mesh,
            collision_mesh,
        }
    }
}
