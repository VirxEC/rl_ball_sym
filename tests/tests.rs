
use rl_ball_sym::load_soccar;

#[test]
fn build() {
    let game = load_soccar(0, 0);

    // test all the default values to make sure they're proper

    assert_eq!(game.index, 0 as u8);
    assert_eq!(game.team, 0 as u8);

    assert_eq!(game.field.field_mesh.ids.len(), 24084);
    assert_eq!(game.field.field_mesh.vertices.len(), 13152);

    assert_eq!(game.gravity.x as i64, 0);
    assert_eq!(game.gravity.y as i64, 0);
    assert_eq!(game.gravity.z as i64, -650);

    // assert_eq!(&game.field.collision_mesh.global, 0);
    assert_eq!(game.field.collision_mesh.mask, 8191 as u64);

    let num_leaves = game.field.collision_mesh.num_leaves as usize;

    assert_eq!(game.field.collision_mesh.num_leaves, 8028 as u64);
    assert_eq!(game.field.collision_mesh.primitives.len(), num_leaves);
    assert_eq!(game.field.collision_mesh.code_ids.len(), num_leaves);
    assert_eq!(game.field.collision_mesh.siblings.len(), 2 * num_leaves);
    assert_eq!(game.field.collision_mesh.parents.len(), 2 * num_leaves);
    assert_eq!(game.field.collision_mesh.ready.len(), 2 * num_leaves);
    assert_eq!(game.field.collision_mesh.ranges.len(), 2 * num_leaves);

    assert_eq!(game.ball.time as i64, 0);
    assert_eq!(game.ball.location.x as i64, 0);
    assert_eq!(game.ball.location.y as i64, 0);
    assert_eq!(game.ball.location.z as i64, 102);
    assert_eq!(game.ball.velocity.x as i64, 0);
    assert_eq!(game.ball.velocity.y as i64, 0);
    assert_eq!(game.ball.velocity.z as i64, 0);
    assert_eq!(game.ball.angular_velocity.x as i64, 0);
    assert_eq!(game.ball.angular_velocity.y as i64, 0);
    assert_eq!(game.ball.angular_velocity.z as i64, 0);
    assert_eq!(game.ball.radius as i64, 91);
    assert_eq!(game.ball.collision_radius as i64, 93);
}

#[test]
fn predict() {
    let game = load_soccar(0, 0);

    assert_eq!(game.ball.time as i64, 0);
    assert_eq!(game.ball.location.x as i64, 0);
    assert_eq!(game.ball.location.y as i64, 0);
    assert_eq!(game.ball.location.z as i64, 102);
    assert_eq!(game.ball.velocity.x as i64, 0);
    assert_eq!(game.ball.velocity.y as i64, 0);
    assert_eq!(game.ball.velocity.z as i64, 0);
    assert_eq!(game.ball.angular_velocity.x as i64, 0);
    assert_eq!(game.ball.angular_velocity.y as i64, 0);
    assert_eq!(game.ball.angular_velocity.z as i64, 0);
    assert_eq!(game.ball.radius as i64, 91);
    assert_eq!(game.ball.collision_radius as i64, 93);

    let ball_prediction = game.ball.get_ball_prediction_struct(&game);
    let last_slice = &ball_prediction.slices[ball_prediction.num_slices - 1];
    // let mut last_slice = game.ball.clone();
    // last_slice.step(&game, 1./120.);

    assert_eq!(last_slice.time as i64, 0);
    assert_eq!(last_slice.location.x as i64, 0);
    assert_eq!(last_slice.location.y as i64, 0);
    assert_eq!(last_slice.location.z as i64, 102);
    assert_eq!(last_slice.velocity.x as i64, 0);
    assert_eq!(last_slice.velocity.y as i64, 0);
    assert_eq!(last_slice.velocity.z as i64, 0);
    assert_eq!(last_slice.angular_velocity.x as i64, 0);
    assert_eq!(last_slice.angular_velocity.y as i64, 0);
    assert_eq!(last_slice.angular_velocity.z as i64, 0);
    assert_eq!(last_slice.radius as i64, 91);
    assert_eq!(last_slice.collision_radius as i64, 93);
}