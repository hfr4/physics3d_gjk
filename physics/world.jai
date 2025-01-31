World :: struct {
    bodies: [..] *Body;
}

deinit :: (using world: *World) {
    for bodies deinit(it);
    array_reset(*bodies);
}

update :: (using world: *World, dt: float) {
    // for bodies  integrate_forces(it, dt); // TODO

    broadphase_contacts := get_broadphase_contacts(world);
    broadphase_contacts.allocator = temp;

    narrowphase_contacts := get_narrowphase_contacts(broadphase_contacts);
    narrowphase_contacts.allocator = temp;

    for narrowphase_contacts solve(it);
    
    for bodies  integrate_velocities(it, dt);
}

apply_gravity :: (using world: *World, gravity: Vector3 = .{ 0, -9.8, 0 }) {
    for bodies {
        impulse := gravity * (1.0 / it.linear_mass_inv) * 0.01;
        apply_linear_impulse(it, impulse);
    }
}