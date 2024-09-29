solve :: (using contact: Narrowphase_Contact) {
    pt_on_a_world    := local_to_world(a, pt_on_a_local);
    pt_on_b_world    := local_to_world(b, pt_on_b_local);

    ra               := pt_on_a_world - center_of_mass_local(a);
    rb               := pt_on_b_world - center_of_mass_local(b);

    tensor_a         := inertia_tensor_inv_world(a);
    tensor_b         := inertia_tensor_inv_world(b);

    angular_ja       := cross(tensor_a * cross(ra, normal), ra);
    angular_jb       := cross(tensor_b * cross(rb, normal), rb);
    angular_factor   := dot(angular_ja + angular_jb, normal);

    vel_a            := a.linear_velocity + cross(a.angular_velocity, ra);
    vel_b            := b.linear_velocity + cross(b.angular_velocity, rb);
    vel_dif          := vel_a - vel_b;

    elasticity       := a.elasticity * b.elasticity;
    impulse_j        := (1.0 + elasticity) * dot(vel_dif, normal) / (a.linear_mass_inv + b.linear_mass_inv + angular_factor);
    impulse_j_vec    := impulse_j * normal;

    apply_impulse_at(a, -impulse_j_vec, pt_on_a_world);
    apply_impulse_at(b,  impulse_j_vec, pt_on_b_world);

    friction         := a.friction * b.friction;
    vel_norm         := normal * dot(normal, vel_dif);
    vel_tang         := vel_dif - vel_norm;
    vel_tang_normal  := normalize(vel_tang);

    inertia_a        := cross(tensor_a * cross(ra, vel_tang_normal), ra);
    inertia_b        := cross(tensor_b * cross(rb, vel_tang_normal), rb);
    inertia_inv      := dot(inertia_a + inertia_b, vel_tang_normal);

    reduced_mass     := 1.0 / (a.linear_mass_inv + b.linear_mass_inv + inertia_inv);
    impulse_friction := vel_tang * reduced_mass * friction;

    apply_impulse_at(a, -impulse_friction, pt_on_a_world);
    apply_impulse_at(b,  impulse_friction, pt_on_b_world);

    // Move colliding objects outside of each other
    tA := a.linear_mass_inv / (a.linear_mass_inv + b.linear_mass_inv);
    tB := b.linear_mass_inv / (a.linear_mass_inv + b.linear_mass_inv);
    ds := pt_on_b_world - pt_on_a_world;

    a.position += ds * tA;
    b.position -= ds * tB;
}