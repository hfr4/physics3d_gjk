Broadphase_Contact :: struct {
    a: *Body;
    b: *Body;
}

get_broadphase_contacts :: (using world: World) ->  [..] Broadphase_Contact {
    broadphase_contacts: [..] Broadphase_Contact;

    for i: 0..bodies.count - 2 {
        bi := bodies[i];

        for j: i + 1..bodies.count - 1 {
            bj := bodies[j];

            if (is_static(bi) && is_static(bj)) || !collision_aabb_aabb(get_aabb(bi), get_aabb(bj)) continue;

            array_add(*broadphase_contacts, .{ bi, bj });
        }
    }

    return broadphase_contacts;
}
