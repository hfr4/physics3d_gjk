Narrowphase_Contact :: struct {
    a:             *Body;
    b:             *Body;
    normal:        Vector3;
    depth:         float;
    pt_on_a_local: Vector3;
    pt_on_b_local: Vector3;
}

get_narrowphase_contacts :: (broadphase_contacts: [..] Broadphase_Contact) -> [..] Narrowphase_Contact {
    narrowphase_contacts: [..] Narrowphase_Contact;

    for bc: broadphase_contacts {
        if bc.a.shape.kind == .SPHERE && bc.b.shape.kind == .SPHERE {
            contact, ok := sphere_sphere_narrowphase_contact(bc.a, bc.b);
            if ok array_add(*narrowphase_contacts, contact);
        } else {
            contact, ok := epa_narrowphase_contact(bc.a, bc.b);
            if ok array_add(*narrowphase_contacts, contact);
        }
    }

    return narrowphase_contacts;
}

sphere_sphere_narrowphase_contact :: (a: *Body, b: *Body) -> Narrowphase_Contact, bool {
    sphere_a := cast(*Sphere_Shape) a.shape;
    sphere_b := cast(*Sphere_Shape) b.shape;

    if !collision_sphere_sphere(a.position, sphere_a.radius, b.position, sphere_b.radius) {
        return .{}, false;
    }

    normal        := normalize(b.position - a.position);
    pt_on_a_world := a.position + normal * sphere_a.radius;
    pt_on_b_world := b.position - normal * sphere_b.radius;

    contact: Narrowphase_Contact;
    contact.a             = a;
    contact.b             = b;
    contact.normal        = normal;
    contact.depth         = length(pt_on_b_world - pt_on_a_world);
    contact.pt_on_a_local = world_to_local(a, pt_on_a_world);
    contact.pt_on_b_local = world_to_local(b, pt_on_b_world);

    return contact, true;
}

epa_narrowphase_contact :: (a: *Body, b: *Body) -> Narrowphase_Contact, bool {
    pt_on_a_world, pt_on_b_world, depth, ok := EPA_Collision(a, b);

    if !ok return .{}, false;

    contact: Narrowphase_Contact;
    contact.a             = a;
    contact.b             = b;
    contact.normal        = normalize(pt_on_b_world - pt_on_a_world);
    contact.depth         = depth;
    contact.pt_on_a_local = world_to_local(a, pt_on_a_world);
    contact.pt_on_b_local = world_to_local(b, pt_on_b_world);

    return contact, true;
}