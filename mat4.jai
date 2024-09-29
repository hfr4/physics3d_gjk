mat4_transformation_matrix :: (tra: Vector3, rot: Vector3, sca: Vector3) -> Matrix4 {
    result := Matrix4_Identity;

    result *= mat4_translation_matrix(tra);
    result *= mat4_y_axis_rotation_matrix(rot.y);
    result *= mat4_x_axis_rotation_matrix(rot.x);
    result *= mat4_z_axis_rotation_matrix(rot.z);
    result *= mat4_scale_matrix(sca);

    return result;
}

mat4_transformation_matrix :: (tra: Vector3, rot: Vector3, sca: float) -> Matrix4 {
    return mat4_transformation_matrix(tra, rot, .{ sca, sca, sca });
}

mat4_transformation_matrix :: (tra: Vector3, rot: Quaternion, sca: Vector3) -> Matrix4 {
    result := Matrix4_Identity;

    result *= mat4_translation_matrix(tra);
    result *= mat4_quaternion_rotation_matrix(rot);
    result *= mat4_scale_matrix(sca);

    return result;
}

mat4_transformation_matrix :: (tra: Vector3, rot: Quaternion, sca: float) -> Matrix4 {
    return mat4_transformation_matrix(tra, rot, .{ sca, sca, sca });
}

mat4_translation_matrix :: (tra: Vector3) -> Matrix4 {
    return .{
        1, 0, 0, tra.x,
        0, 1, 0, tra.y,
        0, 0, 1, tra.z,
        0, 0, 0, 1,
    };
}

mat4_x_axis_rotation_matrix :: (rx: float) -> Matrix4 {
    return .{
        1, 0      ,  0       , 0,
        0, cos(rx), -sin(rx) , 0,
        0, sin(rx),  cos(rx) , 0,
        0, 0      ,  0       , 1,
    };
}

mat4_y_axis_rotation_matrix :: (ry: float) -> Matrix4 {
    return .{
        cos(ry), 0, sin(ry), 0,
        0      , 1, 0      , 0,
       -sin(ry), 0, cos(ry), 0,
        0      , 0, 0      , 1,
    };
}

mat4_z_axis_rotation_matrix :: (rz: float) -> Matrix4 {
    return .{
        cos(rz), -sin(rz), 0, 0,
        sin(rz), cos(rz) , 0, 0,
        0      , 0       , 1, 0,
        0      , 0       , 0, 1,
    };
}

mat4_quaternion_rotation_matrix :: (using q: Quaternion) -> Matrix4 {
    return .{
       -2*(y*y+z*z)+1,  2*(x*y-z*w)  ,  2*(x*z+y*w)  , 0,
        2*(x*y+z*w)  , -2*(x*x+z*z)+1,  2*(y*z-x*w)  , 0,
        2*(x*z-y*w)  ,  2*(y*z+x*w)  , -2*(x*x+y*y)+1, 0,
        0            ,  0            ,  0            , 1,
    };
}

mat4_scale_matrix :: (sca: Vector3) -> Matrix4 {
    return .{
        sca.x, 0    , 0    , 0,
        0    , sca.y, 0    , 0,
        0    , 0    , sca.z, 0,
        0    , 0    , 0    , 1,
    };
}

mat4_perspective_matrix :: (fov: float, near: float, far: float, aspect_ratio: float) -> Matrix4 {
    a :=  1 / (tan(fov / 2) * aspect_ratio);
    b :=  1 / (tan(fov / 2));
    c := -1 * (far + near) / (far - near);
    d := -1 * (far * near * 2) / (far - near);

    return .{
        a, 0, 0 , 0,
        0, b, 0 , 0,
        0, 0, c , d,
        0, 0, -1, 0,
    };
}

mat4_orthographic_matrix :: (left: float, right: float, bottom: float, top: float, near: float, far: float) -> Matrix4  {
    a :=  2 / (right - left);
    b :=  2 / (top   - bottom);
    c := -2 / (far   - near);
    d := -(right + left)   / (right - left);
    e := -(top   + bottom) / (top   - bottom);
    f := -(far   + near)   / (far   - near);

    return .{
        a, 0, 0, d,
        0, b, 0, e,
        0, 0, c, f,
        0, 0, 0, 1,
    };
}

mat4_orthographic_matrix :: (fov: float, size: float, near: float, far: float, aspect_ratio: float) -> Matrix4 {
    l := -size * tan(fov/2) * aspect_ratio;
    r :=  size * tan(fov/2) * aspect_ratio;
    b := -size * tan(fov/2);
    t :=  size * tan(fov/2);

    return mat4_orthographic_matrix(l, r, b, t, near, far);
}

mat4_view_matrix :: (pos: Vector3, dir: Vector3, up: Vector3) -> Matrix4 {
    z := -normalize(dir);
    x := -normalize(cross(up, z));
    y := -cross(z, x);
    
    return .{
        x.x, x.y, x.z, -dot(x, pos),
        y.x, y.y, y.z, -dot(y, pos),
        z.x, z.y, z.z, -dot(z, pos),
        0  , 0  , 0  , 1
    };
}