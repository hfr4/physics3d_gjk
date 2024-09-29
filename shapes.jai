g_diamond :: #run FillDiamond();

FillDiamond :: () -> [56] Vector3 {
    diamond: [56] Vector3;

    pts: [4 + 4] Vector3;
    pts[0] = .{ 0.1, 0.0, -1.0 };
    pts[1] = .{ 1.0, 0.0,  0.0 };
    pts[2] = .{ 1.0, 0.0,  0.1 };
    pts[3] = .{ 0.4, 0.0,  0.4 };

    quat_half: Quaternion;
    set_from_axis_and_angle(*quat_half, .{ 0, 0, 1 }, 2.0 * PI * 0.125 * 0.5);
    pts[4] = Vector3.{ 0.8, 0, 0.3 };
    pts[4] = rotate(pts[4], quat_half);
    pts[5] = rotate(pts[1], quat_half);
    pts[6] = rotate(pts[2], quat_half);

    quat: Quaternion;
    set_from_axis_and_angle(*quat, .{ 0, 0, 1 }, 2.0 * PI * 0.125);
    idx := 0;
    for i: 0..6 {
        diamond[idx] = pts[i];
        idx += 1;
    }

    quat_accumulator: Quaternion;
    for i: 1..7 {
        quat_accumulator *= quat;
        for pt: 0..6 {
            diamond[idx] = rotate(pts[pt], quat_accumulator);
            idx += 1;
        }
    }

    return diamond;
}