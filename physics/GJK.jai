// TODO: Find GJK_closest_points

GJK_Collision :: (a: *Body, b: *Body) -> bool {
    ORIGIN :: Vector3.{ 0, 0, 0 };

    simplex:       Simplex;
    simplex_count: int;

    simplex[simplex_count] = Support(a, b, .{ 1, 1, 1 }, 0.0);
    simplex_count += 1;

    dir          := simplex[0].pos * -1.0;
    min_dist     := FLOAT32_MAX;
    is_colliding := false;

    while true {
        pt := Support(a, b, dir, 0.0);
        if HasPoint(simplex, pt) break;

        simplex[simplex_count] = pt;
        simplex_count += 1;

        if dot(dir, pt.pos - ORIGIN) < 0.0 break;

        dir, lambdas:, is_colliding = SimplexSignedVolumes(simplex, simplex_count);
        if is_colliding break;

        dist := length_squared(dir);
        if dist >= min_dist break;
        min_dist = dist;

        simplex_count = SortValids(*simplex, *lambdas);
        is_colliding  = (simplex_count == 4);
        if is_colliding break;
    }

    return is_colliding;
}

EPA_Collision :: (a: *Body, b: *Body) -> Vector3, Vector3, float, bool {
    BIAS   :: 0.001;
    ORIGIN :: Vector3.{ 0, 0, 0 };

    simplex:       Simplex;
    simplex_count: int;

    simplex[simplex_count] = Support(a, b, .{ 1, 1, 1 }, 0.0);
    simplex_count += 1;

    dir          := simplex[0].pos * -1.0;
    min_dist     := FLOAT32_MAX;
    is_colliding := false;

    while true {
        pt := Support(a, b, dir, 0.0);
        if HasPoint(simplex, pt) break;

        simplex[simplex_count] = pt;
        simplex_count += 1;

        if dot(dir, pt.pos - ORIGIN) < 0.0 break;

        dir, lambdas:, is_colliding = SimplexSignedVolumes(simplex, simplex_count);
        if is_colliding break;

        dist := length_squared(dir);
        if dist >= min_dist break;
        min_dist = dist;

        simplex_count = SortValids(*simplex, *lambdas);
        is_colliding  = (simplex_count == 4);
        if is_colliding break;
    }

    if !is_colliding return .{}, .{}, 0.0, false;

    // Check that we have a 3-simplex (EPA expects a tetrahedron)
    if simplex_count == 1 {
        dir := simplex[0].pos * -1.0;
        simplex[simplex_count] = Support(a, b, dir, 0.0);
        simplex_count += 1;
    }

    if simplex_count == 2 {
        ab := simplex[1].pos - simplex[0].pos;
        u, v := GetOrtho(ab);

        dir := u;
        simplex[simplex_count] = Support(a, b, dir, 0.0);
        simplex_count += 1;
    }

    if simplex_count == 3 {
        ab := simplex[1].pos - simplex[0].pos;
        ac := simplex[2].pos - simplex[0].pos;
        n  := cross(ab, ac);

        dir := n;
        simplex[simplex_count] = Support(a, b, dir, 0.0);
        simplex_count += 1;
    }

    // Get the center point of the simplex
    center: Vector3;
    for 0..3 center += simplex[it].pos;
    center *= 0.25;

    // Now expand the simplex by the bias amount
    for 0..simplex_count - 1 {
        pt  := *simplex[it];
        dir := normalize(pt.pos - center); // ray from "center" to witness point

        pt.on_a += dir * BIAS;
        pt.on_b -= dir * BIAS;
        pt.pos   = pt.on_a - pt.on_b;
    }

    pt_on_a, pt_on_b, depth := EPA_Expand(a, b, BIAS, simplex);

    return pt_on_a, pt_on_b, depth, true;
}

#scope_file

Support_Point :: struct {
    on_a: Vector3; // Point on A
    on_b: Vector3; // Point on B
    pos:  Vector3; // Point on minkowski sum
}

Simplex :: [4] Support_Point;

Triangle :: struct {
    a: int;
    b: int;
    c: int;
}

Edge :: struct {
    a: int;
    b: int;
}

EPA_Expand :: (bodyA: *Body, bodyB: *Body, bias: float, simplex: Simplex) -> Vector3, Vector3, float {
    points: [..] Support_Point;
    points.allocator = temp;
    array_add(*points, simplex[0], simplex[1], simplex[2], simplex[3]);

    triangles: [..] Triangle;
    triangles.allocator = temp;
    for i: 0..3 {
        j := (i + 1) % 4;
        k := (i + 2) % 4;
        l := (i + 3) % 4;

        tri := Triangle.{ i, j, k };

        // The unused point is always on the negative/inside of the triangle.. make sure the normal points away
        dist := SignedDistanceToTriangle(points, tri, points[l].pos);
        if dist > 0.0 Swap(*tri.a, *tri.b);
        
        array_add(*triangles, tri);
    }

    center: Vector3;
    for 0..3 center += simplex[it].pos;
    center *= 0.25;

    // Expand the simplex to find the closest face of the CSO to the origin
    while true {
        tri    := ClosestTriangleToPoint(points, triangles, .{ 0, 0, 0});
        normal := TriangleNormal(points, tri);
        pt     := Support(bodyA, bodyB, normal, bias);

        // if w already exists, then just stop because it means we can't expand any further
        if HasPoint(points, triangles, pt.pos) break;

        dist := SignedDistanceToTriangle(points, tri, pt.pos);
        if dist <= 0.0 break; // can't expand

        new_idx := points.count;
        array_add(*points, pt);

        // Remove Triangles that face this point
        for triangles {
            if SignedDistanceToTriangle(points, it, pt.pos) > 0.0 remove it;
        }
        if triangles.count == 0 break;
        
        // Find Dangling Edges
        edges := FindDanglingEdges(triangles);
        edges.allocator = temp;
        if edges.count == 0 break;
        
        // In theory the edges should be a proper CCW order so we only need to add 
        // the new point as 'a' in order to create new triangles that face away from origin
        for edges {
            tri := Triangle.{ new_idx, it.b, it.a };
    
            // Make sure it's oriented properly
            if SignedDistanceToTriangle(points, tri, center) > 0.0 {
                Swap(*tri.b, *tri.c);
            }

            array_add(*triangles, tri);
        }
    }

    // Get the projection of the origin on the closest triangle
    tri     := ClosestTriangleToPoint(points, triangles, .{ 0, 0, 0 });
    coords  := BarycentricCoordinates(points[tri.a].pos, points[tri.b].pos, points[tri.c].pos, .{ 0, 0, 0 });
    pt_on_a := points[tri.a].on_a * coords.x + points[tri.b].on_a * coords.y + points[tri.c].on_a * coords.z;
    pt_on_b := points[tri.a].on_b * coords.x + points[tri.b].on_b * coords.y + points[tri.c].on_b * coords.z;
    depth   := length(pt_on_b - pt_on_a);

    return pt_on_a, pt_on_b, depth;
}

Support :: (a: Body, b: Body, dir: Vector3, bias: float) -> Support_Point {
    point: Support_Point;
    point.on_a = get_support_point(a,  normalize(dir), bias);
    point.on_b = get_support_point(b, -normalize(dir), bias);
    point.pos  = point.on_a - point.on_b;

    return point;
}

SimplexSignedVolumes :: (simplex: Simplex, simplex_count: int) -> Vector3, Vector4, bool {
    EPSILON :: 0.0001 * 0.0001;

    lambda: Vector4;
    if simplex_count == {
        case 2; v := SignedVolume1D(simplex[0].pos, simplex[1].pos);                                 lambda = .{ v.x, v.y, 0.0, 0.0 };
        case 3; v := SignedVolume2D(simplex[0].pos, simplex[1].pos, simplex[2].pos);                 lambda = .{ v.x, v.y, v.z, 0.0 };
        case 4; v := SignedVolume3D(simplex[0].pos, simplex[1].pos, simplex[2].pos, simplex[3].pos); lambda = .{ v.x, v.y, v.z, v.w };
        case; assert(false);
    }

    dir: Vector3;
    for 0..simplex_count - 1 { 
        dir += simplex[it].pos * lambda.component[it]; 
    }

    new_dir      := dir * -1.0;
    is_colliding := length_squared(dir) < EPSILON;
    
    return new_dir, lambda, is_colliding;
}

SignedVolume1D :: (s1: Vector3, s2: Vector3) -> Vector2 {
    ORIGIN :: Vector3.{ 0, 0, 0 };

    // Choose the axis with the greatest difference/length
    idx := 0;
    max_dist := 0.0;
    for 0..2 {
        dist := s2.component[it] - s1.component[it];
        if dist * dist > max_dist * max_dist { 
            max_dist = dist;
            idx      = it;
        }
    }

    // Project the simplex points and projected origin onto the axis with greatest length
    ab := s2 - s1;                                    // Ray from a to b
    ap := ORIGIN - s1;                                // Ray from a to origin
    p0 := s1 + ab * dot(ab, ap) / length_squared(ab); // projection of the origin onto the line

    a := s1.component[idx];
    b := s2.component[idx];
    p := p0.component[idx];

    if      (p >  a && p <  b) || (p >  b && p <  a) return .{ (b - p) / max_dist, (p - a) / max_dist }; // if p is between [a,b]
    else if (a <= b && p <= a) || (a >= b && p >= a) return .{ 1.0, 0.0 };                               // if p is on the far side of a
    else                                             return .{ 0.0, 1.0 };                               // p must be on the far side of b
}

SignedVolume2D :: (s1: Vector3, s2: Vector3, s3: Vector3) -> Vector3 {
    // Find the axis with the greatest projected area
    idx := 0;
    max_area := 0.0;
    for i: 0..2 {
        j := (i + 1) % 3;
        k := (i + 2) % 3;
        area := (s2.component[j] - s1.component[j]) * (s3.component[k] - s1.component[k]) - (s3.component[j] - s1.component[j]) * (s2.component[k] - s1.component[k]);
        if area * area > max_area * max_area {
            max_area = area;
            idx      = i;
        }
    }

    // Project onto the appropriate axis
    x := (idx + 1) % 3;
    y := (idx + 2) % 3;
    s := Vector2.[ 
        .{ s1.component[x], s1.component[y] }, 
        .{ s2.component[x], s2.component[y] }, 
        .{ s3.component[x], s3.component[y] },
    ];

    n  := TriangleNormal(s1, s2, s3);
    p0 := n * dot(s1, n) / length_squared(n);
    p  := Vector2.{ p0.component[x], p0.component[y] };

    // Get the sub-areas of the triangles formed from the projected origin and the edges
	areas: Vector3;
	areas.x = Cross(s[1] - p, s[2] - p);
	areas.y = Cross(s[2] - p, s[0] - p);
	areas.z = Cross(s[0] - p, s[1] - p);

    // If the projected origin is inside the triangle, then return the barycentric points
    if CompareSigns(max_area, areas.x) && CompareSigns(max_area, areas.y) && CompareSigns(max_area, areas.z) {
        return areas / max_area;
    }

    // If we make it here, then we need to project onto the edges and determine the closest point
    lambdas: Vector3;
    min_dist := FLOAT32_MAX;
    for i: 0..2 {
        j := (i + 1) % 3;
        k := (i + 2) % 3;

        edge_pts := Vector3.[ s1, s2, s3 ];
        edge_lbd := SignedVolume1D(edge_pts[j], edge_pts[k]);
        pt       := edge_pts[j] * edge_lbd.x + edge_pts[k] * edge_lbd.y;
        dist     := length_squared(pt);
        if dist < min_dist {
            min_dist = dist;
            lambdas.component[i] = 0;
            lambdas.component[j] = edge_lbd.x;
            lambdas.component[k] = edge_lbd.y;
        }
    }

    return lambdas;
}

SignedVolume3D :: (s1: Vector3, s2: Vector3, s3: Vector3, s4: Vector3) -> Vector4 {
    M := Matrix4.{
        s1.x, s2.x, s3.x, s4.x,
        s1.y, s2.y, s3.y, s4.y,
        s1.z, s2.z, s3.z, s4.z,
        1.0 , 1.0 , 1.0 , 1.0 ,
    };

    cofactors := Vector4.{ Cofactor(M, 3, 0), Cofactor(M, 3, 1), Cofactor(M, 3, 2), Cofactor(M, 3, 3) };
    det       := cofactors.x + cofactors.y + cofactors.z + cofactors.w;

    // If the barycentric coordinates put the origin inside the simplex, then return them
    if CompareSigns(det, cofactors.x) && CompareSigns(det, cofactors.y) && CompareSigns(det, cofactors.z) && CompareSigns(det, cofactors.w) {
        return cofactors * (1.0 / det);
    }

    // If we get here, then we need to project the origin onto the faces and determine the closest one
    lambdas: Vector4;
    min_dist := FLOAT32_MAX;
    for i: 0..3 {
        j := (i + 1) % 4;
        k := (i + 2) % 4;
        l := (i + 3) % 4;

        face_pts := Vector3.[ s1, s2, s3, s4 ];
        face_lbd := SignedVolume2D(face_pts[i], face_pts[j], face_pts[k]);
        pt       := face_pts[i] * face_lbd.x + face_pts[j] * face_lbd.y + face_pts[k] * face_lbd.z;
        dist     := length_squared(pt);
        if dist < min_dist {
            min_dist = dist;
            lambdas.component[i] = face_lbd.x;
            lambdas.component[j] = face_lbd.y;
            lambdas.component[k] = face_lbd.z;
            lambdas.component[l] = 0.0;
        }
    }

    return lambdas;
}

HasPoint :: (simplex: Simplex, pt: Support_Point) -> bool {
    EPSILON :: 0.0001 * 0.0001;

    for 0..3 {
        dist := simplex[it].pos - pt.pos;
        if length_squared(dist) < EPSILON return true;
    }

    return false;
}

SortValids :: (simplex: *Simplex, lambdas: *Vector4) -> int{
    valid_count := 0;
    valid_lbd: Vector4;
    valid_sim: Simplex;

    for 0..3 {
        if lambdas.component[it] == 0.0 continue;

        valid_lbd.component[valid_count] = lambdas.component[it];
        valid_sim[valid_count]           = simplex.*[it];
        valid_count += 1;
    }

    lambdas.* = valid_lbd;
    simplex.* = valid_sim;

    return valid_count;
}

TriangleNormal :: (a: Vector3, b: Vector3, c: Vector3) -> Vector3 {
    return cross(b - a, c - a);
}

BarycentricCoordinates :: (a: Vector3, b: Vector3, c: Vector3, pt: Vector3) -> Vector3 {
    s1 := a - pt;
    s2 := b - pt;
    s3 := c - pt;

    // Find the axis with the greatest projected area
    idx := 0;
    max_area := 0.0;
    for i: 0..2 {
        j := (i + 1) % 3;
        k := (i + 2) % 3;

        area := (s2.component[j] - s1.component[j]) * (s3.component[k] - s1.component[k]) - (s3.component[j] - s1.component[j]) * (s2.component[k] - s1.component[k]);

        if area * area > max_area * max_area {
            max_area = area;
            idx      = i;
        }
    }

    // Project onto the appropriate axis
    x := (idx + 1) % 3;
    y := (idx + 2) % 3;
    s := Vector2.[ 
        .{ s1.component[x], s1.component[y] }, 
        .{ s2.component[x], s2.component[y] }, 
        .{ s3.component[x], s3.component[y] },
    ];

    n  := TriangleNormal(s1, s2, s3);
    p0 := n * dot(s1, n) / length_squared(n);
    p := Vector2.{ p0.component[x], p0.component[y] };

    // Get the sub-areas of the triangles formed from the projected origin and the edges
	areas: Vector3;
	areas.x = Cross(s[1] - p, s[2] - p);
	areas.y = Cross(s[2] - p, s[0] - p);
	areas.z = Cross(s[0] - p, s[1] - p);

    coords := areas / max_area;
    if isnan(coords.x) || isnan(coords.y) || isnan(coords.z) {
        coords = .{ 1, 0, 0 };
    }

    return coords;
}

TriangleNormal :: (points: [] Support_Point, tri: Triangle) -> Vector3 {
    ab := points[tri.b].pos - points[tri.a].pos;
    ac := points[tri.c].pos - points[tri.a].pos;

    return normalize(cross(ab, ac));
}

SignedDistanceToTriangle :: (points: [] Support_Point, tri: Triangle, pt: Vector3) -> float {
    ap := pt - points[tri.a].pos;
    n  := TriangleNormal(points, tri);

    return dot(n, ap);
}

HasPoint :: (points: [] Support_Point, triangles: [] Triangle, pt: Vector3) -> bool {
    EPSILON :: 0.001 * 0.001;

    for triangles {
        if length_squared(pt - points[it.a].pos) < EPSILON return true;
        if length_squared(pt - points[it.b].pos) < EPSILON return true;
        if length_squared(pt - points[it.c].pos) < EPSILON return true;
    }

    return false;
}

ClosestTriangleToPoint :: (points: [] Support_Point, triangles: [] Triangle, pt: Vector3) -> Triangle {
    min_dist := FLOAT32_MAX;
    idx := -1;
    for triangles {
        dist := SignedDistanceToTriangle(points, it, pt);
        if dist * dist < min_dist {
            min_dist = dist * dist;
            idx      = it_index;
        }
    }

    return triangles[idx];
}

FindDanglingEdges :: (triangles: [] Triangle) -> [..] Edge  {
    edges: [..] Edge;

    for ti, i: triangles {
        counts: [3] int;

        ei := Edge.[ .{ ti.a, ti.b }, .{ ti.b, ti.c }, .{ ti.c, ti.a } ];

        for tj, j: triangles {
            if i == j continue;
            
            ej := Edge.[ .{ tj.a, tj.b }, .{ tj.b, tj.c }, .{ tj.c, tj.a } ];

            for k: 0..2 {
                for l: 0..2 {
                    if (ei[k].a == ej[l].a && ei[k].b == ej[l].b) || (ei[k].a == ej[l].b && ei[k].b == ej[l].a) {
                        counts[k] += 1;
                    }
                }
            }
        }

        // An edge that isn't shared, is dangling 
        for 0..2 {
            if counts[it] == 0 array_add(*edges, ei[it]);
        }
    }

    return edges;
}
