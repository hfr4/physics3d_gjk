VectorN :: struct(N: int) {
    data: [N] float;
}

operator + :: (a: $T/VectorN, b: T) -> T {
    result: T;
    for 0..a.N - 1 {
        result[it] = a[it] + b[it];
    }

    return result;
}

operator - :: (a: $T/VectorN, b: T) -> T {
    result: T;
    for 0..a.N - 1 {
        result[it] = a[it] - b[it];
    }

    return result;
}

operator * :: (v: $T/VectorN, n: float) -> T {
    result: T;
    for 0..v.N - 1 {
        result[it] = v[it] * n;
    }

    return result;
}

operator *[] :: (v: *VectorN, index: int) -> *float {
    return *v.data[index];
}

dot :: (a: $T/VectorN, b: T) -> float {
    result: float;
    for 0..a.N - 1 {
        result += a[it] * b[it];
    }

    return result;
}

MatrixMN :: struct (M: int, N: int) {
    data: [M] VectorN(N);
}

operator * :: (a: MatrixMN, b: MatrixMN) -> MatrixMN(a.M, b.N) {
    transposed := transpose(b);

    result: MatrixMN(a.M, b.N);
    for i: 0..a.M - 1 {
        for j: 0..b.N - 1 {
            result[i][j] = dot(a[i], transposed[j]);
        }
    }

    return result;
}

operator * :: (m: MatrixMN, v: VectorN(m.N)) -> VectorN(m.M) {
    result: VectorN(m.M);
    for 0..m.M - 1 {
        result[it] = dot(v, m[it]);
    }

    return result;
}

operator *[] :: (m: *MatrixMN, index: int) -> *VectorN(m.N) {
    return *m.data[index];
}

transpose :: (m: MatrixMN) -> MatrixMN(m.N, m.M) {
    result: MatrixMN(m.N, m.M);
    for i: 0..m.M - 1 {
        for j: 0..m.N - 1 {
            result[j][i] = m[i][j];
        }
    }

    return result;
}

solve_gauss_seidel :: (m: MatrixMN, v: VectorN) -> VectorN(v.N) {
    iterations := v.N;

    result: VectorN(v.N);
    for 1..iterations {
        for 0..v.N - 1 {
            if m[it][it] != 0.0 {
                result[it] += v[it] / m[it][it] - dot(m[it], result) / m[it][it];
            }
        }
    }

    return result;
}

AABB :: struct {
    min: Vector3;
    max: Vector3;
}

expand :: (using aabb: *AABB, p: Vector3) {
    if p.x < min.x  min.x = p.x;
    if p.y < min.y  min.y = p.y;
    if p.z < min.z  min.z = p.z;
    if p.x > max.x  max.x = p.x;
    if p.y > max.y  max.y = p.y;
    if p.z > max.z  max.z = p.z;
}

expand :: (using aabb: *AABB, pts: [] Vector3) {
    for pts  expand(aabb, it);
}

expand :: (using a: *AABB, b: AABB) {
    expand(a, b.min);
    expand(a, b.max);
}

collision_aabb_aabb :: (a: AABB, b: AABB) -> bool {
    if a.max.x < b.min.x || a.max.y < b.min.y || a.max.z < b.min.z return false;
    if b.max.x < a.min.x || b.max.y < a.min.y || b.max.z < a.min.z return false;
    return true;
}

collision_sphere_sphere :: (pa: Vector3, ra: float, pb: Vector3, rb: float) -> bool {
    r := ra + rb;
    return length_squared(pa - pb) <= r * r;
}

ToMat3 :: (using q: Quaternion) -> Matrix3 {
    result := Matrix3_Identity;
    rotate(*result.v[0], q);
    rotate(*result.v[1], q);
    rotate(*result.v[2], q);

    return result;
}

Inverse :: (using q: Quaternion) -> Quaternion {
    result := q;
    result *= 1.0 / length(result);
    result.x = -result.x;
    result.y = -result.y;
    result.z = -result.z;

    return result;
}

Cross :: (a: Vector2, b: Vector2) -> float {
    return a.x * b.y - b.x * a.y;
}

CompareSigns :: (a: float, b: float) -> bool {
    if a > 0.0 && b > 0.0 return true;
    if a < 0.0 && b < 0.0 return true;
    return false;
}

Cofactor :: (m: Matrix4, i: int, j: int) -> float {
    row1 := Vector3.{ m.coef[(i + 1) % 4][(j + 1) % 4], m.coef[(i + 1) % 4][(j + 2) % 4], m.coef[(i + 1) % 4][(j + 3) % 4] };
    row2 := Vector3.{ m.coef[(i + 2) % 4][(j + 1) % 4], m.coef[(i + 2) % 4][(j + 2) % 4], m.coef[(i + 2) % 4][(j + 3) % 4] };
    row3 := Vector3.{ m.coef[(i + 3) % 4][(j + 1) % 4], m.coef[(i + 3) % 4][(j + 2) % 4], m.coef[(i + 3) % 4][(j + 3) % 4] };
    det  := dot(row1, cross(row2, row3));
    sign := ifx ((i + j) % 2 == 0) then 1 else -1;

    return det * sign;
}

GetOrtho :: (vec: Vector3) -> Vector3, Vector3 {
    n := normalize(vec);
    w := ifx (n.z * n.z > 0.9 * 0.9) then Vector3.{ 1, 0, 0 } else Vector3.{ 0, 0, 1 };
    u := normalize(cross(w, n));
    v := normalize(cross(n, u));
    u = normalize(cross(v, n));

    return u, v;
}