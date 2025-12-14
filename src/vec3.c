#include "vec3.h"
#include <stdio.h>

// ============================================================================
// VECTOR OPERATIONS (ℝ³)
// ============================================================================

Vec3 vec3_zero() { return (Vec3){0, 0, 0}; }
Vec3 vec3_set(float x, float y, float z) { return (Vec3){x, y, z}; }

Vec3 vec3_add(Vec3 a, Vec3 b) { return (Vec3){a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 vec3_sub(Vec3 a, Vec3 b) { return (Vec3){a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 vec3_scale(Vec3 v, float s) { return (Vec3){v.x * s, v.y * s, v.z * s}; }
Vec3 vec3_mul(Vec3 a, Vec3 b) { return (Vec3){a.x * b.x, a.y * b.y, a.z * b.z}; }

float vec3_dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

Vec3 vec3_cross(Vec3 a, Vec3 b) {
    return (Vec3){
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

float vec3_mag_sq(Vec3 v) { return v.x * v.x + v.y * v.y + v.z * v.z; }
float vec3_magnitude(Vec3 v) { return sqrtf(vec3_mag_sq(v)); }

float vec3_dist_sq(Vec3 a, Vec3 b) {
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    float dz = a.z - b.z;
    return dx*dx + dy*dy + dz*dz;
}

Vec3 vec3_normalize(Vec3 v) {
    float m = vec3_mag_sq(v);
    if (m > 1e-9) {
        float inv = 1.0f / sqrtf(m);
        return vec3_scale(v, inv);
    }
    return vec3_zero();
}

Vec3 vec3_reflect(Vec3 v, Vec3 n) {
    // r⃗ = v⃗ - 2(v⃗ ⋅ n̂)n̂
    float d = vec3_dot(v, n);
    return vec3_sub(v, vec3_scale(n, 2.0f * d));
}

Vec3 vec3_lerp(Vec3 a, Vec3 b, float t) {
    return vec3_add(a, vec3_scale(vec3_sub(b, a), t));
}

// ============================================================================
// QUATERNION OPERATIONS (ℍ)
// ============================================================================

Quat quat_identity() { return (Quat){1.0f, 0.0f, 0.0f, 0.0f}; }

// Construction from Axis-Angle representation
// q = cos(θ/2) + sin(θ/2)(xî + yĵ + zk̂)
Quat quat_from_axis_angle(Vec3 axis, float angle) {
    float half_angle = angle * 0.5f;
    float s = sinf(half_angle);
    Vec3 n = vec3_normalize(axis);
    return (Quat){
        cosf(half_angle),
        n.x * s,
        n.y * s,
        n.z * s
    };
}

// Hamilton Product
// q₁ ⊗ q₂ = (w₁w₂ - v⃗₁⋅v⃗₂, w₁v⃗₂ + w₂v⃗₁ + v⃗₁×v⃗₂)
Quat quat_mul(Quat a, Quat b) {
    return (Quat){
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    };
}

Quat quat_conjugate(Quat q) {
    return (Quat){q.w, -q.x, -q.y, -q.z};
}

Quat quat_normalize(Quat q) {
    float mag = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (mag > 1e-9) {
        float inv = 1.0f / mag;
        return (Quat){q.w*inv, q.x*inv, q.y*inv, q.z*inv};
    }
    return quat_identity();
}

// Rotate vector v by quaternion q
// v' = q v q* (where v is treated as pure imaginary quaternion)
Vec3 quat_rotate_vec(Quat q, Vec3 v) {
    Quat vec_q = {0, v.x, v.y, v.z};
    Quat res = quat_mul(quat_mul(q, vec_q), quat_conjugate(q));
    return (Vec3){res.x, res.y, res.z};
}

// Convert Quaternion to Rotation Matrix
Mat3 quat_to_mat3(Quat q) {
    Mat3 m;
    float xx = q.x*q.x, yy = q.y*q.y, zz = q.z*q.z;
    float xy = q.x*q.y, xz = q.x*q.z, yz = q.y*q.z;
    float wx = q.w*q.x, wy = q.w*q.y, wz = q.w*q.z;

    m.m[0][0] = 1.0f - 2.0f*(yy + zz);
    m.m[0][1] = 2.0f*(xy - wz);
    m.m[0][2] = 2.0f*(xz + wy);

    m.m[1][0] = 2.0f*(xy + wz);
    m.m[1][1] = 1.0f - 2.0f*(xx + zz);
    m.m[1][2] = 2.0f*(yz - wx);

    m.m[2][0] = 2.0f*(xz - wy);
    m.m[2][1] = 2.0f*(yz + wx);
    m.m[2][2] = 1.0f - 2.0f*(xx + yy);

    return m;
}

// ============================================================================
// MATRIX OPERATIONS
// ============================================================================

Mat3 mat3_identity() {
    Mat3 m = {{{0}}};
    m.m[0][0] = 1; m.m[1][1] = 1; m.m[2][2] = 1;
    return m;
}

Mat3 mat3_zero() {
    return (Mat3){{{0}}};
}

Vec3 mat3_mul_vec(Mat3 m, Vec3 v) {
    return (Vec3){
        m.m[0][0]*v.x + m.m[0][1]*v.y + m.m[0][2]*v.z,
        m.m[1][0]*v.x + m.m[1][1]*v.y + m.m[1][2]*v.z,
        m.m[2][0]*v.x + m.m[2][1]*v.y + m.m[2][2]*v.z
    };
}

Mat3 mat3_transpose(Mat3 m) {
    Mat3 r;
    for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
            r.m[i][j] = m.m[j][i];
    return r;
}

Mat3 mat3_inverse(Mat3 m) {
    float det = m.m[0][0] * (m.m[1][1] * m.m[2][2] - m.m[2][1] * m.m[1][2]) -
                m.m[0][1] * (m.m[1][0] * m.m[2][2] - m.m[1][2] * m.m[2][0]) +
                m.m[0][2] * (m.m[1][0] * m.m[2][1] - m.m[1][1] * m.m[2][0]);

    if (fabs(det) < 1e-6) return mat3_identity();
    float inv_det = 1.0f / det;

    Mat3 res;
    res.m[0][0] = (m.m[1][1] * m.m[2][2] - m.m[2][1] * m.m[1][2]) * inv_det;
    res.m[0][1] = (m.m[0][2] * m.m[2][1] - m.m[0][1] * m.m[2][2]) * inv_det;
    res.m[0][2] = (m.m[0][1] * m.m[1][2] - m.m[0][2] * m.m[1][1]) * inv_det;
    res.m[1][0] = (m.m[1][2] * m.m[2][0] - m.m[1][0] * m.m[2][2]) * inv_det;
    res.m[1][1] = (m.m[0][0] * m.m[2][2] - m.m[0][2] * m.m[2][0]) * inv_det;
    res.m[1][2] = (m.m[1][0] * m.m[0][2] - m.m[0][0] * m.m[1][2]) * inv_det;
    res.m[2][0] = (m.m[1][0] * m.m[2][1] - m.m[2][0] * m.m[1][1]) * inv_det;
    res.m[2][1] = (m.m[2][0] * m.m[0][1] - m.m[0][0] * m.m[2][1]) * inv_det;
    res.m[2][2] = (m.m[0][0] * m.m[1][1] - m.m[1][0] * m.m[0][1]) * inv_det;

    return res;
}

Mat4 mat4_identity() {
    Mat4 m = {{{0}}};
    m.m[0][0] = 1; m.m[1][1] = 1; m.m[2][2] = 1; m.m[3][3] = 1;
    return m;
}

Mat4 mat4_from_quat_pos(Quat q, Vec3 p) {
    Mat3 rot = quat_to_mat3(q);
    Mat4 res = {{{0}}};
    
    // Copy rotation
    for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
            res.m[i][j] = rot.m[j][i]; // GL is column-major usually, but we use standard mathematical row-major here for logic, transpose for GL
            
    // For standard row-major math:
    // [ R00 R01 R02 Tx ]
    // [ R10 R11 R12 Ty ]
    // [ R20 R21 R22 Tz ]
    // [  0   0   0   1 ]
    
    res.m[0][0] = rot.m[0][0]; res.m[0][1] = rot.m[0][1]; res.m[0][2] = rot.m[0][2]; res.m[0][3] = p.x;
    res.m[1][0] = rot.m[1][0]; res.m[1][1] = rot.m[1][1]; res.m[1][2] = rot.m[1][2]; res.m[1][3] = p.y;
    res.m[2][0] = rot.m[2][0]; res.m[2][1] = rot.m[2][1]; res.m[2][2] = rot.m[2][2]; res.m[2][3] = p.z;
    res.m[3][0] = 0;           res.m[3][1] = 0;           res.m[3][2] = 0;           res.m[3][3] = 1;
    
    return res;
}