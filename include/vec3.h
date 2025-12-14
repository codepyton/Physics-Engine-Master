#ifndef VEC3_H
#define VEC3_H

#include <math.h>
#include <stdbool.h>

// --- Constants ---
#define PI 3.14159265359f
#define DEG2RAD(x) ((x) * PI / 180.0f)
#define RAD2DEG(x) ((x) * 180.0f / PI)

// --- Data Structures ---

// v⃗ ∈ ℝ³
typedef struct {
    float x, y, z;
} Vec3;

// q ∈ ℍ (Quaternion for orientation)
// q = w + xi + yj + zk
typedef struct {
    float w, x, y, z;
} Quat;

// M ∈ ℝ³ˣ³ (Linear transformations, Inertia tensors)
typedef struct {
    float m[3][3];
} Mat3;

// M ∈ ℝ⁴ˣ⁴ (Affine transformations)
typedef struct {
    float m[4][4];
} Mat4;

// --- Vector Operations (ℝ³) ---
Vec3 vec3_zero();
Vec3 vec3_set(float x, float y, float z);
Vec3 vec3_add(Vec3 a, Vec3 b);      // a⃗ + b⃗
Vec3 vec3_sub(Vec3 a, Vec3 b);      // a⃗ - b⃗
Vec3 vec3_scale(Vec3 v, float s);   // s ⋅ v⃗
Vec3 vec3_mul(Vec3 a, Vec3 b);      // Component-wise
float vec3_dot(Vec3 a, Vec3 b);     // ⟨a⃗, b⃗⟩
Vec3 vec3_cross(Vec3 a, Vec3 b);    // a⃗ ∧ b⃗
float vec3_mag_sq(Vec3 v);          // ‖v⃗‖²
float vec3_magnitude(Vec3 v);       // ‖v⃗‖
float vec3_dist_sq(Vec3 a, Vec3 b); // ‖a⃗ - b⃗‖²
Vec3 vec3_normalize(Vec3 v);        // v̂
Vec3 vec3_reflect(Vec3 v, Vec3 n);  // v⃗ - 2(v⃗⋅n̂)n̂
Vec3 vec3_lerp(Vec3 a, Vec3 b, float t);

// --- Quaternion Operations (ℍ) ---
Quat quat_identity();
Quat quat_from_axis_angle(Vec3 axis, float angle);
Quat quat_mul(Quat a, Quat b);      // q₁ ⊗ q₂
Quat quat_conjugate(Quat q);        // q*
Vec3 quat_rotate_vec(Quat q, Vec3 v); // v' = q v q*
Quat quat_normalize(Quat q);
Mat3 quat_to_mat3(Quat q);          // R(q)

// --- Matrix Operations ---
Mat3 mat3_identity();
Mat3 mat3_zero();
Mat3 mat3_mul(Mat3 a, Mat3 b);      // C = AB
Vec3 mat3_mul_vec(Mat3 m, Vec3 v);  // u⃗ = M v⃗
Mat3 mat3_transpose(Mat3 m);        // Mᵀ
Mat3 mat3_inverse(Mat3 m);          // M⁻¹
float mat3_det(Mat3 m);             // det(M)

Mat4 mat4_identity();
Mat4 mat4_translate(Vec3 t);
Mat4 mat4_scale(Vec3 s);
Mat4 mat4_mul(Mat4 a, Mat4 b);
Mat4 mat4_from_quat_pos(Quat q, Vec3 p);

#endif // VEC3_H