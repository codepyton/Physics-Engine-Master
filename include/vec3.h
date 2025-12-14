#ifndef VEC3_H
#define VEC3_H

// Define a structure for a 3D vector.
// A vector v⃗ ∈ ℝ³ is an object with magnitude and direction, representable
// as a tuple (x, y, z) corresponding to a standard basis (î, ĵ, k̂).
typedef struct {
    float x, y, z;
    // An additional component for potential 4D operations (e.g., homogeneous coords),
    // unused in standard 3D physics but useful for alignment.
    float w;
} Vec3;

// --- Vector Operations ---

// Adds two vectors, a⃗ and b⃗.
// Vector addition is defined component-wise:
// c⃗ = a⃗ + b⃗ = (aₓ + bₓ)î + (aᵧ + bᵧ)ĵ + (a₂ + b₂)k̂
Vec3 vec3_add(Vec3 a, Vec3 b);

// Subtracts vector b⃗ from a⃗.
// c⃗ = a⃗ - b⃗ = (aₓ - bₓ)î + (aᵧ - bᵧ)ĵ + (a₂ - b₂)k̂
Vec3 vec3_sub(Vec3 a, Vec3 b);

// Scales a vector by a scalar value s.
// s⋅v⃗ = (s⋅vₓ)î + (s⋅vᵧ)ĵ + (s⋅v₂)k̂
Vec3 vec3_scale(Vec3 v, float s);

// Calculates the dot product of two vectors.
// Geometric definition: a⃗ ⋅ b⃗ = ‖a⃗‖‖b⃗‖cos(θ)
// Algebraic definition: a⃗ ⋅ b⃗ = aₓbₓ + aᵧbᵧ + a₂b₂
float vec3_dot(Vec3 a, Vec3 b);

// Calculates the magnitude (or L2 norm) of a vector.
// The magnitude ‖v⃗‖ is the vector's length, from the Pythagorean theorem.
// ‖v⃗‖ = √(vₓ² + vᵧ² + v₂²) = √(v⃗ ⋅ v⃗)
float vec3_magnitude(Vec3 v);

// Normalizes a vector to have a magnitude of 1.
// A normalized vector, or unit vector v̂, is defined as:
// v̂ = v⃗ / ‖v⃗‖, provided ‖v⃗‖ ≠ 0.
Vec3 vec3_normalize(Vec3 v);

#endif // VEC3_H