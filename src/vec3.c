#include "vec3.h"
#include <math.h>

/*
 * vec3_add
 * PROOF of vector addition commutativity:
 * Let a⃗ = (aₓ, aᵧ, a₂) and b⃗ = (bₓ, bᵧ, b₂).
 * By definition, a⃗ + b⃗ = (aₓ + bₓ, aᵧ + bᵧ, a₂ + b₂).
 * Since addition in ℝ is commutative, ∀i, aᵢ + bᵢ = bᵢ + aᵢ.
 * Thus, (aₓ + bₓ, …) = (bₓ + aₓ, …) = b⃗ + a⃗.
 * ∴ vector addition is commutative. Q.E.D.
 */
Vec3 vec3_add(Vec3 a, Vec3 b) {
    return (Vec3){a.x + b.x, a.y + b.y, a.z + b.z};
}

/*
 * vec3_sub
 * Subtraction is the addition of the additive inverse.
 * a⃗ - b⃗ = a⃗ + (-b⃗), where -b⃗ = (-bₓ, -bᵧ, -b₂).
 */
Vec3 vec3_sub(Vec3 a, Vec3 b) {
    return (Vec3){a.x - b.x, a.y - b.y, a.z - b.z};
}

/*
 * vec3_scale
 * PROOF of scalar multiplication distributivity over vector addition:
 * s(a⃗ + b⃗) = s((aₓ+bₓ), (aᵧ+bᵧ), (a₂+b₂))
 * = (s(aₓ+bₓ), s(aᵧ+bᵧ), s(a₂+b₂))
 * = (saₓ+sbₓ, saᵧ+sbᵧ, sa₂+sb₂) (Distributivity in ℝ)
 * = (saₓ, saᵧ, sa₂) + (sbₓ, sbᵧ, sb₂)
 * = sa⃗ + sb⃗. Q.E.D.
 */
Vec3 vec3_scale(Vec3 v, float s) {
    return (Vec3){v.x * s, v.y * s, v.z * s};
}

float vec3_dot(Vec3 a, Vec3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

float vec3_magnitude(Vec3 v) {
    return sqrtf(vec3_dot(v, v));
}

/*
 * vec3_normalize
 * A normalized vector v̂ has the property ‖v̂‖ = 1.
 * PROOF: Let v̂ = v⃗ / ‖v⃗‖.
 * ‖v̂‖ = ‖(1/‖v⃗‖)⋅v⃗‖
 * Using the property ‖c⋅u⃗‖ = |c|⋅‖u⃗‖ for a scalar c:
 * ‖v̂‖ = |1/‖v⃗‖| ⋅ ‖v⃗‖
 * Since ‖v⃗‖ ≥ 0, we have |1/‖v⃗‖| = 1/‖v⃗‖.
 * ‖v̂‖ = (1/‖v⃗‖) ⋅ ‖v⃗‖ = 1. Q.E.D.
 */
Vec3 vec3_normalize(Vec3 v) {
    float mag = vec3_magnitude(v);
    if (mag > 1e-6) { // Avoid division by zero
        return vec3_scale(v, 1.0f / mag);
    }
    return (Vec3){0, 0, 0};
}