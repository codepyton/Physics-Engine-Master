#include "physics.h"

void physics_apply_force(PhysicsObject* obj, Vec3 force) {
    if (obj->mass > 1e-6) {
        // a⃗_force = F⃗ / m
        Vec3 a_force = vec3_scale(force, 1.0f / obj->mass);
        obj->acceleration = vec3_add(obj->acceleration, a_force);
    }
}

/*
 * physics_update
 * Integrates the equations of motion using the Semi-Implicit Euler method.
 * This method offers a good balance of stability and computational cost.
 *
 * DERIVATION:
 * Based on the Taylor series expansion for position and velocity:
 * x⃗(t + Δt) = x⃗(t) + v⃗(t)Δt + ½a⃗(t)(Δt)² + O((Δt)³)
 * v⃗(t + Δt) = v⃗(t) + a⃗(t)Δt + O((Δt)²)
 *
 * The Semi-Implicit Euler method first updates velocity, then uses the *new*
 * velocity to update position, which enhances stability over the standard
 * forward Euler method.
 *
 * 1. v⃗ₙ₊₁ = v⃗ₙ + a⃗ₙ Δt
 * 2. x⃗ₙ₊₁ = x⃗ₙ + v⃗ₙ₊₁ Δt
 */
void physics_update(PhysicsObject* obj, float dt) {
    // 1. Update velocity
    obj->velocity = vec3_add(obj->velocity, vec3_scale(obj->acceleration, dt));

    // 2. Update position
    obj->position = vec3_add(obj->position, vec3_scale(obj->velocity, dt));

    // 3. Reset acceleration for the next frame. Forces are transient and must
    // be re-applied at each time step.
    obj->acceleration = (Vec3){0.0f, 0.0f, 0.0f};
}