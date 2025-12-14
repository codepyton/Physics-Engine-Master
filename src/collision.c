#include "collision.h"
#include <math.h>

void collision_handle_world_boundaries(PhysicsObject* obj, float box_size) {
    float half_box = box_size / 2.0f;
    float restitution = 0.75f; // Coefficient of restitution (ε)

    // For each axis (i ∈ {x, y, z}):
    // If |posᵢ| + radius > half_box, a collision occurs.
    // The response is to clamp the position and reflect the velocity:
    // v'ᵢ = -ε ⋅ vᵢ
    if (obj->position.x + obj->radius > half_box) {
        obj->position.x = half_box - obj->radius;
        obj->velocity.x *= -restitution;
    }
    if (obj->position.x - obj->radius < -half_box) {
        obj->position.x = -half_box + obj->radius;
        obj->velocity.x *= -restitution;
    }
    if (obj->position.y + obj->radius > half_box) {
        obj->position.y = half_box - obj->radius;
        obj->velocity.y *= -restitution;
    }
    if (obj->position.y - obj->radius < -half_box) {
        obj->position.y = -half_box + obj->radius;
        obj->velocity.y *= -restitution;
    }
    if (obj->position.z + obj->radius > half_box) {
        obj->position.z = half_box - obj->radius;
        obj->velocity.z *= -restitution;
    }
    if (obj->position.z - obj->radius < -half_box) {
        obj->position.z = -half_box + obj->radius;
        obj->velocity.z *= -restitution;
    }
}

/*
 * collision_resolve_spheres
 * Handles the elastic collision between two spheres.
 *
 * 1. Detection:
 * A collision occurs if the distance between centers is less than the sum of radii.
 * Let d = ‖x⃗₂ - x⃗₁‖. Collision if d < (r₁ + r₂).
 *
 * 2. Resolution (based on conservation of momentum and kinetic energy):
 * The collision normal vector is n̂ = (x⃗₂ - x⃗₁) / d.
 * The relative velocity is v⃗ᵣₑₗ = v⃗₁ - v⃗₂.
 * The impulse magnitude `j` for a perfectly elastic collision is calculated.
 * The impulse is a vector quantity J⃗ = jn̂ that describes the change in momentum.
 * j = (-2 * (v⃗ᵣₑₗ ⋅ n̂)) / (1/m₁ + 1/m₂)
 *
 * The change in momentum for each object is:
 * Δp⃗₁ = -J⃗ and Δp⃗₂ = J⃗
 *
 * Since p⃗ = mv⃗, the new velocities are:
 * v⃗'₁ = v⃗₁ - (j/m₁)n̂
 * v⃗'₂ = v⃗₂ + (j/m₂)n̂
 */
void collision_resolve_spheres(PhysicsObject* obj1, PhysicsObject* obj2) {
    Vec3 axis = vec3_sub(obj1->position, obj2->position);
    float dist_sq = vec3_dot(axis, axis);
    float total_radius = obj1->radius + obj2->radius;

    // Check if spheres are colliding (using squared distance to avoid sqrt)
    if (dist_sq < total_radius * total_radius && dist_sq > 1e-9) {
        float dist = sqrtf(dist_sq);
        Vec3 normal = vec3_scale(axis, 1.0f / dist);

        // --- Positional Correction (to prevent overlap) ---
        float overlap = total_radius - dist;
        float mass_ratio1 = obj1->mass / (obj1->mass + obj2->mass);
        float mass_ratio2 = obj2->mass / (obj1->mass + obj2->mass);
        obj1->position = vec3_add(obj1->position, vec3_scale(normal, overlap * mass_ratio2));
        obj2->position = vec3_sub(obj2->position, vec3_scale(normal, overlap * mass_ratio1));


        // --- Impulse Resolution ---
        Vec3 rel_vel = vec3_sub(obj1->velocity, obj2->velocity);
        float vel_along_normal = vec3_dot(rel_vel, normal);

        // Do not resolve if velocities are separating
        if (vel_along_normal > 0) return;

        // For elastic collision, coefficient of restitution ε = 1.
        // We can simplify the general formula for our case.
        float inv_mass1 = 1.0f / obj1->mass;
        float inv_mass2 = 1.0f / obj2->mass;
        
        float impulse_j = -2.0f * vel_along_normal / (inv_mass1 + inv_mass2);
        Vec3 impulse_vec = vec3_scale(normal, impulse_j);
        
        obj1->velocity = vec3_add(obj1->velocity, vec3_scale(impulse_vec, inv_mass1));
        obj2->velocity = vec3_sub(obj2->velocity, vec3_scale(impulse_vec, inv_mass2));
    }
}