#include "physics.h"
#include <string.h>

void physics_init_body(RigidBody* body, Vec3 pos, float mass, float radius, int id) {
    memset(body, 0, sizeof(RigidBody));
    
    body->id = id;
    body->position = pos;
    body->velocity = vec3_zero();
    body->orientation = quat_identity();
    body->angular_velocity = vec3_zero();
    
    body->mass = mass;
    if (mass > 1e-6) {
        body->inv_mass = 1.0f / mass;
        body->is_static = false;
        
        // Inertia Tensor for Solid Sphere: I = (2/5)mr²
        // In local coordinates, this is a diagonal matrix.
        float I = (2.0f / 5.0f) * mass * radius * radius;
        body->inertia_tensor = mat3_identity();
        body->inertia_tensor.m[0][0] = I;
        body->inertia_tensor.m[1][1] = I;
        body->inertia_tensor.m[2][2] = I;
    } else {
        body->inv_mass = 0.0f;
        body->is_static = true;
        body->inertia_tensor = mat3_zero();
    }
    
    body->radius = radius;
    body->restitution = 0.7f;
    body->friction = 0.3f;
    body->drag_linear = 0.1f;
    body->drag_angular = 0.1f;
    body->color = (Vec3){1,1,1};
    
    // Initial World Inertia Calculation
    physics_update_inertia(body);
    
    // Initialize trail
    for(int i=0; i<TRAIL_LENGTH; ++i) body->trail[i] = pos;
    body->trail_head = 0;
}

void physics_update_inertia(RigidBody* body) {
    if (body->is_static) return;
    
    // I⁻¹_world = R I⁻¹_body Rᵀ
    Mat3 R = quat_to_mat3(body->orientation);
    Mat3 I_inv_body = mat3_inverse(body->inertia_tensor);
    Mat3 temp = mat3_mul(R, I_inv_body);
    body->inv_inertia_world = mat3_mul(temp, mat3_transpose(R));
}

void physics_add_force(RigidBody* body, Vec3 force) {
    body->force_accumulator = vec3_add(body->force_accumulator, force);
}

void physics_add_force_at_point(RigidBody* body, Vec3 force, Vec3 point) {
    // F⃗_net += F⃗
    physics_add_force(body, force);
    
    // τ⃗ += r⃗ × F⃗
    Vec3 r = vec3_sub(point, body->position);
    Vec3 torque = vec3_cross(r, force);
    physics_add_torque(body, torque);
}

void physics_add_torque(RigidBody* body, Vec3 torque) {
    body->torque_accumulator = vec3_add(body->torque_accumulator, torque);
}

/*
 * physics_integrate
 *
 * Implements Semi-Implicit Euler for 6DOF Rigid Body Dynamics.
 * * 1. Linear Integration:
 * v⃗_{t+1} = v⃗_t + (F⃗/m)Δt
 * x⃗_{t+1} = x⃗_t + v⃗_{t+1}Δt
 *
 * 2. Rotational Integration:
 * ω⃗_{t+1} = ω⃗_t + (I⁻¹ τ⃗)Δt
 * * Orientation update uses quaternion derivative:
 * dq/dt = ½ ω⃗ ⊗ q
 * q_{t+1} = q_t + (½ ω⃗_{t+1} ⊗ q_t)Δt
 * Re-normalize q_{t+1} to prevent drift.
 *
 * 3. Damping (Drag):
 * Apply linear and angular drag to simulate air resistance.
 */
void physics_integrate(RigidBody* body, float dt) {
    if (body->is_static) return;
    
    // --- 1. Linear Dynamics ---
    
    // Acceleration a⃗ = F⃗_net * (1/m)
    Vec3 accel = vec3_scale(body->force_accumulator, body->inv_mass);
    
    // Update Velocity
    body->velocity = vec3_add(body->velocity, vec3_scale(accel, dt));
    
    // Apply Linear Drag: v⃗ *= 1 / (1 + k_d * dt)
    float linear_damping = 1.0f / (1.0f + body->drag_linear * dt);
    body->velocity = vec3_scale(body->velocity, linear_damping);
    
    // Update Position
    body->position = vec3_add(body->position, vec3_scale(body->velocity, dt));
    
    
    // --- 2. Rotational Dynamics ---
    
    // Angular Acceleration α⃗ = I⁻¹ τ⃗
    // Note: Technically α⃗ = I⁻¹(τ⃗ - ω⃗ × (Iω⃗)) including gyroscopic term.
    // We approximate for stability: α⃗ ≈ I⁻¹ τ⃗
    Vec3 ang_accel = mat3_mul_vec(body->inv_inertia_world, body->torque_accumulator);
    
    // Update Angular Velocity
    body->angular_velocity = vec3_add(body->angular_velocity, vec3_scale(ang_accel, dt));
    
    // Apply Angular Drag
    float angular_damping = 1.0f / (1.0f + body->drag_angular * dt);
    body->angular_velocity = vec3_scale(body->angular_velocity, angular_damping);
    
    // Update Orientation (Quaternion Integration)
    // q_new = q + (0.5 * ω * q) * dt
    Quat q_vel;
    q_vel.w = 0;
    q_vel.x = body->angular_velocity.x;
    q_vel.y = body->angular_velocity.y;
    q_vel.z = body->angular_velocity.z;
    
    Quat spin = quat_mul(q_vel, body->orientation);
    Quat delta_q = (Quat){spin.w * 0.5f * dt, spin.x * 0.5f * dt, spin.y * 0.5f * dt, spin.z * 0.5f * dt};
    
    body->orientation.w += delta_q.w;
    body->orientation.x += delta_q.x;
    body->orientation.y += delta_q.y;
    body->orientation.z += delta_q.z;
    
    body->orientation = quat_normalize(body->orientation);
    
    // Recalculate World Inverse Inertia Tensor
    physics_update_inertia(body);
    
    // --- 3. Cleanup ---
    body->force_accumulator = vec3_zero();
    body->torque_accumulator = vec3_zero();
    
    // Trail Update
    static int trail_counter = 0;
    if (trail_counter++ % 3 == 0) {
        body->trail[body->trail_head] = body->position;
        body->trail_head = (body->trail_head + 1) % TRAIL_LENGTH;
    }
}