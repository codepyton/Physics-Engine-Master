#ifndef PHYSICS_H
#define PHYSICS_H

#include "vec3.h"

#define TRAIL_LENGTH 50

// Type of collider shape
typedef enum {
    SHAPE_SPHERE,
    SHAPE_BOX       // Placeholder for future expansion
} ShapeType;

// Rigid Body State
typedef struct {
    // --- Linear State ---
    Vec3 position;          // x⃗(t)
    Vec3 velocity;          // v⃗(t)
    Vec3 force_accumulator; // F⃗_net

    // --- Rotational State ---
    Quat orientation;       // q(t)
    Vec3 angular_velocity;  // ω⃗(t)
    Vec3 torque_accumulator;// τ⃗_net
    
    // --- Mass Properties ---
    float mass;             // m
    float inv_mass;         // 1/m
    Mat3 inertia_tensor;    // I_body (Constant in local frame)
    Mat3 inv_inertia_world; // I⁻¹(t) (Varies in world frame)

    // --- Material Properties ---
    float radius;           // r (for bounding sphere)
    float restitution;      // ε (Bounciness)
    float friction;         // μ (Friction coeff)
    float drag_linear;      // k_d
    float drag_angular;     // k_ω
    
    // --- Visuals ---
    Vec3 color;
    Vec3 trail[TRAIL_LENGTH];
    int trail_head;
    
    // --- Flags ---
    bool is_static;         // Infinite mass
    bool is_sleeping;       // Optimization flag
    int id;                 // Unique identifier
} RigidBody;

// Initialization
void physics_init_body(RigidBody* body, Vec3 pos, float mass, float radius, int id);

// Force Application
void physics_add_force(RigidBody* body, Vec3 force);
void physics_add_force_at_point(RigidBody* body, Vec3 force, Vec3 point);
void physics_add_torque(RigidBody* body, Vec3 torque);

// Integration Step
void physics_integrate(RigidBody* body, float dt);

// Utility
void physics_update_inertia(RigidBody* body);

#endif // PHYSICS_H