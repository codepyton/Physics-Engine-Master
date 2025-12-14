#ifndef PHYSICS_H
#define PHYSICS_H

#include "vec3.h"

// Represents a physical object in the simulation.
typedef struct {
    Vec3 position;      // x⃗(t) - Position vector
    Vec3 velocity;      // v⃗(t) = dx⃗/dt - Velocity vector
    Vec3 acceleration;  // a⃗(t) = dv⃗/dt - Acceleration vector
    Vec3 color;         // Color for rendering
    float radius;       // r - Radius for collision
    float mass;         // m - Mass
} PhysicsObject;

// Applies a force to an object.
// From Newton's Second Law: F⃗ = ma⃗.
// This implies acceleration is the net force over mass: a⃗ = (∑F⃗)/m.
// This function adds the contribution F⃗/m to the object's acceleration.
void physics_apply_force(PhysicsObject* obj, Vec3 force);

// Updates the state of a physics object over a time step 'dt'.
// Performs numerical integration to solve the equations of motion.
void physics_update(PhysicsObject* obj, float dt);

#endif // PHYSICS_H