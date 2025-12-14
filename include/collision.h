#ifndef COLLISION_H
#define COLLISION_H

#include "physics.h"

// Handles collisions with the boundaries of a cube.
void collision_handle_world_boundaries(PhysicsObject* obj, float box_size);

// Detects and resolves a collision between two physics objects (spheres).
void collision_resolve_spheres(PhysicsObject* obj1, PhysicsObject* obj2);

#endif // COLLISION_H