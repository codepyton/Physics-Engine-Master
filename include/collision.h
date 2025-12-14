#ifndef COLLISION_H
#define COLLISION_H

#include "physics.h"
#include <stdbool.h>

#define MAX_OCTREE_DEPTH 4
#define OCTREE_CAPACITY 8

// A bounding box (AABB)
typedef struct {
    Vec3 min;
    Vec3 max;
} AABB;

// Octree Node
// Divides space recursively into 8 octants to optimize spatial queries.
typedef struct OctreeNode {
    AABB bounds;
    struct OctreeNode* children[8]; // Pointers to sub-octants
    RigidBody** bodies;             // Dynamic array of pointers to bodies in this node
    int body_count;
    int capacity;
    bool is_leaf;
} OctreeNode;

// Contact Manifold
// Stores details about a collision for resolution
typedef struct {
    RigidBody* a;
    RigidBody* b;
    Vec3 normal;        // Points from A to B
    Vec3 point;         // Point of contact in world space
    float penetration;  // Depth of overlap
} Contact;

// --- Collision System ---
void collision_system_init(float world_size);
void collision_system_update(RigidBody* bodies, int count);
void collision_system_cleanup();

// --- Octree Methods ---
OctreeNode* octree_build(AABB bounds, RigidBody** bodies, int count, int depth);
void octree_destroy(OctreeNode* node);
void octree_query(OctreeNode* node, RigidBody* body, RigidBody** results, int* count);

// --- Narrow Phase ---
bool collision_detect_sphere_sphere(RigidBody* a, RigidBody* b, Contact* contact);

// --- Resolution ---
void collision_resolve(Contact* c);

#endif // COLLISION_H