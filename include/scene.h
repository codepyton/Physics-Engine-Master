#ifndef SCENE_H
#define SCENE_H

#include "physics.h"

#define MAX_OBJECTS 16

typedef struct {
    PhysicsObject objects[MAX_OBJECTS];
    int num_objects;
    float box_size;
} Scene;

// Initializes a scene with default values.
void scene_init(Scene* scene, float box_size);

// Adds a new physics object to the scene.
void scene_add_object(Scene* scene, PhysicsObject obj);

// Updates the entire scene for a given time step.
void scene_update(Scene* scene, float dt);

#endif // SCENE_H