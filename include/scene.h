#ifndef SCENE_H
#define SCENE_H

#include "physics.h"
#include <stdbool.h>

#define MAX_BODIES 64
#define MAX_PARTICLES 512

// Particle for effects
typedef struct {
    Vec3 position;
    Vec3 velocity;
    float life;
    float max_life;
    Vec3 color;
    bool active;
} Particle;

// Particle Emitter
typedef struct {
    Vec3 position;
    int rate;
    bool active;
} Emitter;

// Scene definition
typedef struct {
    RigidBody bodies[MAX_BODIES];
    int body_count;
    
    Particle particles[MAX_PARTICLES];
    int particle_count;
    
    // Forces
    Vec3 gravity;
    bool gravity_enabled;
    float time_scale;
    
    // World Properties
    float world_size;
} Scene;

void scene_init(Scene* scene);
void scene_reset(Scene* scene);
void scene_add_body(Scene* scene, RigidBody body);
void scene_update(Scene* scene, float dt);
void scene_spawn_explosion(Scene* scene, Vec3 pos, int count);
RigidBody* scene_get_body(Scene* scene, int index);

#endif // SCENE_H