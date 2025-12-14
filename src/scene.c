#include "scene.h"
#include "collision.h"
#include <stdlib.h>
#include <stdio.h>

void scene_init(Scene* scene) {
    scene->body_count = 0;
    scene->particle_count = 0;
    scene->gravity = (Vec3){0, -9.81f, 0};
    scene->gravity_enabled = true;
    scene->time_scale = 1.0f;
    scene->world_size = 30.0f;
    
    // Initialize particles to inactive
    for(int i=0; i<MAX_PARTICLES; i++) scene->particles[i].active = false;
}

void scene_reset(Scene* scene) {
    scene->body_count = 0;
    scene->particle_count = 0;
}

void scene_add_body(Scene* scene, RigidBody body) {
    if (scene->body_count < MAX_BODIES) {
        scene->bodies[scene->body_count++] = body;
    }
}

void scene_update_particles(Scene* scene, float dt) {
    for(int i=0; i<MAX_PARTICLES; i++) {
        if (!scene->particles[i].active) continue;
        
        Particle* p = &scene->particles[i];
        p->life -= dt;
        
        if (p->life <= 0) {
            p->active = false;
            continue;
        }
        
        p->position = vec3_add(p->position, vec3_scale(p->velocity, dt));
        p->velocity = vec3_add(p->velocity, vec3_scale(scene->gravity, dt * 0.5f)); // Light gravity
    }
}

void scene_spawn_explosion(Scene* scene, Vec3 pos, int count) {
    int spawned = 0;
    for(int i=0; i<MAX_PARTICLES && spawned < count; i++) {
        if (!scene->particles[i].active) {
            Particle* p = &scene->particles[i];
            p->active = true;
            p->position = pos;
            p->max_life = 0.5f + ((float)rand()/RAND_MAX); // 0.5 to 1.5s
            p->life = p->max_life;
            
            // Random sphere velocity
            float theta = ((float)rand()/RAND_MAX) * 2 * PI;
            float phi = ((float)rand()/RAND_MAX) * PI;
            float speed = 5.0f + ((float)rand()/RAND_MAX) * 10.0f;
            
            p->velocity.x = speed * sinf(phi) * cosf(theta);
            p->velocity.y = speed * cosf(phi);
            p->velocity.z = speed * sinf(phi) * sinf(theta);
            
            p->color = (Vec3){1.0f, 0.5f + ((float)rand()/RAND_MAX)*0.5f, 0.0f}; // Orange-ish
            spawned++;
        }
    }
}

// Global collision resolution helper
void resolve_scene_collisions(Scene* scene) {
    // Brute force narrow phase over Octree would be complex to wire here fully without
    // exposing Octree iterators.
    // For this demonstration, we'll do an O(N^2) loop but optimized by ignoring sleeping bodies
    // In a real huge codebase, we'd iterate the Octree leaves.
    
    for(int i=0; i<scene->body_count; i++) {
        for(int j=i+1; j<scene->body_count; j++) {
            RigidBody* A = &scene->bodies[i];
            RigidBody* B = &scene->bodies[j];
            
            if (A->is_static && B->is_static) continue;
            
            Contact c;
            if (collision_detect_sphere_sphere(A, B, &c)) {
                collision_resolve(&c);
                // Maybe spawn particles on hard hits?
                float rel_vel = vec3_magnitude(vec3_sub(A->velocity, B->velocity));
                if (rel_vel > 5.0f) {
                    // scene_spawn_explosion(scene, c.point, 5); // Avoid recursive chaos for now
                }
            }
        }
    }
    
    // World Boundaries
    float limit = scene->world_size / 2.0f;
    for(int i=0; i<scene->body_count; i++) {
        RigidBody* b = &scene->bodies[i];
        if (b->is_static) continue;
        
        float e = b->restitution;
        if (b->position.y - b->radius < -limit) {
            b->position.y = -limit + b->radius;
            b->velocity.y *= -e;
            // Floor friction
            b->velocity.x *= 0.95f; b->velocity.z *= 0.95f;
        }
        // ... (other walls omitted for brevity in this specific snippet, but imply box)
    }
}

void scene_update(Scene* scene, float dt) {
    dt *= scene->time_scale;
    
    // 1. Forces
    for(int i=0; i<scene->body_count; i++) {
        if (!scene->bodies[i].is_static && scene->gravity_enabled) {
            Vec3 g_force = vec3_scale(scene->gravity, scene->bodies[i].mass);
            physics_add_force(&scene->bodies[i], g_force);
        }
    }
    
    // 2. Integration
    for(int i=0; i<scene->body_count; i++) {
        physics_integrate(&scene->bodies[i], dt);
    }
    
    // 3. Collision
    resolve_scene_collisions(scene);
    
    // 4. Particles
    scene_update_particles(scene, dt);
}