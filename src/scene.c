#include "scene.h"
#include "collision.h"
#include <stdio.h>

void scene_init(Scene* scene, float box_size) {
    scene->num_objects = 0;
    scene->box_size = box_size;
}

void scene_add_object(Scene* scene, PhysicsObject obj) {
    if (scene->num_objects < MAX_OBJECTS) {
        scene->objects[scene->num_objects++] = obj;
    } else {
        fprintf(stderr, "Warning: Maximum number of objects reached.\n");
    }
}

void scene_update(Scene* scene, float dt) {
    // 1. Apply forces to all objects
    Vec3 gravity = {0.0f, -9.81f, 0.0f};
    for (int i = 0; i < scene->num_objects; ++i) {
        // F⃗₉ = mg⃗
        physics_apply_force(&scene->objects[i], vec3_scale(gravity, scene->objects[i].mass));
    }

    // 2. Update positions and velocities for all objects
    for (int i = 0; i < scene->num_objects; ++i) {
        physics_update(&scene->objects[i], dt);
    }
    
    // 3. Handle collisions
    //    a) Collisions between objects
    //       This is an O(n²) check. For a large number of objects,
    //       spatial partitioning (e.g., grids, octrees) would be needed.
    for (int i = 0; i < scene->num_objects; ++i) {
        for (int j = i + 1; j < scene->num_objects; ++j) {
            collision_resolve_spheres(&scene->objects[i], &scene->objects[j]);
        }
    }

    //    b) Collisions with world boundaries
    for (int i = 0; i < scene->num_objects; ++i) {
        collision_handle_world_boundaries(&scene->objects[i], scene->box_size);
    }
}