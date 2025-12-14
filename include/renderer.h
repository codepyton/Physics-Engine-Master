#ifndef RENDERER_H
#define RENDERER_H

#include "scene.h"

// Camera System
typedef struct {
    Vec3 pos;
    float yaw;
    float pitch;
    Vec3 front;
    Vec3 right;
    Vec3 up;
} Camera;

extern Camera g_camera;

void renderer_init();
void renderer_resize(int w, int h);
void renderer_render_scene(const Scene* scene);
void renderer_update_camera_mouse(float x, float y);
void renderer_update_camera_keyboard(bool* keys, float dt);
void renderer_debug_octree(struct OctreeNode* node);

#endif // RENDERER_H