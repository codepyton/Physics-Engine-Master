#ifndef RENDERER_H
#define RENDERER_H

#include "scene.h"

// Sets up the initial OpenGL state (lighting, depth testing, etc.).
void renderer_setup();

// Called when the window is resized. Sets up the projection matrix.
void renderer_reshape(int width, int height);

// Draws the entire scene.
void renderer_draw(const Scene* scene);

#endif // RENDERER_H