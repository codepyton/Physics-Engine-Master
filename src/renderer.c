#include "renderer.h"
#include <GL/glut.h>

void renderer_setup() {
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_NORMALIZE); // Important for scaled objects

    GLfloat light_pos[] = {2.0f, 5.0f, 5.0f, 0.0f};
    glLightfv(GL_LIGHT0, GL_POSITION, light_pos);

    glClearColor(0.1f, 0.1f, 0.15f, 1.0f);
}

void renderer_reshape(int width, int height) {
    if (height == 0) height = 1;
    float aspect = (float)width / (float)height;

    glViewport(0, 0, width, height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, aspect, 1.0, 100.0);
}

void draw_bounding_box(float size) {
    glColor3f(0.7f, 0.7f, 0.7f);
    glutWireCube(size);
}

void draw_sphere(const PhysicsObject* obj) {
    glColor3f(obj->color.x, obj->color.y, obj->color.z);
    glPushMatrix();
    glTranslatef(obj->position.x, obj->position.y, obj->position.z);
    glutSolidSphere(obj->radius, 20, 20);
    glPopMatrix();
}

void renderer_draw(const Scene* scene) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Setup the camera using gluLookAt
    gluLookAt(
        0.0, 10.0, 25.0,  // Eye position (e⃗)
        0.0, 0.0, 0.0,    // Center/look-at point (c⃗)
        0.0, 1.0, 0.0     // Up vector (u⃗)
    );

    draw_bounding_box(scene->box_size);
    
    // Iterate through all objects in the scene and draw them
    for (int i = 0; i < scene->num_objects; ++i) {
        draw_sphere(&scene->objects[i]);
    }

    glutSwapBuffers();
}