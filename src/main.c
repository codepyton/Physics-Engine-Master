#include <GL/glut.h>
#include <stdlib.h> // For rand()
#include <time.h>   // For time()
#include "scene.h"
#include "renderer.h"

// Global state for our simulation
Scene physics_scene;
int last_time = 0;

void update() {
    int current_time = glutGet(GLUT_ELAPSED_TIME);
    float dt = (current_time - last_time) / 1000.0f;
    last_time = current_time;

    // Clamp delta time to prevent physics explosion with large steps
    if (dt > 0.05f) {
        dt = 0.05f;
    }

    // Delegate the entire update logic to the scene manager
    scene_update(&physics_scene, dt);

    glutPostRedisplay();
}

void display() {
    renderer_draw(&physics_scene);
}

void reshape(int w, int h) {
    renderer_reshape(w, h);
}

// Helper to get a random float between two values
float random_float(float min, float max) {
    return min + ((float)rand() / RAND_MAX) * (max - min);
}

void init_simulation() {
    srand(time(NULL));
    float box_size = 12.0f;
    scene_init(&physics_scene, box_size);

    int num_spheres_to_add = 15;
    for (int i = 0; i < num_spheres_to_add; ++i) {
        PhysicsObject sphere;
        
        sphere.radius = random_float(0.3f, 0.8f);
        sphere.mass = sphere.radius * 5.0f; // Mass proportional to radius

        sphere.position = (Vec3){
            random_float(-box_size/3.0f, box_size/3.0f),
            random_float(0.0f, box_size/2.0f),
            random_float(-box_size/3.0f, box_size/3.0f)
        };
        
        sphere.velocity = (Vec3){
            random_float(-4.0f, 4.0f),
            random_float(-2.0f, 2.0f),
            random_float(-4.0f, 4.0f)
        };

        sphere.color = (Vec3){
            random_float(0.2f, 1.0f),
            random_float(0.2f, 1.0f),
            random_float(0.2f, 1.0f)
        };

        sphere.acceleration = (Vec3){0, 0, 0};

        scene_add_object(&physics_scene, sphere);
    }

    last_time = glutGet(GLUT_ELAPSED_TIME);
}

int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(1024, 768);
    glutCreateWindow("Advanced 3D Physics Simulation - Multiple Objects");

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutIdleFunc(update);

    renderer_setup();
    init_simulation();

    glutMainLoop();
    return 0;
}