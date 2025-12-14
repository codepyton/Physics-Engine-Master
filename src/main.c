#include <GL/glut.h>
#include "scene.h"
#include "renderer.h"
#include <time.h>
#include <stdlib.h>

Scene g_scene;
bool keys[32];
int last_time;
int mouse_x, mouse_y;
bool mouse_locked = true;

void init() {
    srand(time(NULL));
    scene_init(&g_scene);
    renderer_init();
    
    // Create random Rigid Bodies
    for(int i=0; i<50; i++) {
        RigidBody b;
        float r = 0.5f + ((float)rand()/RAND_MAX)*1.0f;
        Vec3 pos = {
            ((float)rand()/RAND_MAX * 20.0f) - 10.0f,
            5.0f + ((float)rand()/RAND_MAX * 20.0f),
            ((float)rand()/RAND_MAX * 20.0f) - 10.0f
        };
        physics_init_body(&b, pos, r*10.0f, r, i);
        
        // Random Rotation
        Vec3 axis = { (float)rand(), (float)rand(), (float)rand() };
        float angle = ((float)rand()/RAND_MAX) * PI * 2;
        b.orientation = quat_from_axis_angle(axis, angle);
        
        // Random Angular Velocity (Spin)
        b.angular_velocity = (Vec3){
            ((float)rand()/RAND_MAX - 0.5f) * 5.0f,
            ((float)rand()/RAND_MAX - 0.5f) * 5.0f,
            ((float)rand()/RAND_MAX - 0.5f) * 5.0f
        };
        
        b.color = (Vec3){ (float)rand()/RAND_MAX, (float)rand()/RAND_MAX, (float)rand()/RAND_MAX };
        scene_add_body(&g_scene, b);
    }
    
    last_time = glutGet(GLUT_ELAPSED_TIME);
}

void update() {
    int now = glutGet(GLUT_ELAPSED_TIME);
    float dt = (now - last_time) / 1000.0f;
    last_time = now;
    if(dt > 0.1f) dt = 0.1f;
    
    renderer_update_camera_keyboard(keys, dt);
    scene_update(&g_scene, dt);
    
    glutPostRedisplay();
}

void display() {
    renderer_render_scene(&g_scene);
}

void reshape(int w, int h) {
    renderer_resize(w, h);
}

void keyboard_down(unsigned char key, int x, int y) {
    (void)x; (void)y;
    keys[key] = true;
    if(key == 27) exit(0);
    if(key == 'p') scene_spawn_explosion(&g_scene, (Vec3){0,5,0}, 50);
}

void keyboard_up(unsigned char key, int x, int y) {
    (void)x; (void)y;
    keys[key] = false;
}

void mouse_motion(int x, int y) {
    if(!mouse_locked) return;
    
    int cx = glutGet(GLUT_WINDOW_WIDTH) / 2;
    int cy = glutGet(GLUT_WINDOW_HEIGHT) / 2;
    
    float dx = x - cx;
    float dy = cy - y; // Invert Y
    
    renderer_update_camera_mouse(dx, dy);
    
    if(dx != 0 || dy != 0)
        glutWarpPointer(cx, cy);
}

int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(1280, 720);
    glutCreateWindow("Advanced Rigid Body Physics");
    
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutIdleFunc(update);
    glutKeyboardFunc(keyboard_down);
    glutKeyboardUpFunc(keyboard_up);
    glutPassiveMotionFunc(mouse_motion);
    
    glutSetCursor(GLUT_CURSOR_NONE);
    
    init();
    
    glutMainLoop();
    return 0;
}