#include "renderer.h"
#include <GL/glut.h>
#include "collision.h" // For OctreeNode definition

Camera g_camera;

void renderer_init() {
    g_camera.pos = (Vec3){0, 10, 40};
    g_camera.yaw = -90.0f;
    g_camera.pitch = -20.0f;
    g_camera.front = (Vec3){0, 0, -1};
    g_camera.up = (Vec3){0, 1, 0};
    
    // Lighting
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    GLfloat light_pos[] = {20.0f, 50.0f, 20.0f, 1.0f};
    glLightfv(GL_LIGHT0, GL_POSITION, light_pos);
    GLfloat ambient[] = {0.2f, 0.2f, 0.2f, 1.0f};
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
    
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_NORMALIZE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glClearColor(0.1f, 0.12f, 0.15f, 1.0f);
}

void renderer_resize(int w, int h) {
    if(h==0) h=1;
    float aspect = (float)w/h;
    glViewport(0,0,w,h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0f, aspect, 0.1f, 500.0f);
    glMatrixMode(GL_MODELVIEW);
}

void renderer_update_camera_vectors() {
    Vec3 f;
    f.x = cosf(DEG2RAD(g_camera.yaw)) * cosf(DEG2RAD(g_camera.pitch));
    f.y = sinf(DEG2RAD(g_camera.pitch));
    f.z = sinf(DEG2RAD(g_camera.yaw)) * cosf(DEG2RAD(g_camera.pitch));
    g_camera.front = vec3_normalize(f);
    g_camera.right = vec3_normalize(vec3_cross(g_camera.front, (Vec3){0,1,0}));
    g_camera.up = vec3_normalize(vec3_cross(g_camera.right, g_camera.front));
}

void renderer_update_camera_mouse(float x_offset, float y_offset) {
    g_camera.yaw += x_offset * 0.1f;
    g_camera.pitch += y_offset * 0.1f;
    if(g_camera.pitch > 89.0f) g_camera.pitch = 89.0f;
    if(g_camera.pitch < -89.0f) g_camera.pitch = -89.0f;
    renderer_update_camera_vectors();
}

void renderer_update_camera_keyboard(bool* keys, float dt) {
    float vel = 20.0f * dt;
    if(keys['w']) g_camera.pos = vec3_add(g_camera.pos, vec3_scale(g_camera.front, vel));
    if(keys['s']) g_camera.pos = vec3_sub(g_camera.pos, vec3_scale(g_camera.front, vel));
    if(keys['a']) g_camera.pos = vec3_sub(g_camera.pos, vec3_scale(g_camera.right, vel));
    if(keys['d']) g_camera.pos = vec3_add(g_camera.pos, vec3_scale(g_camera.right, vel));
}

void draw_body(const RigidBody* b) {
    glPushMatrix();
    
    // 1. Transform Matrix from Position & Quaternion
    Mat4 transform = mat4_from_quat_pos(b->orientation, b->position);
    
    // OpenGL expects column-major, and our logic might be row-major,
    // but the `mat4_from_quat_pos` output was designed for logic.
    // Let's create a GL array.
    float m[16];
    // We assume the struct is row-major storage in logic, so for GL we transpose.
    m[0] = transform.m[0][0]; m[4] = transform.m[0][1]; m[8] = transform.m[0][2]; m[12] = transform.m[0][3];
    m[1] = transform.m[1][0]; m[5] = transform.m[1][1]; m[9] = transform.m[1][2]; m[13] = transform.m[1][3];
    m[2] = transform.m[2][0]; m[6] = transform.m[2][1]; m[10] = transform.m[2][2]; m[14] = transform.m[2][3];
    m[3] = transform.m[3][0]; m[7] = transform.m[3][1]; m[11] = transform.m[3][2]; m[15] = transform.m[3][3];
    
    glMultMatrixf(m);

    // 2. Draw Sphere
    glColor3f(b->color.x, b->color.y, b->color.z);
    glutSolidSphere(b->radius, 16, 16);
    
    // 3. Draw Local Axes to visualize rotation
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
    glColor3f(1,0,0); glVertex3f(0,0,0); glVertex3f(b->radius*1.5f, 0, 0);
    glColor3f(0,1,0); glVertex3f(0,0,0); glVertex3f(0, b->radius*1.5f, 0);
    glColor3f(0,0,1); glVertex3f(0,0,0); glVertex3f(0, 0, b->radius*1.5f);
    glEnd();
    glEnable(GL_LIGHTING);
    
    glPopMatrix();
    
    // 4. Draw Trail (World Space)
    glDisable(GL_LIGHTING);
    glLineWidth(1.0f);
    glBegin(GL_LINE_STRIP);
    for(int i=0; i<TRAIL_LENGTH; i++) {
        int idx = (b->trail_head + i) % TRAIL_LENGTH;
        float alpha = (float)i/TRAIL_LENGTH;
        glColor4f(b->color.x, b->color.y, b->color.z, alpha);
        glVertex3f(b->trail[idx].x, b->trail[idx].y, b->trail[idx].z);
    }
    glEnd();
    glEnable(GL_LIGHTING);
}

void draw_particles(const Scene* scene) {
    glDisable(GL_LIGHTING);
    glPointSize(3.0f);
    glBegin(GL_POINTS);
    for(int i=0; i<MAX_PARTICLES; i++) {
        const Particle* p = &scene->particles[i];
        if(p->active) {
            float alpha = p->life / p->max_life;
            glColor4f(p->color.x, p->color.y, p->color.z, alpha);
            glVertex3f(p->position.x, p->position.y, p->position.z);
        }
    }
    glEnd();
    glEnable(GL_LIGHTING);
}

void renderer_render_scene(const Scene* scene) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    gluLookAt(g_camera.pos.x, g_camera.pos.y, g_camera.pos.z,
              g_camera.pos.x + g_camera.front.x,
              g_camera.pos.y + g_camera.front.y,
              g_camera.pos.z + g_camera.front.z,
              g_camera.up.x, g_camera.up.y, g_camera.up.z);
              
    // Draw Floor Grid
    glDisable(GL_LIGHTING);
    glColor3f(0.3f, 0.3f, 0.3f);
    glBegin(GL_LINES);
    float sz = scene->world_size/2.0f;
    for(float i=-sz; i<=sz; i+=2.0f) {
        glVertex3f(i, -sz, -sz); glVertex3f(i, -sz, sz);
        glVertex3f(-sz, -sz, i); glVertex3f(sz, -sz, i);
    }
    glEnd();
    glEnable(GL_LIGHTING);
    
    // Draw Bodies
    for(int i=0; i<scene->body_count; i++) {
        draw_body(&scene->bodies[i]);
    }
    
    draw_particles(scene);
    
    glutSwapBuffers();
}