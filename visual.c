#include "visual.h"
#include "physics.h"

#include <GL/gl.h>
#include <GL/glu.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.141592653589
#endif

#define MAX_TRAIL_POINTS 3000
#define MAX_TRAIL_SEGMENTS 200

typedef struct {
    double x[MAX_TRAIL_POINTS];
    double y[MAX_TRAIL_POINTS];
    int count;
} TrailSegment;

typedef struct {
    float h_r;
    float b_r;
    float L;
    float L_l;
    float L_a;
} Body;

static TrailSegment trailSegments[MAX_TRAIL_SEGMENTS];
static int trailSegmentCount = 0;
static int currentSegmentIndex = 0;

CameraState camera = {15.0f, 10.0f, 0.0f};
static Body skater = {0.25f, 0.075f, 1.25f, 1.75f, 1.5f};

static const int screen_w = 1080, screen_h = 720;

void startNewSegment(void) {
    if (trailSegmentCount < MAX_TRAIL_SEGMENTS) {
        trailSegments[trailSegmentCount].count = 0;
        currentSegmentIndex = trailSegmentCount;
        trailSegmentCount++;
    }
}

void addTrailPoint(double x, double y) {
    if (trailSegmentCount == 0) startNewSegment();
    TrailSegment* seg = &trailSegments[currentSegmentIndex];
    if (seg->count < MAX_TRAIL_POINTS) {
        seg->x[seg->count] = x;
        seg->y[seg->count] = y;
        seg->count++;
    }
}

void resetCamera(void) {
    camera.distance = 15.0f;
    camera.height = 10.0f;
    camera.angleH = 0.0f;
}

void drawCylinder(float radius, float height) {
    GLUquadric* q = gluNewQuadric();
    gluQuadricNormals(q, GLU_SMOOTH);
    gluQuadricDrawStyle(q, GLU_FILL);
    glColor3f(0.75f, 0.75f, 0.8f);
    gluCylinder(q, radius, radius, height, 20, 10);
    gluDeleteQuadric(q);
}

void drawSphere(float radius) {
    GLUquadric* q = gluNewQuadric();
    gluQuadricNormals(q, GLU_SMOOTH);
    gluQuadricDrawStyle(q, GLU_FILL);
    glColor3f(1.0f, 1.0f, 1.0f);
    gluSphere(q, radius, 30, 30);
    gluDeleteQuadric(q);
}

/* Фигурист: ноги поворачиваются вместе с коньком (psi_low),
   туловище, голова и руки - ещё на угол скрутки twist (psi_up - psi_low) */
   
void drawSkater(float hx_a, float hy_a, float px_a, float py_a, float l_a, float twist) {
    float c = (float)(M_PI / 180.0);
    float shoulder = skater.L_l + skater.L * 0.8f;
    float ex = skater.L_a/2 * cosf(hx_a*c) * cosf(hy_a*c);
    float ey = shoulder - (skater.L_a/2) * sinf(hx_a*c);
    float ez = (skater.L_a/2) * sinf(hy_a*c) * cosf(hx_a*c);

    glPushMatrix(); glTranslatef(-0.5f*skater.b_r, 0.0f, 0.0f); glRotatef(-90.0f, 1.0f, 0.0f, 0.0f); drawCylinder(skater.b_r/2, skater.L_l); glPopMatrix();
    glPushMatrix(); glTranslatef(0.5f*skater.b_r, skater.L_l, 0.0f); glRotatef(90.0f + l_a, 1.0f, 0.0f, 0.0f); drawCylinder(skater.b_r/2, skater.L_l); glPopMatrix();

    glPushMatrix();
    glRotatef(twist, 0.0f, 1.0f, 0.0f);
    glPushMatrix(); glTranslatef(0.0f, skater.L_l, 0.0f); glRotatef(-90.0f, 1.0f, 0.0f, 0.0f); drawCylinder(skater.b_r, skater.L); glPopMatrix();
    glPushMatrix(); glTranslatef(0.0f, skater.L_l + skater.L + skater.h_r, 0.0f); drawSphere(skater.h_r); glPopMatrix();

    glPushMatrix(); glTranslatef(-skater.b_r, shoulder, 0.0f); glRotatef(-90.0f + hy_a, 0.0f, 1.0f, 0.0f); glRotatef(hx_a, 1.0f, 0.0f, 0.0f); drawCylinder(skater.b_r/2, skater.L_a/2); glPopMatrix();
    glPushMatrix(); glTranslatef(-ex - skater.b_r, ey, ez); glRotatef(py_a, 0.0f, 1.0f, 0.0f); glRotatef(px_a, 1.0f, 0.0f, 0.0f); drawCylinder(skater.b_r/2, skater.L_a/2); glPopMatrix();
    glPushMatrix(); glTranslatef(-skater.b_r, shoulder, 0.0f); drawSphere(skater.h_r*0.3f); glPopMatrix();
    glPushMatrix(); glTranslatef(-ex - skater.b_r, ey, ez); drawSphere(skater.h_r*0.3f); glPopMatrix();

    glPushMatrix(); glTranslatef(skater.b_r, shoulder, 0.0f); glRotatef(90.0f - hy_a, 0.0f, 1.0f, 0.0f); glRotatef(hx_a, 1.0f, 0.0f, 0.0f); drawCylinder(skater.b_r/2, skater.L_a/2); glPopMatrix();
    glPushMatrix(); glTranslatef(ex + skater.b_r, ey, ez); glRotatef(-py_a, 0.0f, 1.0f, 0.0f); glRotatef(px_a + 10.0f, 1.0f, 0.0f, 0.0f); drawCylinder(skater.b_r/2, skater.L_a/2); glPopMatrix();
    glPushMatrix(); glTranslatef(skater.b_r, shoulder, 0.0f); drawSphere(skater.h_r*0.3f); glPopMatrix();
    glPushMatrix(); glTranslatef(ex + skater.b_r, ey, ez); drawSphere(skater.h_r*0.3f); glPopMatrix();
    glPopMatrix();
}

void drawTrail(void) {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_LIGHTING);
    glDepthMask(GL_FALSE);
    glLineWidth(3.0f);
    for (int seg = 0; seg < trailSegmentCount; seg++) {
        int n = trailSegments[seg].count;
        if (n < 2) continue;
        glBegin(GL_LINE_STRIP);
        for (int i = 0; i < n; i++) {
            float a = 1.0f - (float)(n - 1 - i) / (float)(n - 1) * 0.7f;
            glColor4f(0.55f, 0.85f, 1.0f, a);
            glVertex3f((float)trailSegments[seg].x[i], 0.01f, (float)trailSegments[seg].y[i]);
        }
        glEnd();
    }
    glDepthMask(GL_TRUE);
    glEnable(GL_LIGHTING);
    glDisable(GL_BLEND);
}

void drawFloor(float size, int divisions) {
    float half = size / 2.0f, step = size / divisions;
    glDisable(GL_LIGHTING);

    glColor3f(0.0f, 0.0f, 0.0f);
    glBegin(GL_QUADS);
    glVertex3f(-half, -0.002f, -half);
    glVertex3f(-half, -0.002f,  half);
    glVertex3f( half, -0.002f,  half);
    glVertex3f( half, -0.002f, -half);
    glEnd();

    glBegin(GL_LINES);
    glColor3f(0.18f, 0.18f, 0.22f);
    for (int i = 0; i <= divisions; i++) {
        float pos = -half + i * step;
        glVertex3f(-half, 0.0f, pos); glVertex3f(half, 0.0f, pos);
        glVertex3f(pos, 0.0f, -half); glVertex3f(pos, 0.0f, half);
    }
    glEnd();
    glEnable(GL_LIGHTING);
}

void visualInit(void) {
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
    glClearColor(0.02f, 0.02f, 0.03f, 1.0f);
    startNewSegment();
}

static float mix(float a, float b, float u) {
    return a + (b - a) * u;
}

void draw(void) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    double x = state[0], y = state[1];
    double alpha = state[6], theta = state[8];
    double psi_low = state[11];
    double twist = wrapAngle(state[10] - state[11]) * 180.0 / M_PI;
    double lift = 0.0;
    if (phy.state == STATE_JUMPING) lift = state[4] - phy.z_takeoff;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, (double)screen_w / (double)screen_h, 0.1, 300.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    float camAngle = camera.angleH * (float)M_PI / 180.0f;
    float camX = (float)x + camera.distance * sinf(camAngle);
    float camZ = (float)y + camera.distance * cosf(camAngle);
    gluLookAt(camX, camera.height, camZ, x, 0.5 + lift, y, 0.0, 1.0, 0.0);

    GLfloat light_pos[] = {5.0f, 10.0f, 5.0f, 1.0f};
    GLfloat la[] = {0.35f, 0.35f, 0.35f, 1.0f};
    GLfloat ld[] = {1.0f, 1.0f, 1.0f, 1.0f};
    glLightfv(GL_LIGHT0, GL_POSITION, light_pos);
    glLightfv(GL_LIGHT0, GL_AMBIENT, la);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, ld);

    drawFloor(1200.0f, 400);
    drawTrail();

    double psi_axis = (phy.state == STATE_JUMPING) ? phy.psiHold : psi_low;
    float side = phy.backward ? 1.0f : -1.0f;

    glPushMatrix();
    glTranslatef((float)x, (float)lift, (float)y);
    glRotatef((float)(psi_axis * 180.0 / M_PI), 0.0f, 1.0f, 0.0f);
    glRotatef((float)(alpha * 180.0 / M_PI) * side, 0.0f, 0.0f, 1.0f);
    glRotatef((float)(theta * 180.0 / M_PI), 1.0f, 0.0f, 0.0f);
    if (phy.state == STATE_JUMPING)
        glRotatef((float)((psi_low - psi_axis) * 180.0 / M_PI), 0.0f, 1.0f, 0.0f);

    if (phy.state == STATE_JUMPING) {
        float u = (float)smoothStep(phy.tuck);
        drawSkater(mix(0, 70, u), mix(0, 90, u), mix(0, -75, u), mix(-60, 110, u), 0, (float)twist);
    } else if (phy.state == STATE_TOE_PICK) {
        drawSkater(0, 0, 0, -60, 55, (float)twist);
    } else {
        float kick = 0.0f;
        if (phy.pushing) {
            double tau = phy.t - phy.pushStart;
            if (tau >= 0.0 && tau <= params.push_time)
                kick = (float)(45.0 * sin(M_PI * tau / params.push_time));
        }
        drawSkater(0, 0, 0, -60, 20 + kick, (float)twist);
    }
    glPopMatrix();
}
