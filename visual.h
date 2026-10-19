#ifndef VISUAL_H
#define VISUAL_H

typedef struct {
    float distance;
    float height;
    float angleH;
} CameraState;

extern CameraState camera;

void visualInit(void);
void draw(void);
void resetCamera(void);

void startNewSegment(void);
void addTrailPoint(double x, double y);

#endif
