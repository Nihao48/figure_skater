#include "physics.h"
#include "visual.h"

#include <SDL2/SDL.h>
#include <SDL2/SDL_opengl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static SDL_Window* window;
static double lastTrailTime = -1.0;

void printStats(void) {
    static int frameCount = 0;
    frameCount++;
    if (frameCount % 20 == 0) {
        printf("\r[%-10s] t=%6.2fс | V=%5.2f м/с | S=%7.1fм | α=%+5.1f° | θ=%4.1f° | z=%4.2fм | L=%+6.2f   ",
               getStateName(), phy.t, getSpeed(), phy.totalDistance,
               state[6] * 180.0 / M_PI, state[8] * 180.0 / M_PI, state[4], state[12] + state[13]);
        fflush(stdout);
    }
}

static int get_input(void) {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        switch (event.type) {
            case SDL_QUIT: return 0;
            case SDL_KEYDOWN:
                switch (event.key.keysym.sym) {
                    case SDLK_ESCAPE: return 0;
                    case SDLK_SPACE: startJump(); break;
                    case SDLK_UP:
                    case SDLK_e: doPush(); break;
                    case SDLK_LEFT: setDirection(+1); break;
                    case SDLK_RIGHT: setDirection(-1); break;
                    case SDLK_DOWN: setDirection(0); break;
                    case SDLK_w: camera.distance -= 0.5f; if (camera.distance < 3.0f) camera.distance = 3.0f; break;
                    case SDLK_s: camera.distance += 0.5f; if (camera.distance > 50.0f) camera.distance = 50.0f; break;
                    case SDLK_a: camera.angleH -= 5.0f; break;
                    case SDLK_d: camera.angleH += 5.0f; break;
                    case SDLK_r: resetCamera(); break;
                    default: break;
                }
                break;
            default: break;
        }
    }
    return 1;
}

static void updateTrail(void) {
    if (phy.newTrailSegment) {
        startNewSegment();
        phy.newTrailSegment = 0;
    }
    if (phy.state != STATE_JUMPING && phy.t - lastTrailTime >= 0.05) {
        addTrailPoint(state[0], state[1]);
        lastTrailTime = phy.t;
    }
}

int main(int argc, char* argv[]) {
    if (argc != 3) {
        printf("Использование: %s <скорость> <угол_град>\n", argv[0]);
        printf("Пример:        %s 4 20\n", argv[0]);
        printf("Проверки:      %s --test\n", argv[0]);
        return 1;
    }

    params.max_alpha = atof(argv[2]) * M_PI / 180.0;
    physicsInit(atof(argv[1]));
    printf("V=%.2f α=%.1f°\n", getSpeed(), atof(argv[2]));
    printf("↑/E — толчок, ←/→/↓ — дуга, пробел — лутц\n");

    SDL_Init(SDL_INIT_VIDEO);
    window = SDL_CreateWindow("Рррруская ракета", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                              1080, 720, SDL_WINDOW_OPENGL);
    SDL_GLContext ctx = SDL_GL_CreateContext(window);
    SDL_GL_SetSwapInterval(1);
    visualInit();

    const double dt = 0.001;
    Uint64 prev = SDL_GetPerformanceCounter();
    double accumulator = 0.0;

    while (get_input()) {
        Uint64 now = SDL_GetPerformanceCounter();
        accumulator += (double)(now - prev) / SDL_GetPerformanceFrequency();
        prev = now;
        if (accumulator > 0.1) accumulator = 0.1;

        while (accumulator >= dt) {
            physicsStep(dt);
            updateTrail();
            accumulator -= dt;
        }
        printStats();
        draw();
        SDL_GL_SwapWindow(window);
    }

    printf("\nКонец. t=%.2fс S=%.2fм\n", phy.t, phy.totalDistance);
    physicsDestroy();
    SDL_GL_DeleteContext(ctx);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
