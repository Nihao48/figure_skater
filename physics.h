#ifndef PHYSICS_H
#define PHYSICS_H

/*
 *   state[0]  x, state[1] y      положение фигуриста на льду
 *   state[2]  vx, state[3] vy    скорость
 *   state[4]  z, state[5] vz     высота центра масс (ЦМ)
 *   state[6]  alpha, state[7]    боковой наклон и его скорость
 *   state[8]  theta, state[9]    наклон корпуса вперёд и его скорость
 *   state[10] psi_up             угол поворота верха тела (плечи, руки)
 *   state[11] psi_low            угол поворота низа тела (ноги, конёк)
 *   state[12] L_up               момент импульса верха
 *   state[13] L_low              момент импульса низа
 */

#define N_STATE 14

typedef enum {
    STATE_SKATING,
    STATE_TURN,
    STATE_PREPARE_JUMP,
    STATE_TOE_PICK,
    STATE_JUMPING,
    STATE_LANDING
} SkaterState;

typedef struct {
    double t;
    double stateTimer;
    SkaterState state;

    int direction;
    double max_alpha;
    double h_target;
    double theta_target;
    int backward;
    int turnToJump;

    int pushing;
    double pushStart;

    double pickX, pickY;
    double nx, ny;
    double psiHold;

    double tuck;
    double tilt;

    double totalDistance;
    int newTrailSegment;

    int jumpDone;
    double v_entry, z_takeoff, z_max, vz_takeoff, L_takeoff, w_shoulders;
    double flightTime, jumpAngle;
} PhysicsState;

typedef struct {
    double g;
    double mu;
    double max_alpha;
    double R_min;
    double react_w;
    double shift_max;
    double F_push;
    double push_time;
    double M_ice_max;
    double k_twist;
    double phi_twist;
    double k_hip;

    int jumpDir;
    double alpha_jump;
    double entry_time;
    double theta_entry;
    double v_kick;
    double k_leg;
    double c_leg;
    double hip_offset;
    double hold_arm;
    double tuck_time;
} PhysicsParams;

typedef struct {
    double m, H;
    double k_arm, k_head, k_trunk;
    double L_arm, R, r_head;
    double h_stand, h_crouch;
    double L_leg, L_toe;
    double I_low;
} Anthropometry;

extern PhysicsState phy;
extern PhysicsParams params;
extern Anthropometry anthro;
extern double state[N_STATE];

void physicsInit(double v0);
void physicsStep(double dt);
void physicsDestroy(void);

void doPush(void);
void setDirection(int dir);
void startJump(void);

double getSpeed(void);
double wrapAngle(double a);
double smoothStep(double u);
const char* getStateName(void);

#endif
