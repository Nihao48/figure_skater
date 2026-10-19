#include "physics.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG (M_PI / 180.0)

#define V_EPS    0.02
#define TURN_TOL 0.15

Anthropometry anthro = {
    .m = 69.4, .H = 1.722,
    .k_arm = 0.04946, .k_head = 0.06940, .k_trunk = 0.0,
    .L_arm = 0.636, .R = 0.20, .r_head = 0.10,
    .h_stand = 0.95, .h_crouch = 0.80,
    .L_leg = 1.05, .L_toe = 0.15,
    .I_low = 0.20
};

PhysicsParams params = {
    .g = 9.81,
    .mu = 0.006,
    .max_alpha = 20.0 * DEG,
    .R_min = 2.0,
    .react_w = 8.0,
    .shift_max = 0.15,
    .F_push = 230.0,
    .push_time = 0.45,
    .M_ice_max = 25.0,
    .k_twist = 400.0,
    .phi_twist = 30.0 * DEG,
    .k_hip = 600.0,

    .jumpDir = +1,
    .alpha_jump = 12.0 * DEG,
    .entry_time = 1.0,
    .theta_entry = 12.0 * DEG,
    .v_kick = 0.5,
    .k_leg = 40000.0,
    .c_leg = 1500.0,
    .hip_offset = 0.10,
    .hold_arm = 0.10,
    .tuck_time = 0.12
};

PhysicsState phy;
double state[N_STATE];

static int quiet = 0;

typedef struct {
    int n;
    double *k1, *k2, *k3, *k4;
    double *tmp;
} RK4Solver;

RK4Solver* rk4_init(int n) {
    RK4Solver *s = (RK4Solver*)malloc(sizeof(RK4Solver));
    s->n = n;
    s->k1 = (double*)malloc(sizeof(double) * n);
    s->k2 = (double*)malloc(sizeof(double) * n);
    s->k3 = (double*)malloc(sizeof(double) * n);
    s->k4 = (double*)malloc(sizeof(double) * n);
    s->tmp = (double*)malloc(sizeof(double) * n);
    return s;
}

void rk4_destroy(RK4Solver *s) {
    free(s->k1); free(s->k2); free(s->k3); free(s->k4); free(s->tmp);
    free(s);
}

void rk4_step(RK4Solver *s, double t, double *x, double h,
              void (*f)(double, double*, double*, void*), void *context) {
    int n = s->n;
    f(t, x, s->k1, context);
    for (int i = 0; i < n; i++) s->tmp[i] = x[i] + 0.5 * h * s->k1[i];
    f(t + 0.5 * h, s->tmp, s->k2, context);
    for (int i = 0; i < n; i++) s->tmp[i] = x[i] + 0.5 * h * s->k2[i];
    f(t + 0.5 * h, s->tmp, s->k3, context);
    for (int i = 0; i < n; i++) s->tmp[i] = x[i] + h * s->k3[i];
    f(t + h, s->tmp, s->k4, context);
    for (int i = 0; i < n; i++)
        x[i] += h * (s->k1[i] + 2.0*s->k2[i] + 2.0*s->k3[i] + s->k4[i]) / 6.0;
}

static RK4Solver *solver = NULL;

static double clamp(double v, double lo, double hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

double smoothStep(double u) {
    u = clamp(u, 0.0, 1.0);
    return u * u * (3.0 - 2.0 * u);
}

double wrapAngle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a <= -M_PI) a += 2.0 * M_PI;
    return a;
}

double getSpeed(void) {
    return sqrt(state[2]*state[2] + state[3]*state[3]);
}

static double armsTerm(void) {
    return 2.0 * anthro.k_arm * (anthro.L_arm*anthro.L_arm / (3.0*anthro.R*anthro.R) + 1.0);
}

static double headTerm(void) {
    return 0.4 * anthro.k_head * anthro.r_head*anthro.r_head / (anthro.R*anthro.R);
}

static double bookTwistMoment(double w0) {
    return anthro.m * anthro.R*anthro.R * w0 * (armsTerm() + headTerm() + 0.25 * anthro.k_trunk);
}

static double getIup(double tuck) {
    double mR2 = anthro.m * anthro.R * anthro.R;
    double I_open = mR2 * (armsTerm() + headTerm() + 0.3 * anthro.k_trunk) - anthro.I_low;
    double I_tuck = mR2 * (0.3 * (2.0*anthro.k_arm + anthro.k_trunk) + headTerm()) - anthro.I_low;
    return I_open + (I_tuck - I_open) * smoothStep(tuck);
}

static double arcCurvature(double alpha, double v) {
    if (v < V_EPS) return 0.0;
    double L = anthro.H, g = params.g;
    double D = v*v*v*v - 8.0/3.0 * alpha*alpha * g * v*v * L;
    if (D < 0.0) D = 0.0;
    return 2.0 * alpha * g / (v*v + sqrt(D));
}

static double inertiaForce(double alpha, double kappa, double v) {
    return anthro.m * v*v * kappa * (1.0 - fabs(kappa) * 0.5 * anthro.H * fabs(sin(alpha)));
}

static double inertiaArm(double alpha, double kappa) {
    double s = fabs(kappa) * anthro.H * fabs(sin(alpha));
    return anthro.H / 3.0 * (3.0 - 2.0*s) / (2.0 - s);
}

static double getTargetAlpha(double v) {
    double w = v / params.R_min;
    double limit = w*w * params.R_min / (params.g + 2.0/3.0 * w*w * anthro.H);
    return phy.direction * fmin(phy.max_alpha, limit);
}

static double leanAcceleration(double alpha, double alpha_dot, double v, double kappa) {
    double m = anthro.m, L = anthro.H, g = params.g, w = params.react_w;
    double I_A = m * L * L / 3.0;

    double M_gravity = m * g * 0.5 * L * sin(alpha);
    double M_inertia = -inertiaForce(alpha, kappa, v) * inertiaArm(alpha, kappa) * cos(alpha);
    double M_max = m * g * params.shift_max;
    double M_balance = clamp(I_A * (w*w * (getTargetAlpha(v) - alpha) - 2.0*w * alpha_dot), -M_max, M_max);

    return (M_gravity + M_inertia + M_balance) / I_A;
}

static double freeLeanAcceleration(double alpha, double alpha_dot) {
    double w = params.react_w;
    return -w*w * alpha - 2.0*w * alpha_dot;
}

static double pitchAcceleration(double theta, double theta_dot) {
    double m_tr = anthro.m * (2.0*anthro.k_arm + anthro.k_head + 0.5*anthro.k_trunk);
    double l = anthro.H - anthro.L_leg;
    double I_tr = m_tr * l * l / 3.0;
    double c_hip = 2.0 * sqrt(params.k_hip * I_tr);
    return (m_tr * params.g * 0.5 * l * sin(theta)
            + params.k_hip * (phy.theta_target - theta) - c_hip * theta_dot) / I_tr;
}

static double twistTorque(double *x) {
    double I_up = getIup(phy.tuck);
    double phi = wrapAngle(x[10] - x[11]);
    double w_rel = x[12] / I_up - x[13] / anthro.I_low;

    double phi_target = 0.0;
    if (phy.state == STATE_PREPARE_JUMP) phi_target = -params.jumpDir * params.phi_twist;
    if (phy.state == STATE_TOE_PICK)     phi_target =  params.jumpDir * params.phi_twist;

    double I_red = I_up * anthro.I_low / (I_up + anthro.I_low);
    double c = 2.0 * sqrt(params.k_twist * I_red);
    return params.k_twist * wrapAngle(phi_target - phi) - c * w_rel;
}

static double bladeTorque(double *x, double v, double kappa) {
    double w = params.react_w;
    double I_sum = getIup(phy.tuck) + anthro.I_low;
    double w_low = x[13] / anthro.I_low;
    double tau = -2.0 * w * I_sum * w_low;
    if (v > V_EPS) {
        double psi_target = atan2(x[2], x[3]) + (phy.backward ? M_PI : 0.0);
        tau = I_sum * (w*w * wrapAngle(psi_target - x[11]) + 2.0*w * (v * kappa - w_low));
    }
    return clamp(tau, -params.M_ice_max, params.M_ice_max);
}

static double legForce(double t, double *x, double *lx, double *ly, double *r) {
    double hipX = x[0] + params.jumpDir * params.hip_offset * phy.nx;
    double hipY = x[1] + params.jumpDir * params.hip_offset * phy.ny;
    *lx = hipX - phy.pickX;
    *ly = hipY - phy.pickY;
    *r = sqrt((*lx)*(*lx) + (*ly)*(*ly) + x[4]*x[4]);
    double r_dot = ((*lx)*x[2] + (*ly)*x[3] + x[4]*x[5]) / *r;

    double kick = fmin(params.v_kick * (t - phy.pushStart), anthro.L_toe);
    double kick_dot = (kick < anthro.L_toe) ? params.v_kick : 0.0;
    double F = params.k_leg * (anthro.L_leg + kick - *r) - params.c_leg * (r_dot - kick_dot);
    return fmax(F, 0.0);
}

void f_skating(double t, double *x, double *fx, void *context) {
    PhysicsParams *p = (PhysicsParams*)context;
    double vx = x[2], vy = x[3], z = x[4], vz = x[5];
    double alpha = x[6];
    double v = sqrt(vx*vx + vy*vy);

    double az = (p->k_leg * (phy.h_target - z) - p->c_leg * vz) / anthro.m;

    double kappa = arcCurvature(alpha, v);
    double a_centre = v * v * kappa;
    double Nb = anthro.m * fmax(p->g + az, 0.0);
    double Nn = fabs(inertiaForce(alpha, kappa, v));
    double N = sqrt(Nb*Nb + Nn*Nn);

    double F_push = 0.0;
    double tau = t - phy.pushStart;
    if (phy.pushing && tau >= 0.0 && tau <= p->push_time)
        F_push = p->F_push * sin(M_PI * tau / p->push_time);

    fx[0] = vx;
    fx[1] = vy;
    fx[2] = 0.0;
    fx[3] = 0.0;
    if (v > V_EPS) {
        double a_tangent = (F_push - p->mu * N) / anthro.m;
        fx[2] = a_tangent * vx / v + a_centre * vy / v;
        fx[3] = a_tangent * vy / v - a_centre * vx / v;
    } else if (F_push > 0.0) {
        double psi = x[11] + (phy.backward ? M_PI : 0.0);
        fx[2] = F_push / anthro.m * sin(psi);
        fx[3] = F_push / anthro.m * cos(psi);
    }
    fx[4] = vz;
    fx[5] = az;
    fx[6] = x[7];
    fx[7] = leanAcceleration(alpha, x[7], v, kappa);
    fx[8] = x[9];
    fx[9] = pitchAcceleration(x[8], x[9]);

    double M_twist = twistTorque(x);
    fx[10] = x[12] / getIup(phy.tuck);
    fx[11] = x[13] / anthro.I_low;
    fx[12] = M_twist;
    fx[13] = -M_twist + bladeTorque(x, v, kappa);
}

void f_toePick(double t, double *x, double *fx, void *context) {
    PhysicsParams *p = (PhysicsParams*)context;
    double lx, ly, r;
    double F = legForce(t, x, &lx, &ly, &r);
    double Fx = F * lx / r, Fy = F * ly / r, Fz = F * x[4] / r;

    fx[0] = x[2];
    fx[1] = x[3];
    fx[2] = Fx / anthro.m;
    fx[3] = Fy / anthro.m;
    fx[4] = x[5];
    fx[5] = Fz / anthro.m - p->g;
    fx[6] = x[7];
    fx[7] = freeLeanAcceleration(x[6], x[7]);
    fx[8] = x[9];
    fx[9] = pitchAcceleration(x[8], x[9]);

    double rx = p->jumpDir * p->hip_offset * phy.nx;
    double ry = p->jumpDir * p->hip_offset * phy.ny;
    double M_leg = ry * Fx - rx * Fy;

    double M_twist = twistTorque(x);
    double M_hold = clamp(M_twist, -p->hold_arm * F, p->hold_arm * F);

    fx[10] = x[12] / getIup(phy.tuck);
    fx[11] = x[13] / anthro.I_low;
    fx[12] = M_twist + M_leg;
    fx[13] = -M_twist + M_hold;
}

void f_jumping(double t, double *x, double *fx, void *context) {
    PhysicsParams *p = (PhysicsParams*)context;
    (void)t;
    fx[0] = x[2];
    fx[1] = x[3];
    fx[2] = 0.0;
    fx[3] = 0.0;
    fx[4] = x[5];
    fx[5] = -p->g;
    fx[6] = 0.0;
    fx[7] = 0.0;
    fx[8] = 0.0;
    fx[9] = 0.0;

    double M_twist = twistTorque(x);
    fx[10] = x[12] / getIup(phy.tuck);
    fx[11] = x[13] / anthro.I_low;
    fx[12] = M_twist;
    fx[13] = -M_twist;
}

void f_skater(double t, double *x, double *fx, void *context) {
    if (phy.state == STATE_TOE_PICK)      f_toePick(t, x, fx, context);
    else if (phy.state == STATE_JUMPING)  f_jumping(t, x, fx, context);
    else                                  f_skating(t, x, fx, context);
}

static void setState(SkaterState s) {
    phy.state = s;
    phy.stateTimer = 0.0;
}

void doPush(void) {
    if (phy.state != STATE_SKATING || phy.pushing) return;
    phy.pushing = 1;
    phy.pushStart = phy.t;
}

void setDirection(int dir) {
    if (phy.state == STATE_SKATING) phy.direction = dir;
}

void startJump(void) {
    if (phy.state != STATE_SKATING || getSpeed() < V_EPS) return;
    if (!quiet) printf("\nЛутц: v = %.2f м/с\n", getSpeed());
    phy.pushing = 0;
    phy.jumpDone = 0;
    phy.backward = 1;
    phy.turnToJump = 1;
    phy.max_alpha = params.alpha_jump;
    setState(STATE_TURN);
}

static void startToePick(void) {
    double v = getSpeed();
    double ux = state[2] / v, uy = state[3] / v;
    phy.v_entry = v;
    phy.nx = uy;
    phy.ny = -ux;
    double d = sqrt(anthro.L_leg*anthro.L_leg - state[4]*state[4]);
    phy.pickX = state[0] + d * ux + params.jumpDir * params.hip_offset * phy.nx;
    phy.pickY = state[1] + d * uy + params.jumpDir * params.hip_offset * phy.ny;
    phy.pushStart = phy.t;
    phy.theta_target = 0.0;
    setState(STATE_TOE_PICK);
}

static void takeOff(void) {
    phy.tilt = sqrt(state[6]*state[6] + state[8]*state[8]);
    phy.psiHold = state[11];
    state[7] = 0.0;
    state[9] = 0.0;
    state[12] /= cos(phy.tilt);
    state[13] /= cos(phy.tilt);

    phy.z_takeoff = state[4];
    phy.z_max = state[4];
    phy.vz_takeoff = state[5];
    phy.L_takeoff = state[12] + state[13];
    phy.w_shoulders = state[12] / getIup(0.0);
    phy.flightTime = 0.0;
    phy.jumpAngle = 0.0;
    phy.tuck = 0.0;
    phy.newTrailSegment = 1;
    setState(STATE_JUMPING);
}

static void printJumpReport(void) {
    if (quiet) return;
    double turns = fabs(phy.jumpAngle) / (2.0 * M_PI);
    int n = (int)floor(turns + 0.25);
    if (n > 5) n = 5;
    const char *names[] = {"не прыжок", "одинарный", "двойной", "тройной", "четверной", "пятерной"};
    double I_tuck = getIup(1.0) + anthro.I_low;
    printf("------------------------------------------------------------\n");
    printf(" v захода %.2f м/с, v_верт %.2f м/с, высота ЦМ %.2f м, полёт %.3f с\n",
           phy.v_entry, phy.vz_takeoff, phy.z_max - phy.z_takeoff, phy.flightTime);
    printf(" ось вращения наклонена на %.1f°\n", phy.tilt / DEG);
    printf(" L = %.2f кг·м²/с; плечи w0 = %.1f рад/с, книга (4.19): K = %.1f\n",
           phy.L_takeoff, fabs(phy.w_shoulders), bookTwistMoment(fabs(phy.w_shoulders)));
    printf(" в группировке w = L/I = %.1f рад/с (%.2f об/с)\n",
           fabs(phy.L_takeoff) / I_tuck, fabs(phy.L_takeoff) / I_tuck / (2.0 * M_PI));
    printf(" оборотов: %.2f -> %s лутц%s\n", turns, names[n], (n >= 1 && turns < n) ? " (недокрут)" : "");
    printf("------------------------------------------------------------\n");
}

static void land(void) {
    state[12] *= cos(phy.tilt);
    state[13] *= cos(phy.tilt);
    phy.jumpDone = 1;
    phy.direction = params.jumpDir;
    phy.h_target = anthro.h_stand;
    phy.newTrailSegment = 1;
    setState(STATE_LANDING);
    printJumpReport();
}

void updateJump(double dt) {
    switch (phy.state) {
    case STATE_SKATING:
        phy.h_target = anthro.h_stand;
        phy.theta_target = 0.0;
        if (phy.pushing && phy.t - phy.pushStart > params.push_time) phy.pushing = 0;
        break;

    case STATE_TURN: {
        double target = atan2(state[2], state[3]) + (phy.backward ? M_PI : 0.0);
        if (fabs(wrapAngle(target - state[11])) < TURN_TOL &&
            fabs(wrapAngle(state[10] - state[11])) < TURN_TOL) {
            if (phy.turnToJump) {
                phy.direction = -params.jumpDir;
                setState(STATE_PREPARE_JUMP);
            } else {
                phy.max_alpha = params.max_alpha;
                setState(STATE_SKATING);
            }
        }
        break;
    }

    case STATE_PREPARE_JUMP: {
        double s = smoothStep(phy.stateTimer / params.entry_time);
        phy.h_target = anthro.h_stand + (anthro.h_crouch - anthro.h_stand) * s;
        phy.theta_target = params.theta_entry * s;
        if (phy.stateTimer >= params.entry_time) startToePick();
        break;
    }

    case STATE_JUMPING: {
        double D = state[5]*state[5] + 2.0 * params.g * (state[4] - phy.z_takeoff);
        double timeLeft = (state[5] + sqrt(fmax(D, 0.0))) / params.g;
        if (state[5] < 0.0 && timeLeft < params.tuck_time) phy.tuck -= dt / params.tuck_time;
        else                                                phy.tuck += dt / params.tuck_time;
        phy.tuck = clamp(phy.tuck, 0.0, 1.0);
        break;
    }

    case STATE_LANDING:
        if (phy.stateTimer > params.entry_time) {
            phy.backward = 0;
            phy.turnToJump = 0;
            setState(STATE_TURN);
        }
        break;

    default:
        break;
    }
}

void physicsStep(double dt) {
    updateJump(dt);

    double psiBefore = state[11];
    rk4_step(solver, phy.t, state, dt, f_skater, &params);
    phy.t += dt;
    phy.stateTimer += dt;

    double v = getSpeed();
    if (phy.state != STATE_JUMPING && phy.state != STATE_TOE_PICK && v < V_EPS && !phy.pushing) {
        state[2] = 0.0;
        state[3] = 0.0;
    }
    phy.totalDistance += v * dt;

    if (phy.state == STATE_TOE_PICK) {
        double lx, ly, r;
        double F = legForce(phy.t, state, &lx, &ly, &r);
        if (F <= 0.0) {
            if (state[5] <= 0.0 && !quiet) printf("\nЛутц: срыв, нога не подбросила\n");
            takeOff();
        }
    } else if (phy.state == STATE_JUMPING) {
        phy.jumpAngle += wrapAngle(state[11] - psiBefore);
        phy.flightTime += dt;
        if (state[4] > phy.z_max) phy.z_max = state[4];
        if (state[4] <= phy.z_takeoff && state[5] < 0.0) land();
    }
}

void physicsInit(double v0) {
    anthro.k_trunk = 1.0 - 2.0 * anthro.k_arm - anthro.k_head;
    memset(&phy, 0, sizeof(phy));
    memset(state, 0, sizeof(state));
    phy.state = STATE_SKATING;
    phy.direction = 1;
    phy.max_alpha = params.max_alpha;
    phy.h_target = anthro.h_stand;
    state[3] = v0;
    state[4] = anthro.h_stand;
    if (solver == NULL) solver = rk4_init(N_STATE);
}

void physicsDestroy(void) {
    if (solver) rk4_destroy(solver);
    solver = NULL;
}

const char* getStateName(void) {
    switch (phy.state) {
        case STATE_SKATING:      return phy.pushing ? "Толчок" : "Скольжение";
        case STATE_TURN:         return "Тройка";
        case STATE_PREPARE_JUMP: return "Заход";
        case STATE_TOE_PICK:     return "ЗУБЕЦ";
        case STATE_JUMPING:      return "ПРЫЖОК";
        case STATE_LANDING:      return "Выезд";
    }
    return "";
}
