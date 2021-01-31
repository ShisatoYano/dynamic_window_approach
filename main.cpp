#include <bits/stdc++.h>
#include <algorithm>

using namespace std;

// simulation parameters
#define MAX_SPD_MS (1.0f)
#define MIN_SPD_MS (-0.5f)
#define MAX_YAWRATE_RS float(40.0 * M_PI / 180.0)
#define MAX_ACCEL_MS2 (0.2f)
#define MAX_DYAWRATE_RS2 float(40.0 * M_PI / 180.0)
#define V_RESO (0.01f)
#define YAWRATE_RESO float(0.1 * M_PI / 180.0)
#define DT_S (0.1f)
#define PREDICT_TIME_S (3.0f)
#define GOAL_COST_GAIN (1.0f)
#define SPD_COST_GAIN (1.0f)
#define ROBOT_RADIUS_M (1.0f)

struct State {
    float x_m;
    float y_m;
    float yaw_rad;
    float v_ms;
    float omega_rads;
};

struct Position {
    float x_m;
    float y_m;
};

struct Input {
    float v_ms;
    float omega_rads;
};

struct DynamicWindow {
    float min_v;
    float max_v;
    float min_yr;
    float max_yr;
};

struct InTraj {
    Input u;
    vector<State> traj;
};

DynamicWindow calc_dynamic_window(State *st)
{
    DynamicWindow dw = {0.0, 0.0, 0.0, 0.0};

    // from robot specification
    float vs[4] = {MIN_SPD_MS, MAX_SPD_MS, -MAX_YAWRATE_RS, MAX_YAWRATE_RS};

    // from motion model
    float vd[4] = {st->v_ms - MAX_ACCEL_MS2 * DT_S,
                   st->v_ms + MAX_ACCEL_MS2 * DT_S,
                   st->omega_rads - MAX_DYAWRATE_RS2 * DT_S,
                   st->omega_rads + MAX_DYAWRATE_RS2 * DT_S};

    dw.min_v = max(vs[0], vd[0]);
    dw.max_v = min(vs[1], vd[1]);
    dw.min_yr = max(vs[2], vd[2]);
    dw.max_yr = min(vs[3], vs[3]);

    return dw;
}

InTraj calc_final_input(State *st, Input *u, DynamicWindow *dw,
                        Position *goal, float obst[][2])
{
    InTraj best_u_traj;

    return best_u_traj;
}

InTraj dwa_control(State *st, Input *u, Position *goal, float obst[][2])
{
    DynamicWindow dw = calc_dynamic_window(st);

    InTraj u_traj = calc_final_input(st, u, &dw, goal, obst);

    return u_traj;
}

int main() {
    // initial state
    State st = {0.0, 0.0, M_PI / 8.0, 0.0, 0.0};

    // goal position
    Position goal = {10.0, 10.0};

    // obstacles
    float obst[10][2] = {
            {-1.0, -1.0},
            {0.0, 2.0},
            {4.0, 2.0},
            {5.0, 4.0},
            {5.0, 5.0},
            {5.0, 6.0},
            {5.0, 9.0},
            {8.0, 9.0},
            {7.0, 9.0},
            {12.0, 12.0}
    };

    // initial control input
    Input u = {0.0, 0.0};

    // initial state history
    vector<State> trajectory;
    trajectory.push_back(st);

    InTraj u_ltraj;

    // simulation process
    for (int i = 0; i < 1000; ++i) {
        u_ltraj = dwa_control(&st, &u, &goal, obst);
    }

    return 0;
}
