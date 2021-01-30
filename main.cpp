#include <bits/stdc++.h>

using namespace std;

// simulation parameters
#define MAX_SPD_MS (1.0)
#define MIN_SPD_MS (-0.5)
#define MAX_YAWRATE_RS (40.0 * M_PI / 180.0)
#define MAX_ACCEL_MS2 (0.2)
#define MAX_DYAWRATE_RS2 (40.0 * M_PI / 180.0)
#define V_RESO (0.01)
#define YAWRATE_RESO (0.1 * M_PI / 180.0)
#define DT_S (0.1)
#define PREDICT_TIME_S (3.0)
#define GOAL_COST_GAIN (1.0)
#define SPD_COST_GAIN (1.0)
#define ROBOT_RADIUS_M (1.0)

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

    // simulation process
    for (int i = 0; i < 1000; ++i) {

    }

    return 0;
}
