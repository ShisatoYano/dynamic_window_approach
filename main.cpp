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

State motion(State *st, Input *u)
{
    State st_ret;

    st_ret.yaw_rad = st->yaw_rad + u->omega_rads * DT_S;
    st_ret.x_m = st->x_m + u->v_ms * cos(st->yaw_rad) * DT_S;
    st_ret.y_m = st->y_m + u->v_ms * sin(st->yaw_rad) * DT_S;
    st_ret.v_ms = u->v_ms;
    st_ret.omega_rads = u->omega_rads;

    return st_ret;
}

vector<State> calc_trajectory(State *st_init, float v, float y)
{
    State st = {st_init->x_m, st_init->y_m, st_init->yaw_rad,
                st_init->v_ms, st_init->omega_rads};

    vector<State> traj;
    traj.push_back(st);

    Input u = {v, y};

    float time = 0.0;
    while (time <= PREDICT_TIME_S)
    {
        st = motion(&st, &u);
        traj.push_back(st);
        time += DT_S;
    }

    return traj;
}

float calc_to_goal_cost(vector<State> *traj, Position *goal)
{
    float goal_magnitude = sqrt(goal->x_m * goal->x_m + goal->y_m * goal->y_m);
    float traj_magnitude = sqrt(traj->back().x_m * traj->back().x_m
                                + traj->back().y_m * traj->back().y_m);
    float dot_product = (goal->x_m * traj->back().x_m) + (goal->y_m * traj->back().y_m);
    float error = dot_product / (goal_magnitude * traj_magnitude);
    float error_angle = acos(error);
    float cost = GOAL_COST_GAIN * error_angle;

    return cost;
}

float calc_obstacle_cost(vector<State> *traj, float obst[][2], int obst_num)
{
    int skip_n = 2;
    float min_r = INFINITY;

    float ox, oy, dx, dy, r;

    for (int i = 0; i < traj->size(); i+=skip_n) {
        for (int j = 0; j < obst_num; ++j) {
            ox = obst[j][0];
            oy = obst[j][1];
            dx = traj->at(i).x_m - ox;
            dy = traj->at(i).y_m - oy;

            r = sqrt(dx * dx + dy * dy);
            if (r <= ROBOT_RADIUS_M)
            {
                return INFINITY;
            }

            if (min_r >= r)
            {
                min_r = r;
            }
        }
    }

    return 1.0 / min_r;
}

InTraj calc_final_input(State *st, Input *u, DynamicWindow *dw,
                        Position *goal, float obst[][2], int obst_num)
{
    State st_init = {st->x_m, st->y_m, st->yaw_rad, st->v_ms, st->omega_rads};

    float min_cost = 10000.0;

    Input min_u = {u->v_ms, u->omega_rads};
    min_u.v_ms = 0.0;

    vector<State> best_traj;
    best_traj.push_back(st_init);

    // evaluate all trajectory with sampled input in dynamic window
    vector<State> traj;
    float to_goal_cost, speed_cost, obst_cost, final_cost;
    for (float v = dw->min_v; v < dw->max_v; v+=V_RESO) {
        for (float y = dw->min_yr; y < dw->max_yr; y+=YAWRATE_RESO) {
            traj = calc_trajectory(&st_init, v, y);

            // calculate cost
            to_goal_cost = calc_to_goal_cost(&traj, goal);
            speed_cost = SPD_COST_GAIN * (MAX_SPD_MS - traj.back().v_ms);
            obst_cost = calc_obstacle_cost(&traj, obst, obst_num);
            final_cost = to_goal_cost + speed_cost + obst_cost;

            // search minimum trajectory
            if (min_cost >= final_cost)
            {
                min_cost = final_cost;
                min_u.v_ms = v;
                min_u.omega_rads = y;
                best_traj = traj;
            }
        }
    }

    InTraj u_traj;
    u_traj.u = min_u;
    u_traj.traj = best_traj;

    return u_traj;
}

InTraj dwa_control(State *st, Input *u, Position *goal, float obst[][2], int obst_num)
{
    DynamicWindow dw = calc_dynamic_window(st);

    InTraj u_traj = calc_final_input(st, u, &dw, goal, obst, obst_num);

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
    int obst_num = sizeof(obst) / sizeof(*obst);

    // initial control input
    Input u = {0.0, 0.0};

    // initial state history
    vector<State> trajectory;
    trajectory.push_back(st);

    InTraj u_ltraj;

    // simulation process
    for (int i = 0; i < 1000; ++i) {
        u_ltraj = dwa_control(&st, &u, &goal, obst, obst_num);

        st = motion(&st, &u_ltraj.u);
        trajectory.push_back(st);

        // check goal
        if (sqrt((st.x_m - goal.x_m) * (st.x_m - goal.x_m) + (st.y_m - goal.y_m) * (st.y_m - goal.y_m)) <= ROBOT_RADIUS_M)
        {
            cout << "Goal!!" << endl;
            break;
        }
    }

    // print trajectory
    for (int i = 0; i < trajectory.size(); ++i) {
        cout << "(" << trajectory.at(i).x_m << ", " << trajectory.at(i).y_m << ")" << endl;
    }

    return 0;
}
