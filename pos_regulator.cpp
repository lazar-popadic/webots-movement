#include "main.hpp"

#define DISTANCE_LIMIT 10
#define DISTANCE_LIMIT2 20
#define PHI_LIMIT 1
#define PHI_PRIM_LIMIT 2

static double vel_ref = 0;
static double ang_vel_ref = 0;

double error_x = 0;
double error_y = 0;
double error_phi = 0;
double error_phi_prim = 0;
double distance = 0;

coord p0;
coord p1;
coord p2;
coord p3;

coord bezier_coords[BEZIER_RESOLUTION];
int16_t bezier_counter = 1;

coord cur_error = {0, 0};
double cur_dis_error = 0;

bool done = true;
int8_t reg_type = 0;
int8_t phase = 0;

PID distance_loop(0.16, 0.052, 0.0, 6); //.......... 10
PID angle_loop(2.4, 0.72, 0.0, 72);      // 2.5, 0.75, .....

static void normalize_error_phi();
static void normalize_error_phi_prim();
static void rotate(bool not_moving);
static void go_to_xy(bool not_moving);

static void normalize_error_phi()
{
    if (error_phi > 180.0)
        error_phi -= 360.0;
    else if (error_phi < -180.0)
        error_phi += 360.0;
}

static void normalize_error_phi_prim()
{
    if (error_phi_prim > 180.0)
        error_phi_prim -= 360.0;
    else if (error_phi_prim < -180.0)
        error_phi_prim += 360.0;
}

bool follow_bezier(double robot_x, double robot_y, double robot_phi, double desired_x, double desired_y, double desired_phi, double offs_r, double offs_d)
{
    done = false;
    p0.x = robot_x;
    p0.y = robot_y;

    p3.x = desired_x;
    p3.y = desired_y;

    p1.x = robot_x + offs_r * cos(robot_phi);
    p1.y = robot_y + offs_r * sin(robot_phi);

    p2.x = desired_x - offs_d * cos(desired_phi);
    p2.y = desired_y - offs_d * sin(desired_phi);

    switch (phase)
    {
    case 0: // calculate all point
        cubic_bezier(bezier_coords, p0, p1, p2, p3);
        phase = 1;
        std::cout << "next x  =  " << bezier_coords[1].x << "                 next y  =  " << bezier_coords[1].y << std::endl;
        break;

    case 1: // follow
        error_x = desired_x - robot_x;
        error_y = desired_y - robot_y;
        cur_error.x = bezier_coords[bezier_counter].x - robot_x;
        cur_error.y = bezier_coords[bezier_counter].y - robot_y;

        error_phi_prim = atan2(cur_error.y, cur_error.x) * 180 / M_PI - robot_phi;
        normalize_error_phi_prim();

        distance = sqrt(error_x * error_x + error_y * error_y);
        cur_dis_error = sqrt(cur_error.x * cur_error.x + cur_error.y * cur_error.y);

        // if (fabs(error_phi_prim) > 180)
        //     vel_ref = -distance_loop.calculate_zero(distance);
        // else
            vel_ref = distance_loop.calculate_zero(distance);

        ang_vel_ref = angle_loop.calculate_zero(error_phi_prim);

        if (error_phi_prim < 2 && cur_dis_error < 100 )   // && cur_dis_error < BEZIER_RESOLUTION / bezier_counter * 20
        {
            if (bezier_counter == BEZIER_RESOLUTION)
            {
                done = true;
                phase = 1;
                bezier_counter = 0;
            }
            else
                bezier_counter++;
            std::cout << "next x  =  " << bezier_coords[bezier_counter].x << "                 next y  =  " << bezier_coords[bezier_counter].y << std::endl;
        }
        break;
    }

    return done;
}

bool calculate(double robot_x, double robot_y, double robot_phi, double desired_x, double desired_y, double desired_phi, bool not_moving)
{
    done = false;
    error_x = desired_x - robot_x;
    error_y = desired_y - robot_y;
    error_phi = desired_phi - robot_phi;
    normalize_error_phi();
    error_phi_prim = atan2(error_y, error_x) * 180 / M_PI - robot_phi;
    normalize_error_phi_prim();
    distance = sqrt(error_x * error_x + error_y * error_y);

    switch (reg_type)
    {
    case -1: // rotate
        rotate(not_moving);
        break;
    case 1: // go_to_xy
        go_to_xy(not_moving);
        break;
    }

    return done;
}

static void rotate(bool not_moving)
{
    vel_ref = 0;
    ang_vel_ref = angle_loop.calculate_zero(error_phi);
    if (fabs(error_phi) < PHI_LIMIT && not_moving)
    {
        reg_type = 0;
        done = true;
    }
}

static void go_to_xy(bool not_moving)
{
    switch (phase)
    {
    case 0: // rot2pos
        vel_ref = 0;
        ang_vel_ref = angle_loop.calculate_zero(error_phi_prim);
        if (fabs(error_phi_prim) < PHI_PRIM_LIMIT && not_moving)
            phase = 1;
        break;

    case 1: // tran
        if (fabs(error_phi_prim) > 180)
            vel_ref = -distance_loop.calculate_zero(distance);
        else
            vel_ref = distance_loop.calculate_zero(distance);
        if (fabs(distance) > DISTANCE_LIMIT2)
        {
            ang_vel_ref = angle_loop.calculate_zero(error_phi_prim);
            // ref[0]*=1.0;
        }
        else
            ang_vel_ref = 0;
        if (fabs(distance) < DISTANCE_LIMIT && not_moving)
        {
            phase = 0;
            reg_type = 0;
            done = true;
        }
        break;
    }
}

void set_reg_type(int8_t type)
{
    reg_type = type;
}

double get_vel_ref()
{
    return vel_ref;
}

double get_ang_vel_ref()
{
    return ang_vel_ref;
}