#include "main.hpp"

#define DISTANCE_LIMIT 20
#define DISTANCE_LIMIT2 75
#define PHI_LIMIT 1
#define PHI_PRIM_LIMIT 2

extern MyRobot robot_obj;

static double vel_ref = 0;
static double ang_vel_ref = 0;

double error_x = 0;
double error_y = 0;
double error_phi = 0;
double error_phi_prim = 0;
double distance = 0;
double error_phi_prim_1 = 0;
double error_phi_prim_2 = 0;
double error_phi_final = 0;

coord p0;
coord p1;
coord p2;
coord p3;
coord p4;

coord bezier_coords[2 * BEZIER_RESOLUTION + 1];
coord bezier_coords_low_res[2 * BEZIER_RESOLUTION + 1];
int bezier_counter = 0;
int bezier_total = 0;

curve bezier_a;

coord cur_error = {0, 0};
coord next_error = {0, 0};
double cur_dis_error = 0;
double next_dis_error = 0;
double t = 0;

bool done = true;
int8_t reg_type = 0;
int8_t phase = 0;

PID distance_loop(0.16, 0.04, 0.0, 10);
PID angle_loop(2.5, 0.56, 0.0, 72);

PID bezier_distance_loop(0.16, 0.04, 0.0, 10);
PID bezier_angle_loop(2.0, 0.02, 0.08, 72);
double bezier_distance = 0;

static void normalize_error_phi();
static void normalize_error_phi_prim();
static void rotate(bool not_moving);
static void go_to_xy(bool not_moving);

void normalize_angle(double *angle)
{
    if (*angle > 180.0)
        *angle -= 360.0;
    else if (*angle < -180.0)
        *angle += 360.0;
}

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

bool follow_curve_2(curve bezier, target robot_position)
{
    done = false;

    switch (phase)
    {
    case 0: // curve
        cur_error.x = bezier.points_low_res[bezier_counter].x - robot_position.x;
        cur_error.y = bezier.points_low_res[bezier_counter].y - robot_position.y;
        next_error.x = bezier.points_low_res[bezier_counter + 1].x - robot_position.x;
        next_error.y = bezier.points_low_res[bezier_counter + 1].y - robot_position.y;

        error_phi_prim_1 = atan2(cur_error.y, cur_error.x) * 180 / M_PI - robot_position.phi;
        error_phi_prim_2 = atan2(next_error.y, next_error.x) * 180 / M_PI - robot_position.phi;
        normalize_angle(&error_phi_prim_1);
        normalize_angle(&error_phi_prim_2);

        distance = bezier.distance - bezier_counter * POINT_DISTANCE;
        cur_dis_error = sqrt(cur_error.x * cur_error.x + cur_error.y * cur_error.y);

        cur_dis_error *= cos(error_phi_prim_1 * M_PI / 180);
        t = cur_dis_error / POINT_DISTANCE;
        saturation(&t, 1, 0);

        error_phi_prim = t * error_phi_prim_1 + (1 - t) * error_phi_prim_2;
        normalize_error_phi_prim();
        vel_ref = bezier_distance_loop.calculate_zero(distance);
        ang_vel_ref = bezier_angle_loop.calculate_zero(error_phi_prim);

        if (cur_dis_error < 0)
        {
            bezier_counter++;
            // std::cout << "     x  =  " << robot_obj.get_x() << "                      y  =  " << robot_obj.get_y() << std::endl;
            // std::cout << "next x  =  " << bezier.points_low_res[bezier_counter].x << "    next y  =  " << bezier.points_low_res[bezier_counter].y << std::endl;
            // std::cout << "bezier_counter  =  " << bezier_counter << std::endl;
            if (bezier_counter == bezier.number_of_points - 1)
            {
                phase = 1;
                bezier_counter = 0;
                // std::cout << "final     x  =  " << bezier.end_target.x << "final                      y  =  " << bezier.end_target.y << std::endl;
            }
        }
        break;

    case 1: // ending
        error_x = bezier.end_target.x - robot_position.x;
        error_y = bezier.end_target.y - robot_position.y;

        error_phi_prim = atan2(error_y, error_x) * 180 / M_PI - robot_position.phi;
        normalize_error_phi_prim();
        error_phi = bezier.end_target.phi - robot_position.phi;
        normalize_error_phi();

        distance = sqrt(error_x * error_x + error_y * error_y);
        distance *= cos(error_phi_prim * M_PI / 180);
        vel_ref = distance_loop.calculate_zero(distance);

        t = distance / OFFS_DESIRED_TRAN;

        error_phi_final = t * error_phi_prim + (1 - t) * error_phi;

        ang_vel_ref = angle_loop.calculate_zero(error_phi_final);

        if (distance < 0)
        {
            done = true;
            phase = 0;

            error_x = bezier.end_target.x - robot_position.x;
            error_y = bezier.end_target.y - robot_position.y;
            distance = sqrt(error_x * error_x + error_y * error_y);
            std::cout << "final distance error =  " << distance << "  mm" << std::endl;
            std::cout << "final angle error    =  " << error_phi << "  degrees" << std::endl;
        }
        break;
    }

    return done;
}

bool follow_curve(target robot_position, double desired_x, double desired_y, double desired_phi)
{
    done = false;

    switch (phase)
    {
    case 0: // calculate all point
        normalize_angle(&desired_phi);
        p0.x = robot_position.x;
        p0.y = robot_position.y;

        p4.x = desired_x;
        p4.y = desired_y;

        p1.x = robot_position.x + OFFS_ROBOT * cos(robot_position.phi / 180 * M_PI);
        p1.y = robot_position.y + OFFS_ROBOT * sin(robot_position.phi / 180 * M_PI);

        p2.x = desired_x - (OFFS_DESIRED + OFFS_DESIRED_TRAN) * cos(desired_phi / 180 * M_PI);
        p2.y = desired_y - (OFFS_DESIRED + OFFS_DESIRED_TRAN) * sin(desired_phi / 180 * M_PI);

        p3.x = desired_x - OFFS_DESIRED_TRAN * cos(desired_phi / 180 * M_PI);
        p3.y = desired_y - OFFS_DESIRED_TRAN * sin(desired_phi / 180 * M_PI);

        cubic_bezier_curve(&bezier_a, p0, p1, p2, p3);
        phase = 1;
        // std::cout << "number of points  =  " << bezier_a.number_of_points << std::endl;
        break;

    case 1: // follow
        cur_error.x = bezier_a.points_low_res[bezier_counter].x - robot_position.x;
        cur_error.y = bezier_a.points_low_res[bezier_counter].y - robot_position.y;
        next_error.x = bezier_a.points_low_res[bezier_counter + 1].x - robot_position.x;
        next_error.y = bezier_a.points_low_res[bezier_counter + 1].y - robot_position.y;

        error_phi_prim_1 = atan2(cur_error.y, cur_error.x) * 180 / M_PI - robot_position.phi;
        error_phi_prim_2 = atan2(next_error.y, next_error.x) * 180 / M_PI - robot_position.phi;
        normalize_angle(&error_phi_prim_1);
        normalize_angle(&error_phi_prim_2);

        distance = bezier_a.distance - bezier_counter * POINT_DISTANCE;
        cur_dis_error = sqrt(cur_error.x * cur_error.x + cur_error.y * cur_error.y);

        cur_dis_error *= cos(error_phi_prim_1 * M_PI / 180);
        t = cur_dis_error / POINT_DISTANCE;
        saturation(&t, 1, 0);

        error_phi_prim = t * error_phi_prim_1 + (1 - t) * error_phi_prim_2;
        normalize_error_phi_prim();
        vel_ref = bezier_distance_loop.calculate_zero(distance);
        ang_vel_ref = bezier_angle_loop.calculate_zero(error_phi_prim);

        if (cur_dis_error < 0)
        {
            bezier_counter++;
            // std::cout << "     x  =  " << robot_obj.get_x() << "                      y  =  " << robot_obj.get_y() << std::endl;
            if (bezier_counter == bezier_a.number_of_points - 1)
            {
                phase = 2;
                bezier_counter = 0;
                // std::cout << "final     x  =  " << p4.x << "final                      y  =  " << p4.y << std::endl;
            }
        }
        break;

    case 2: // final straight
        error_x = p4.x - robot_position.x;
        error_y = p4.y - robot_position.y;

        error_phi_prim = atan2(error_y, error_x) * 180 / M_PI - robot_position.phi;
        normalize_error_phi_prim();
        error_phi = desired_phi - robot_position.phi;
        normalize_error_phi();

        distance = sqrt(error_x * error_x + error_y * error_y);
        distance *= cos(error_phi_prim * M_PI / 180);
        vel_ref = distance_loop.calculate_zero(distance);

        t = distance / OFFS_DESIRED_TRAN;

        error_phi_final = t * error_phi_prim + (1 - t) * error_phi;

        ang_vel_ref = angle_loop.calculate_zero(error_phi_final);

        if (distance < 0)
        {
            done = true;
            phase = 0;

            error_x = p4.x - robot_position.x;
            error_y = p4.y - robot_position.y;
            distance = sqrt(error_x * error_x + error_y * error_y);
            std::cout << "final distance error =  " << distance << std::endl;
            std::cout << "final angle error    =  " << error_phi << std::endl;
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
        if (fabs(error_phi_prim) > 90)
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

void acc_ramp(double *signal, double reference, double acc)
{
    if (fabs(reference) - fabs(*signal) > acc)
        *signal += acc;
    else
        *signal = reference;
}

void saturation(double *signal, double max, double min)
{
    if (*signal > max)
        *signal = max;
    else if (*signal < min)
        *signal = min;
}