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

double error_sum = 0;

int curve_cnt = 0;

double cur_segment_len = 0;
coord cur_error = {0, 0};
coord next_error = {0, 0};
coord previous_error = {0, 0};
double cur_dis_error = 0;
double cur_dis_error_projected = 0;
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

static void rotate(bool not_moving);
static void go_to_xy(bool not_moving);

task status;

task follow_curve(curve *curve_ptr)
{
    switch (phase)
    {
    case 0: // curve
        status.finished = false;
        status.success = false;
        cur_error.x = curve_ptr->equ_pts[curve_cnt].x - get_robot().get_position().x;
        cur_error.y = curve_ptr->equ_pts[curve_cnt].y - get_robot().get_position().y;
        next_error.x = curve_ptr->equ_pts[curve_cnt + 1].x - get_robot().get_position().x;
        next_error.y = curve_ptr->equ_pts[curve_cnt + 1].y - get_robot().get_position().y;

        error_phi_prim_1 = atan2(cur_error.y, cur_error.x) * 180 / M_PI - get_robot().get_position().phi;
        error_phi_prim_2 = atan2(next_error.y, next_error.x) * 180 / M_PI - get_robot().get_position().phi;
        normalize_angle(&error_phi_prim_1);
        normalize_angle(&error_phi_prim_2);

        distance = curve_ptr->dis - curve_cnt * POINT_DISTANCE;
        cur_dis_error = sqrt(cur_error.x * cur_error.x + cur_error.y * cur_error.y);
        cur_dis_error_projected = cur_dis_error * cos(error_phi_prim_1 * M_PI / 180);

        if (curve_cnt != curve_ptr->num_equ_pts)
            t = cur_dis_error_projected / POINT_DISTANCE;
        else
        {
            cur_segment_len = sqrt((curve_ptr->equ_pts[curve_cnt].x - curve_ptr->equ_pts[curve_cnt - 1].x) * (curve_ptr->equ_pts[curve_cnt].x - curve_ptr->equ_pts[curve_cnt - 1].x) + (curve_ptr->equ_pts[curve_cnt].y - curve_ptr->equ_pts[curve_cnt - 1].y) * (curve_ptr->equ_pts[curve_cnt].y - curve_ptr->equ_pts[curve_cnt - 1].y));
            t = cur_dis_error_projected / cur_segment_len;
        }
        saturation(&t, 1, 0);

        error_phi_prim = t * error_phi_prim_1 + (1 - t) * error_phi_prim_2;
        normalize_angle(&error_phi_prim);
        vel_ref = bezier_distance_loop.calculate_zero(distance);
        ang_vel_ref = bezier_angle_loop.calculate_zero(error_phi_prim);

        if (cur_dis_error < POINT_DISTANCE * 1.0 && cur_dis_error_projected < 0)
        {
            error_sum += fabs(cur_dis_error * cos(error_phi_prim_1 * M_PI / 180));
            curve_cnt++;
            if (curve_cnt == curve_ptr->num_equ_pts)
            {
                phase = 1;
                curve_cnt = 0;
            }
        }
        if (cur_dis_error > POINT_DISTANCE * 4.0)
        {
            vel_ref = 0;
            ang_vel_ref = 0;
            phase = 1;
            curve_cnt = 0;
            status.finished = true;
            status.success = false;
        }
        break;

    case 1: // ending
        cur_error.x = curve_ptr->equ_pts[curve_ptr->num_equ_pts].x - get_robot().get_position().x;
        cur_error.y = curve_ptr->equ_pts[curve_ptr->num_equ_pts].y - get_robot().get_position().y;

        error_phi_prim = atan2(cur_error.y, cur_error.x) * 180 / M_PI - get_robot().get_position().phi;
        normalize_angle(&error_phi_prim);
        error_phi = atan2(curve_ptr->equ_pts[curve_ptr->num_equ_pts].y - curve_ptr->equ_pts[curve_ptr->num_equ_pts - 1].y, curve_ptr->equ_pts[curve_ptr->num_equ_pts].x - curve_ptr->equ_pts[curve_ptr->num_equ_pts - 1].x) * 180 / M_PI - get_robot().get_position().phi;
        normalize_angle(&error_phi);

        cur_dis_error = sqrt(cur_error.x * cur_error.x + cur_error.y * cur_error.y);
        cur_dis_error *= cos(error_phi_prim * M_PI / 180);
        vel_ref = distance_loop.calculate_zero(cur_dis_error);
        // vel_ref = 6;
        // vel_ref = distance_loop.calculate_zero(cur_dis_error + 100);        // dodaj jedan if sa ovako necim, samo ne budz

        t = cur_dis_error / POINT_DISTANCE;
        saturation(&t, 1, 0);

        error_phi_final = t * error_phi_prim + (1 - t) * error_phi;

        ang_vel_ref = angle_loop.calculate_zero(error_phi_final);

        if (cur_dis_error < 0)
        {
            error_sum += fabs(cur_dis_error * cos(error_phi_prim * M_PI / 180));
            status.finished = true;
            status.success = false;
            phase = 0;

            // std::cout << "error_sum    =  " << error_sum << "  mm" << std::endl;
            free(curve_ptr->equ_pts);
            free(curve_ptr);
            // std::cout << "memory has been freed" << std::endl;
        }
        break;
    }

    return status;
}

bool calculate(double robot_x, double robot_y, double robot_phi, double desired_x, double desired_y, double desired_phi, bool not_moving)
{
    done = false;
    error_x = desired_x - robot_x;
    error_y = desired_y - robot_y;
    error_phi = desired_phi - robot_phi;
    error_phi_prim = atan2(error_y, error_x) * 180 / M_PI - robot_phi;
    normalize_angle(&error_phi);
    normalize_angle(&error_phi_prim);
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
        vel_ref = 0;
        ang_vel_ref = 0;
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
            vel_ref = 0;
            ang_vel_ref = 0;
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
        *signal += sign(reference) * acc;
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

void scale_vel_ref(double *ref_1, double *ref_2, double limit)
{
    double abs_max_var = abs_max(*ref_1, *ref_2);
    if (abs_max_var > limit)
    {
        double factor = limit / abs_max_var;
        *ref_1 *= factor;
        *ref_2 *= factor;
    }
}

double abs_max(double a, double b)
{
    if (fabs(a) > fabs(b))
        return fabs(a);
    return fabs(b);
}

int sign(double signal)
{
    if (signal > 0)
        return 1;
    if (signal < 0)
        return -1;
    return 0;
}

void normalize_angle(double *angle)
{
    if (*angle > 180.0)
        *angle -= 360.0;
    else if (*angle < -180.0)
        *angle += 360.0;
}