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

// PID distance_loop(0.16, 0.04, 0.0, 10);
// PID angle_loop(2.5, 0.56, 0.0, 72);
PID distance_loop(0.1, 0.0, 0.0, 10);
PID angle_loop(1.0, 0.0, 0.0, 72);

// PID bezier_distance_loop(0.16, 0.04, 0.0, 10);
// PID bezier_angle_loop(2.0, 0.02, 0.08, 72);
PID bezier_distance_loop(0.004, 0.0, 0.0, 10);
PID bezier_angle_loop(2.0, 0.0, 0.0, 72);
double bezier_distance = 0;

static void rotate();
static void go_to_xy();

task status;
bool movement_status;

void follow_curve()
{
    switch (phase)
    {
    case 0: // curve
        status.finished = false;
        status.success = false;
        cur_error.x = get_curve_ptr()->equ_pts[curve_cnt].x - get_robot().get_position().x;
        cur_error.y = get_curve_ptr()->equ_pts[curve_cnt].y - get_robot().get_position().y;
        next_error.x = get_curve_ptr()->equ_pts[curve_cnt + 1].x - get_robot().get_position().x;
        next_error.y = get_curve_ptr()->equ_pts[curve_cnt + 1].y - get_robot().get_position().y;

        error_phi_prim_1 = atan2(cur_error.y, cur_error.x) * 180 / M_PI + get_dir() * 180 - get_robot().get_position().phi;
        error_phi_prim_2 = atan2(next_error.y, next_error.x) * 180 / M_PI + get_dir() * 180 - get_robot().get_position().phi;
        normalize_angle(&error_phi_prim_1);
        normalize_angle(&error_phi_prim_2);

        distance = get_curve_ptr()->dis - curve_cnt * POINT_DISTANCE;
        distance *= (get_dir() * (-2) + 1);
        cur_dis_error = sqrt(cur_error.x * cur_error.x + cur_error.y * cur_error.y);
        cur_dis_error_projected = cur_dis_error * cos(error_phi_prim_1 * M_PI / 180);

        // if (curve_cnt != curve_ptr->num_equ_pts)
        t = cur_dis_error_projected / POINT_DISTANCE;
        // else
        // {
        //     cur_segment_len = sqrt((curve_ptr->equ_pts[curve_cnt].x - curve_ptr->equ_pts[curve_cnt - 1].x) * (curve_ptr->equ_pts[curve_cnt].x - curve_ptr->equ_pts[curve_cnt - 1].x) + (curve_ptr->equ_pts[curve_cnt].y - curve_ptr->equ_pts[curve_cnt - 1].y) * (curve_ptr->equ_pts[curve_cnt].y - curve_ptr->equ_pts[curve_cnt - 1].y));
        //     t = cur_dis_error_projected / cur_segment_len;
        // }
        saturation(&t, 1, 0);
        // t = 1 / ( 1 + exp ( -10 * ( t - 0.5 ) ) );   // losije je sa ovim

        error_phi_prim = t * error_phi_prim_1 + (1 - t) * error_phi_prim_2;
        normalize_angle(&error_phi_prim);
        // vel_ref = bezier_distance_loop.calculate_zero(distance);
        vel_ref = 0.08 * distance;
        saturation(&vel_ref, 10, -10);
        // ang_vel_ref = bezier_angle_loop.calculate_zero(error_phi_prim);
        ang_vel_ref = 1.0 * error_phi_prim;
        saturation(&ang_vel_ref, 72, -72);
        // std::cout << "cur_dis_error    =  " << cur_dis_error << "  mm" << std::endl;

        if (cur_dis_error_projected < 0)
        {
            // error_sum += fabs(cur_dis_error * cos(error_phi_prim_1 * M_PI / 180));
            curve_cnt++;
            if (curve_cnt == get_curve_ptr()->num_equ_pts)
            {
                phase = 1;
                curve_cnt = 0;
            }
        }
        // if (cur_dis_error > POINT_DISTANCE * 1.2)
        // {
        //     vel_ref = 0;
        //     ang_vel_ref = 0;
        //     phase = 1;
        //     curve_cnt = 0;
        //     status.finished = true;
        //     status.success = false;
        // }
        break;

    case 1: // ending
        cur_error.x = get_curve_ptr()->equ_pts[get_curve_ptr()->num_equ_pts].x - get_robot().get_position().x;
        cur_error.y = get_curve_ptr()->equ_pts[get_curve_ptr()->num_equ_pts].y - get_robot().get_position().y;

        error_phi_prim = atan2(cur_error.y, cur_error.x) * 180 / M_PI + get_dir() * 180 - get_robot().get_position().phi;
        normalize_angle(&error_phi_prim);
        error_phi = atan2(get_curve_ptr()->equ_pts[get_curve_ptr()->num_equ_pts].y - get_curve_ptr()->equ_pts[get_curve_ptr()->num_equ_pts - 1].y, get_curve_ptr()->equ_pts[get_curve_ptr()->num_equ_pts].x - get_curve_ptr()->equ_pts[get_curve_ptr()->num_equ_pts - 1].x) * 180 / M_PI + get_dir() * 180 - get_robot().get_position().phi;
        normalize_angle(&error_phi);

        cur_dis_error = sqrt(cur_error.x * cur_error.x + cur_error.y * cur_error.y);
        cur_dis_error_projected = cur_dis_error * cos(error_phi_prim * M_PI / 180);
        // cur_dis_error *= (get_dir() * (-2) + 1);     //ovo ne treba
        vel_ref = 0.08 * distance * cur_dis_error;
        saturation(&vel_ref, 10, -10);
        // vel_ref = 6;
        // vel_ref = distance_loop.calculate_zero(cur_dis_error + 100);        // dodaj jedan if sa ovako necim, samo ne budz

        // t = cur_dis_error_projected / POINT_DISTANCE;
        cur_segment_len = sqrt((get_curve_ptr()->equ_pts[get_curve_ptr()->num_equ_pts].x - get_curve_ptr()->equ_pts[get_curve_ptr()->num_equ_pts - 1].x) * (get_curve_ptr()->equ_pts[get_curve_ptr()->num_equ_pts].x - get_curve_ptr()->equ_pts[get_curve_ptr()->num_equ_pts - 1].x) + (get_curve_ptr()->equ_pts[get_curve_ptr()->num_equ_pts].y - get_curve_ptr()->equ_pts[get_curve_ptr()->num_equ_pts - 1].y) * (get_curve_ptr()->equ_pts[get_curve_ptr()->num_equ_pts].y - get_curve_ptr()->equ_pts[get_curve_ptr()->num_equ_pts - 1].y));
        t = cur_dis_error_projected / cur_segment_len;
        saturation(&t, 1, 0);

        error_phi_final = t * error_phi_prim + (1 - t) * error_phi;

        // ang_vel_ref = angle_loop.calculate_zero(error_phi_final);
        ang_vel_ref = 1.0 * error_phi_final;
        saturation(&ang_vel_ref, 72, -72);

        if (cur_dis_error_projected < 0)
        {
            // error_sum += fabs(cur_dis_error * cos(error_phi_prim * M_PI / 180));
            status.finished = true;
            // if (cur_dis_error > POINT_DISTANCE * 1.2)
            //     status.success = false;
            // else
            status.success = true;
            phase = 0;
            movement_finished();

            // std::cout << "error_sum    =  " << error_sum << "  mm" << std::endl;
            free(get_curve_ptr()->equ_pts);
            free(get_curve_ptr());
            // std::cout << "memory has been freed" << std::endl;
        }
        break;
    }
}

void move()
{
    switch (reg_type)
    {
    case -1:
        rotate();
        break;
    case 1:
        go_to_xy();
        break;
    case 2:
        follow_curve();
        break;
    }
}

static void rotate()
{
    error_phi = get_desired().phi - get_robot().get_position().phi;
    normalize_angle(&error_phi);
    vel_ref = 0;
    ang_vel_ref = 1.0 * error_phi;
    saturation(&ang_vel_ref, 72, -72);
    if (fabs(error_phi) < PHI_LIMIT)
    {
        vel_ref = 0;
        ang_vel_ref = 0;
        set_reg_type(0);
        movement_finished();
    }
}

static void go_to_xy()
{
    error_x = get_desired().x - get_robot().get_position().x;
    error_y = get_desired().y - get_robot().get_position().y;
    error_phi_prim = atan2(error_y, error_x) * 180 / M_PI + get_dir() * 180 - get_robot().get_position().phi;
    normalize_angle(&error_phi_prim);

    switch (phase)
    {
    case 0: // rot2pos
        vel_ref = 0;
        ang_vel_ref = 1.0 * error_phi_prim;
        saturation(&ang_vel_ref, 72, -72);
        if (fabs(error_phi_prim) < PHI_PRIM_LIMIT)
            phase = 1;
        break;

    case 1: // tran
        distance = (get_dir() * (-2) + 1) * sqrt(error_x * error_x + error_y * error_y);
        if (fabs(error_phi_prim) > 90)
            vel_ref = -0.08 * distance;
        else
            vel_ref = 0.08 * distance;
        saturation(&vel_ref, 10, -10);
        if (fabs(distance) > DISTANCE_LIMIT2)
            ang_vel_ref = 1.0 * error_phi_prim;
        else
            ang_vel_ref = 0;
        saturation(&ang_vel_ref, 72, -72);
        if (fabs(distance) * cos(error_phi_prim) < 0)
        {
            vel_ref = 0;
            ang_vel_ref = 0;
            phase = 0;
            movement_finished();
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

void movement_started()
{
    movement_status = 1;
}

void movement_finished()
{
    movement_status = 0;
}

bool get_movement_status()
{
    return movement_status;
}

void move_to_xy (double x, double y, int dir)
{
  movement_started();
  set_reg_type(1);
  set_desired_x (x);
  set_desired_y (y);
  set_dir(dir);
}

void rot_to_angle (double phi)
{
  movement_started();
  set_reg_type(-1);
  set_desired_phi(phi);
}

void move_on_dir (double distance, int dir)
{
  movement_started();
  set_reg_type(1);
  set_desired_x (get_robot().get_position().x + (dir * (-2) + 1) * distance * cos (get_robot().get_position().phi * M_PI / 180));
  set_desired_y (get_robot().get_position().y + (dir * (-2) + 1) * distance * sin (get_robot().get_position().phi * M_PI / 180));
  set_dir(dir);
}

void rot_to_xy (double x, double y, int dir)
{
  movement_started();
  set_reg_type(-1);
  set_desired_phi(atan2(y - get_robot().get_position().y, x - get_robot().get_position().x) * 180 / M_PI + dir * 180);
  set_dir(dir);
}

void move_on_path (double x, double y, double phi, int dir)
{
    movement_started();
    set_reg_type(2);
    set_curve_ptr((curve *)malloc(sizeof(curve)));
    create_curve(get_curve_ptr(), create_target(x, y, phi), dir);
    set_dir(dir);
}