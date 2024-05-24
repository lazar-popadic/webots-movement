#ifndef __MAIN_HPP
#define __MAIN_HPP

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <math.h>
#include <cmath>
#include "MyRobot.hpp"
#include "pid.hpp"
#include <vector>

#define VEL_LIMIT 0.001
#define ANG_VEL_LIMIT 0.012
#define BEZIER_RESOLUTION 1000
#define POINT_DISTANCE 60

#define OFFS_ROBOT 400
#define OFFS_DESIRED 520

#define FORW 0
#define BACW 1

#define MAX_VEL 3.2
#define MAX_ANG_VEL 36.0

#define AVOID_DIS   500

using namespace webots;

typedef struct
{
    double x;
    double y;
} coord;

typedef struct
{
    coord pts[BEZIER_RESOLUTION];
    coord *equ_pts;
    int num_equ_pts;
    double dis;
    double max_ang_change;
} curve;

typedef struct
{
    int8_t finished : 1;
    int8_t success : 1;
} task;

typedef struct
{
    double p;
    double i;
    double d;
    double lmt;
    double ctrl;
    double ctrl_p;
    double ctrl_pp;
    double err_p;
    double err_sum;
    double err_dif;
    double sum_lmt;
}pid;

double get_ang_vel_ref();
double get_vel_ref();
void set_reg_type(int8_t type);
void acc_ramp(double *signal, double reference, double acc);
void saturation(double *signal, double max, double min);
void normalize_angle(double *angle);
double abs_max(double a, double b);
void scale_vel_ref(double *ref_1, double *ref_2, double limit);
int sign(double signal);

target create_target(double x, double y, double phi);

int create_curve(curve *curve_ptr, target desired_position, int dir);
int cubic_bezier_pts(curve *curve_ptr, coord p0, coord p1, coord p2, coord p3);
void equ_coords(curve *curve_ptr);
MyRobot get_robot();
void follow_curve();
void set_curve_ptr(curve* ptr);
curve* get_curve_ptr();
void move_on_path(double x, double y, double phi, int dir, bool cont, double cruising_vel);

void move();
void move_to_xy(double x, double y, int dir, double cruising_vel, double max_ang_vel);
void rot_to_angle(double phi, double max_ang_vel);
void move_on_dir(double distance, int dir, double cruising_vel);
void rot_to_xy(double x, double y, int dir, double max_ang_vel);
bool get_movement_status();
void movement_finished();
void movement_started();
target get_desired();
void set_desired_x(double x);
void set_desired_y(double y);
void set_desired_phi(double phi);
void set_dir(int tran_dir);
int get_dir();

double calculate(pid* pid_ptr, double err);
double calculate2(pid *pid_ptr, double ref, double val);
void init_pid(pid* pid_ptr, double p, double i, double d, double limit, double sum_limit);
void pid_init();
double abs_min3(double a, double b, double c);

void clear_cruising_vel();
void clear_max_ang_vel();
void set_cruising_vel(double vel);
void set_max_ang_vel(double vel);
double get_cruising_vel();
double get_max_ang_vel();
double vel_s_curve(double *vel, double prev_vel, double vel_ref, double jerk_slope);

coord get_obstacle();
double abs_max(double a, double b);
double abs_min(double a, double b);

#endif /* __MAIN_HPP */
