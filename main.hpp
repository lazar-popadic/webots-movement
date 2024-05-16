#ifndef __MAIN_HPP
#define __MAIN_HPP

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <cmath>
#include "MyRobot.hpp"
#include "pid.hpp"
#include <vector>
#include <math.h>

#define VEL_LIMIT 0.001
#define ANG_VEL_LIMIT 0.012
#define BEZIER_RESOLUTION 1000
#define POINT_DISTANCE 60

#define OFFS_ROBOT 520
#define OFFS_DESIRED 520

#define FORW 0
#define BACW 1

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
} curve;

typedef struct
{
    int8_t finished : 1;
    int8_t success : 1;
} task;

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

void create_curve(curve *curve_ptr, target desired_position, int dir);
void cubic_bezier_pts(curve *curve_ptr, coord p0, coord p1, coord p2, coord p3);
void equ_coords(curve *curve_ptr);
MyRobot get_robot();
void follow_curve();
void set_curve_ptr(curve* ptr);
curve* get_curve_ptr();
void move_on_path (double x, double y, double phi, int dir);

void move();
void move_to_xy(double x, double y, int dir);
void rot_to_angle(double phi);
void move_on_dir(double distance, int dir);
void rot_to_xy(double x, double y, int dir);
bool get_movement_status();
void movement_finished();
void movement_started();
target get_desired();
void set_desired_x(double x);
void set_desired_y(double y);
void set_desired_phi(double phi);
void set_dir(int tran_dir);
int get_dir();

#endif /* __MAIN_HPP */
