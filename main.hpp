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
#define POINT_DISTANCE 50
#define MAX_CURVE_LENGTH 10000

#define OFFS_ROBOT 600
#define OFFS_DESIRED 1000

using namespace webots;

typedef struct
{
    double x;
    double y;
} coord;

typedef struct
{
    coord pts[BEZIER_RESOLUTION];
    coord* equ_pts;
    int num_equ_pts;
    double dis;
} curve;

typedef struct
{
    int8_t  finished:   1;
    int8_t  success:    1;
} task;

double get_ang_vel_ref();
double get_vel_ref();
void set_reg_type(int8_t type);
bool calculate(double robot_x, double robot_y, double robot_phi, double desired_x, double desired_y, double desired_phi, bool not_moving);
void acc_ramp(double *signal, double reference, double acc);
void saturation(double *signal, double max, double min);
void normalize_angle(double *angle);
double abs_max(double a, double b);
void scale_vel_ref(double *ref_1, double *ref_2, double limit);
int sign(double signal);

target create_target(double x, double y, double phi);
void add_straight (curve *bezier, double distance);

void create_curve(curve *curve_ptr, target desired_position);
void cubic_bezier_pts(curve *curve_ptr, coord p0, coord p1, coord p2, coord p3);
void equ_coords(curve *curve_ptr);
MyRobot get_robot();
task follow_curve(curve *curve_ptr);

#endif /* __MAIN_HPP */