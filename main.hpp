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

#define VEL_LIMIT 0.1
#define ANG_VEL_LIMIT 1.2
#define BEZIER_RESOLUTION 1000
#define POINT_DISTANCE 40
#define MAX_CURVE_LENGTH 10000

#define OFFS_ROBOT 360
#define OFFS_DESIRED 720

using namespace webots;

typedef struct
{
    double x;
    double y;
} coord;

typedef struct
{
    coord points[BEZIER_RESOLUTION * 5];
    double distance;
    double min_curve_radius;
    int number_of_points_2;
    target end_target;
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

void create_curve(curve *bezier, target robot_position, target desired_position);
void cubic_bezier_curve(curve *bezier, coord p0, coord p1, coord p2, coord p3);
target create_target(double x, double y, double phi);

void equidistant_coords(coord* new_curve, int* number_of_points, double* distance, curve bezier);
void cubic_bezier_simple(curve *bezier, coord p0, coord p1, coord p2, coord p3);
void add_to_curve_2(curve *bezier, curve added_curve);
task follow_curve_3(coord* coords_to_follow, int number_of_points, double total_distance, target robot_position, bool not_moving);
void add_straight (curve *bezier, double distance);
#endif /* __MAIN_HPP */