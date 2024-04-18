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

#define VEL_LIMIT           1
#define ANG_VEL_LIMIT       10
#define BEZIER_RESOLUTION   1000
#define POINT_DISTANCE      40.0
#define MAX_CURVE_LENGTH    10000

#define OFFS_ROBOT          240
#define OFFS_DESIRED_TRAN   120
#define OFFS_DESIRED        600

using namespace webots;

typedef struct
{
    double x;
    double y;
}coord;

typedef struct
{
    coord points[BEZIER_RESOLUTION];
    coord points_low_res[MAX_CURVE_LENGTH / (unsigned long)POINT_DISTANCE + 1];
    double distance;
    double min_curve_radius;
    int number_of_points;
    target end_target;
}curve;

double get_ang_vel_ref();
double get_vel_ref();
void set_reg_type(int8_t type);
bool calculate(double robot_x, double robot_y, double robot_phi, double desired_x, double desired_y, double desired_phi, bool not_moving);
void acc_ramp(double* signal, double reference, double acc);
void saturation (double* signal, double max, double min);
void normalize_angle(double *angle);

void create_curve(curve *bezier, target robot_position, target desired_position);
void cubic_bezier_curve (curve* bezier, coord p0, coord p1, coord p2, coord p3);
bool follow_curve (target robot_position, double desired_x, double desired_y, double desired_phi);
void add_to_curve(curve *bezier, curve added_curve);
bool follow_curve_2(curve bezier, target robot_position);
target create_target(double x, double y, double phi);
void create_curve_2(curve *bezier, target robot_position, target targets[], int number_of_targets);
#endif /* __MAIN_HPP */