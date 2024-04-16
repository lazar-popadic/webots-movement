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

#define VEL_LIMIT 1
#define ANG_VEL_LIMIT 10
#define BEZIER_RESOLUTION 4

using namespace webots;

typedef struct
{
    double x;
    double y;
    double phi;
}target;

typedef struct
{
    double x;
    double y;
}coord;

double get_ang_vel_ref();
double get_vel_ref();
void set_reg_type(int8_t type);
bool calculate(double robot_x, double robot_y, double robot_phi, double desired_x, double desired_y, double desired_phi, bool not_moving);
double cubic_bezier (coord bezier[], coord p0, coord p1, coord p2, coord p3);
bool follow_bezier(double robot_x, double robot_y, double robot_phi, double desired_x, double desired_y, double desired_phi, double offs_r, double offs_d1, double offs_d2);

#endif /* __MAIN_HPP */