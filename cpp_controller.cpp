// File:          cpp_controller.cpp
// Date:
// Description:
// Author:
// Modifications:

#include "main.hpp"
#include <iostream>

// All the webots classes are defined in the "webots" namespace
// using namespace webots;

bool init = false;
double vel_control = 0;
double ang_vel_control = 0;
double ang_vel_right = 0;
double ang_vel_left = 0;
int8_t position_prescaler = 4;
int8_t counter = 1;
double vel_ref = 0;
double ang_vel_ref = 0;
bool break_controller = false;

PID vel_loop(1.0, 1.0, 0.1, 12);
PID ang_vel_loop(0.1, 0.1, 0.0042, 12);
pid ang_vel_loop_2;
pid vel_loop_2;

MyRobot robot_obj(-1250.0, -750.0, 0.0);
task current_task_status;

curve *curve_ptr;

static int phase = 0;
target desired_position;
int dir;

double cruising_vel = 999, max_ang_vel = 999;
double vel_ramp, ang_vel_ramp;
double prev_vel = 0;
double cur_vel = 0;
double prev_ang_vel = 0;
double cur_ang_vel = 0;
coord obstacle = {0, 0};

int main(int argc, char **argv)
{
  webots::Robot *my_robot = new webots::Robot();

  Motor *left_motor = my_robot->getMotor("left wheel motor");
  Motor *right_motor = my_robot->getMotor("right wheel motor");
  PositionSensor *left_passive = my_robot->getPositionSensor("left wheel sensor");
  PositionSensor *right_passive = my_robot->getPositionSensor("right wheel sensor");

  left_motor->setPosition(HUGE_VAL);
  right_motor->setPosition(HUGE_VAL);
  left_motor->setVelocity(0);
  right_motor->setVelocity(0);
  right_passive->enable(1);
  left_passive->enable(1);

  init_pid(&ang_vel_loop_2, 0.1, 0.1, 0.0042, 12, 12);
  init_pid(&vel_loop_2, 1.0, 1.0, 0.1, 12, 12);
  pid_init();

  while (my_robot->step(1) != -1)
  {
    // INIT
    if (!init)
    {
      init = true;
      robot_obj.set_prev(right_passive->getValue(), left_passive->getValue());
    }
    // END_INIT

    // TIMER_ISR
    robot_obj.update_odom(right_passive->getValue(), left_passive->getValue());

    counter++;
    if (!(counter % position_prescaler))
    {
      counter = 1;
      move();
      vel_ref = get_vel_ref();
      ang_vel_ref = get_ang_vel_ref();
      // acc_ramp(&vel_ref, get_vel_ref(), 0.2);
      // acc_ramp(&ang_vel_ref, get_ang_vel_ref(), 2.4);
      // std::cout << "vel_ref      =  " << vel_ref << std::endl;

      cur_vel = robot_obj.get_vel();
      cur_ang_vel = robot_obj.get_ang_vel();
      vel_ref = sign(vel_ref) * abs_min3(vel_ref, cruising_vel, fabs(vel_s_curve(&cur_vel, prev_vel, vel_ref, 0.08)));
      // ang_vel_ref = sign(ang_vel_ref) * abs_min3(ang_vel_ref, max_ang_vel, fabs(vel_s_curve(&cur_ang_vel, prev_ang_vel, ang_vel_ref, 0.8)));
      // vel_ref = sign(vel_ref) * abs_min3(vel_ref, cruising_vel, 9999);
      ang_vel_ref = sign(ang_vel_ref) * abs_min3(ang_vel_ref, max_ang_vel, 9999);

      // std::cout << "vel_ref      =  " << vel_ref << std::endl;

      // std::cout << "cur_vel      =  " << cur_vel << std::endl;
      // std::cout << "prev_vel     =  " << prev_vel << std::endl;
      // std::cout << "               " << std::endl;

      // vel_ref = ;
      // ang_vel_ref = ;
      prev_vel = robot_obj.get_vel();
      prev_ang_vel = robot_obj.get_ang_vel();
    }

    robot_obj.is_moving(VEL_LIMIT, ANG_VEL_LIMIT);

    ang_vel_control = ang_vel_loop.calculate(ang_vel_ref, robot_obj.get_ang_vel());
    vel_control = vel_loop.calculate(vel_ref, robot_obj.get_vel());
    // ang_vel_control = calculate2(&ang_vel_loop_2, ang_vel_ref, robot_obj.get_ang_vel());
    // vel_control = calculate2(&vel_loop_2, vel_ref, robot_obj.get_vel());

    ang_vel_right = vel_control + ang_vel_control;
    ang_vel_left = vel_control - ang_vel_control;
    scale_vel_ref(&ang_vel_right, &ang_vel_left, 12);

    right_motor->setVelocity(ang_vel_right);
    left_motor->setVelocity(ang_vel_left);
    // END_TIMER_ISR

    // MAIN
    switch (phase)
    {
    case 0:
      move_on_path(0, 0, 90, FORW, true, MAX_VEL);
      phase = 1;
      break;

    case 1:
      if (!get_movement_status())
        phase = 2;
      break;

    case 2:
      move_on_path(500, 0, -90, FORW, false, MAX_VEL / 2);
      phase = 3;
      break;

    case 3:
      if (!get_movement_status())
        phase = 4;
      break;

    case 4:
      move_on_path(250, -500, 180, FORW, true, MAX_VEL);
      phase = 5;
      break;

    case 5:
      if (!get_movement_status())
        phase = 6;
      break;

    case 6:
      move_on_dir(250 * sqrt(2), FORW, MAX_VEL / 3);
      phase = 7;
      break;

    case 7:
      if (!get_movement_status())
        phase = 8;
      break;

    case 8:
      move_on_path(0, 0, 0, FORW, false, MAX_VEL);
      phase = 9;
      break;

    case 9:
      if (!get_movement_status())
        phase = 99;
      break;

    case 99:
      break_controller = true;
      break;
    }

    if (break_controller) // i >= sizeof(targets) / sizeof(target)
    {
      left_motor->setVelocity(0);
      right_motor->setVelocity(0);
      std::cout << " " << std::endl;
      std::cout << "controller finished successfully " << std::endl;
      std::cout << "x  =  " << robot_obj.get_x() << "     y  =  " << robot_obj.get_y() << "     phi  =  " << robot_obj.get_phi() << std::endl;
      std::cout << "time = " << my_robot->getTime() << std::endl;
      break;
    }
    // END_MAIN
  };

  delete my_robot;
  return 0;
}

MyRobot get_robot()
{
  return robot_obj;
}

target get_desired()
{
  return desired_position;
}

void set_desired_x(double x)
{
  desired_position.x = x;
}

void set_desired_y(double y)
{
  desired_position.y = y;
}

void set_desired_phi(double phi)
{
  desired_position.phi = phi;
}

void set_dir(int tran_dir)
{
  dir = tran_dir;
}

int get_dir()
{
  return dir;
}

curve *get_curve_ptr()
{
  return curve_ptr;
}

void set_curve_ptr(curve *ptr)
{
  curve_ptr = ptr;
}

void clear_cruising_vel()
{
  cruising_vel = 9999;
}

void clear_max_ang_vel()
{
  max_ang_vel = 9999;
}

void set_cruising_vel(double vel)
{
  cruising_vel = vel;
}

void set_max_ang_vel(double vel)
{
  max_ang_vel = vel;
}

double get_cruising_vel()
{
  return cruising_vel;
}

double get_max_ang_vel()
{
  return max_ang_vel;
}

coord get_obstacle()
{
  return obstacle;
}