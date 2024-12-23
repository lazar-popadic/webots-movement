// File:          cpp_controller.cpp
// Date:
// Description:
// Author:
// Modifications:

#include "main.hpp"
#include <iostream>
// #include "matplotlibcpp.h"
// namespace plt = matplotlibcpp;

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

MyRobot robot_obj(1275.0, -125.0, 180.0);
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
double prev_vel_right = 0;
double prev_vel_left = 0;
double pid_ang_vel_right = 0;
double pid_ang_vel_left = 0;
double cur_ang_vel_right = 0;
double cur_ang_vel_left = 0;

// int8_t plt_cnt = 1;
// std::vector<double> v_vec, sim_time_vec, w_vec;
// double start_time = 0;

coord obstacle = {0, 0};
webots::Robot *my_robot = new webots::Robot();

int main(int argc, char **argv)
{
  //webots::Robot *my_robot = new webots::Robot();

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

  init_pid(&ang_vel_loop_2, 0.1, 0.1, 0.0042, 16, 16);
  init_pid(&vel_loop_2, 1.0, 1.0, 0.1, 16, 16);
  pid_init();
  
  // std::vector<double> x_plt, y_plt;
  // // plt::plot({1,3,2,4});
  // // plt::show();
  // plt::ion();
  // // plt::figure();
  // plt::figure_size(800, 400); // 800 pixels wide and 400 pixels high
  // plt::xlim(-1500,1500);
  // plt::ylim(-1000,1000);
  // plt::title("Robot trajectory");
  
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

      cur_vel = robot_obj.get_vel();
      cur_ang_vel = robot_obj.get_ang_vel();

      vel_ref = sign(vel_ref) * abs_min3(vel_ref, cruising_vel, fabs(vel_s_curve(&cur_vel, prev_vel, vel_ref, 0.08)));
      ang_vel_ref = sign(ang_vel_ref) * abs_min3(ang_vel_ref, max_ang_vel, fabs(vel_s_curve(&cur_ang_vel, prev_ang_vel, ang_vel_ref, 1)));

      prev_vel = robot_obj.get_vel();
      prev_ang_vel = robot_obj.get_ang_vel();
      // std::cout << "robot_obj.get_ang_vel()      =  " << robot_obj.get_ang_vel() << std::endl;
      // std::cout << "robot_obj.get_vel()      =  " << robot_obj.get_vel() << std::endl;
    }

    robot_obj.is_moving(VEL_LIMIT, ANG_VEL_LIMIT);

    ang_vel_control = ang_vel_loop.calculate(ang_vel_ref, robot_obj.get_ang_vel());
    vel_control = vel_loop.calculate(vel_ref, robot_obj.get_vel());

    pid_ang_vel_right = vel_control + ang_vel_control;
    pid_ang_vel_left = vel_control - ang_vel_control;
    scale_vel_ref(&pid_ang_vel_right, &pid_ang_vel_left, 16);

    cur_ang_vel_right = ang_vel_right;
    cur_ang_vel_left = ang_vel_left;

    ang_vel_right = abs_min(pid_ang_vel_right, vel_s_curve(&cur_ang_vel_right, prev_vel_right, pid_ang_vel_right, 0.04));
    ang_vel_left = abs_min(pid_ang_vel_left, vel_s_curve(&cur_ang_vel_left, prev_vel_left, pid_ang_vel_left, 0.04));

    prev_vel_right = cur_ang_vel_right;
    prev_vel_left = cur_ang_vel_left;

    right_motor->setVelocity(ang_vel_right);
    left_motor->setVelocity(ang_vel_left);
    // END_TIMER_ISR

    // MAIN
    switch (phase)
    {
    case 0:
      if (tactic_template() == SUCCESS)
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
      std::cout << "controller finished successfully " << std::endl;
      std::cout << "x  =  " << robot_obj.get_x() << "     y  =  " << robot_obj.get_y() << "     phi  =  " << robot_obj.get_phi() << std::endl;
      break;
    }
    // END_MAIN

    // PLOT
    // plt_cnt++;
    // if (!(plt_cnt %= 64))
    // {
    //   x_plt.push_back(robot_obj.get_position().x);
    //   y_plt.push_back(robot_obj.get_position().y);

    //   plt::clf();
    //   // sim_time_vec.push_back(sim_time);
    //   // v_vec.push_back(robot_obj.get_vel());
    //   // w_vec.push_back(robot_obj.get_ang_vel());
    //   // plt::plot(sim_time_vec, v_vec, {{"color", "blue"}, {"linewidth", "1"}});
    //   // plt::plot(sim_time_vec, w_vec, {{"color", "red"}, {"linewidth", "1"}});
    //   plt::xlim(-1500,1500);
    //   plt::ylim(-1000,1000);
    //   // plt::subplot(1,2,1);
    //   plt::plot(x_plt, y_plt, {{"color", "black"}, {"linewidth", "0.5"}});
    //   // plt::subplot(1,2,2);
    //   plt::scatter(std::vector<double>{robot_obj.get_position().x},std::vector<double>{robot_obj.get_position().y}, 64);
    //   plt::pause(0.001);
    //   plt::show();
    // }
    // END_PLOT
  };

  delete my_robot;
  return 0;
}

double get_time()
{
  return my_robot->getTime();
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

uint8_t delay_nb_2 (uint32_t *start_time, uint32_t delay_ms)
{
  *start_time = uint_min (*start_time, get_time ()*1000);

  if (get_time ()*1000 <= *start_time + delay_ms)
	  return 0;
  *start_time = 0xffffffff;
  return 1;
}

uint32_t
uint_min (uint32_t a, uint32_t b)
{
  if (a < b)
	  return a;
  return b;
}
