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
int8_t position_prescaler = 4;
int8_t counter = 1;
double vel_ref = 0;
double ang_vel_ref = 0;
target targets[4] = {{750, 250, 90}, {750, 750, 180}, {250, 750, -90}, {250, 250, 0}};
int8_t i = 0;
bool break_controller = false;
curve curve_to_follow;
curve curve_to_add_1;
curve curve_to_add_2;
curve curve_to_add_3;

PID vel_loop(1.0, 1.0, 0.1, 6);
PID ang_vel_loop(0.1, 0.1, 0.0042, 12);

MyRobot robot_obj(-1250.0, -750.0, 0.0);

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

  create_curve(&curve_to_follow, robot_obj.get_position(), 0.0, 0.0, 0.0);
  create_curve(&curve_to_add_1, create_target(0, 0, 0), 0, -750, 180);
  create_curve(&curve_to_add_2, create_target(0, -750, 180), -750, -750, 180);
  create_curve(&curve_to_add_3, create_target(-750, -750, 180), 0, 0, 0);
  add_to_curve(&curve_to_follow, curve_to_add_1);
  add_to_curve(&curve_to_follow, curve_to_add_2);
  add_to_curve(&curve_to_follow, curve_to_add_3);

  while (my_robot->step(1) != -1)
  {
    if (!init)
    {
      init = true;
      robot_obj.set_prev(right_passive->getValue(), left_passive->getValue());
    }
    robot_obj.update_odom(right_passive->getValue(), left_passive->getValue());

    counter++;
    if (!(counter % position_prescaler))
    {
      counter = 1;
      // set_reg_type(1);
      // if (calculate(robot_obj.get_x(), robot_obj.get_y(), robot_obj.get_phi(), -1250.0, -250.0, 0.0, robot_obj.get_not_moving()))
      //   break_controller = true;
      // i ++;
      
      if (follow_curve_2(curve_to_follow, robot_obj.get_position()))
        break_controller = true;

      // if (follow_curve(robot_obj.get_position(), 0.0, 0.0, 0.0))
        // break_controller = true;

      // vel_ref = get_vel_ref();
      ang_vel_ref = get_ang_vel_ref();
      acc_ramp(&vel_ref, get_vel_ref(), 2.0);
      // acc_ramp(&ang_vel_ref, get_ang_vel_ref(), 24.0);

      // std::cout << "vel_ref  =  " << vel_ref << "                 ang_vel_ref  =  " << ang_vel_ref << std::endl;
    }

    robot_obj.is_moving(VEL_LIMIT, ANG_VEL_LIMIT);

    ang_vel_control = ang_vel_loop.calculate(ang_vel_ref, robot_obj.get_ang_vel());
    vel_control = vel_loop.calculate(vel_ref, robot_obj.get_vel());

    right_motor->setVelocity(vel_control + ang_vel_control);
    left_motor->setVelocity(vel_control - ang_vel_control);

    if (break_controller) // i >= sizeof(targets) / sizeof(target) || 
    {
      left_motor->setVelocity(0);
      right_motor->setVelocity(0);
      std::cout << " " << std::endl;
      std::cout << "x  =  " << robot_obj.get_x() << "     y  =  " << robot_obj.get_y() << "     phi  =  " << robot_obj.get_phi() << std::endl;
      std::cout << "time = " << my_robot->getTime() << std::endl;
      break;
    }
  };

  delete my_robot;
  return 0;
}