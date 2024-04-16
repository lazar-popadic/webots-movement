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
int8_t position_prescaler = 10;
int8_t counter = 1;
double vel_ref = 0;
double ang_vel_ref = 0;
target targets[4] = {{750, 250, 90}, {750, 750, 180}, {250, 750, -90}, {250, 250, 0}};
int8_t i = 0;
bool brake_controller = false;

PID vel_loop(1.0, 1.0, 0.1, 6);
PID ang_vel_loop(0.1, 0.1, 0.0042, 6);

int main(int argc, char **argv)
{
  webots::Robot *my_robot = new webots::Robot();

  // get the time step of the current world.
  // int timeStep = (int)my_robot->getBasicTimeStep();

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

  MyRobot robot_obj(-1250.0, -750.0, 0.0);

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
      set_reg_type(1);
      // if (calculate(robot_obj.get_x(), robot_obj.get_y(), robot_obj.get_phi(), -1000.0, 500.0, 150, robot_obj.get_not_moving()))
      // brake_controller = true;
      //   i ++;

      if (follow_bezier(robot_obj.get_x(), robot_obj.get_y(), robot_obj.get_phi(), -1000.0, -500.0, 90.0, 250.0, 50.0, 450.0))
        brake_controller = true;

      vel_ref = get_vel_ref();
      ang_vel_ref = get_ang_vel_ref();

      // std::cout << "vel_ref  =  " << vel_ref << "                 ang_vel_ref  =  " << ang_vel_ref << std::endl;
    }

    robot_obj.is_moving(VEL_LIMIT, ANG_VEL_LIMIT);

    ang_vel_control = ang_vel_loop.calculate(ang_vel_ref, robot_obj.get_ang_vel());
    vel_control = vel_loop.calculate(vel_ref, robot_obj.get_vel());

    right_motor->setVelocity(vel_control + ang_vel_control);
    left_motor->setVelocity(vel_control - ang_vel_control);

    if (i >= sizeof(targets) / sizeof(target) || brake_controller)
    {
      left_motor->setVelocity(0);
      right_motor->setVelocity(0);
      std::cout << " "<< std::endl;
      std::cout << "x  =  " << robot_obj.get_x() << "     y  =  " << robot_obj.get_y() << std::endl;
      std::cout << "phi  =  " << robot_obj.get_phi() << std::endl;
      std::cout << " "<< std::endl;
      std::cout << "time = " << my_robot->getTime() << std::endl;
      break;
    }
  };

  delete my_robot;
  return 0;
}