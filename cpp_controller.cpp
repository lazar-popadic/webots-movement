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
curve *curve_to_follow;
curve curve_2;
curve *curve_3;
target targets[2] = {{750, 750, 0}, {-750, 0, -90}}; // {-1250, -500, 90}
static int phase = 0;
coord *coords_to_follow;
int number_of_points;
double distance_2;

PID vel_loop(1.0, 1.0, 0.1, 2);
PID ang_vel_loop(0.1, 0.1, 0.0042, 2);

MyRobot robot_obj(0.0, 0.0, 0.0);
task current_task_status;

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
      switch (phase)
      {
      case 0:
        curve_to_follow = (curve *)malloc(sizeof(curve));

        create_curve(curve_to_follow, robot_obj.get_position(), create_target(750, -750, 0));

        curve_3 = (curve *)malloc(sizeof(curve));
        create_curve(curve_3, create_target(750, -750, 0), create_target(1250, -250, 90));
        add_to_curve_2(curve_to_follow, *curve_3);
        free(curve_3);
        curve_3 = (curve *)malloc(sizeof(curve));
        create_curve(curve_3, create_target(1250, -250, 90), create_target(1250, 250, 90));
        add_to_curve_2(curve_to_follow, *curve_3);
        curve_3 = (curve *)malloc(sizeof(curve));
        create_curve(curve_3, create_target(1250, 250, 90), create_target(0, 0, 180));
        add_to_curve_2(curve_to_follow, *curve_3);
        free(curve_3);

        coords_to_follow = (coord *)malloc((curve_to_follow->distance / POINT_DISTANCE + 1) * sizeof(coord));
        equidistant_coords(coords_to_follow, &number_of_points, &distance_2, *curve_to_follow);
        phase++;
        break;

      case 1:
        current_task_status = follow_curve_3(coords_to_follow, number_of_points, distance_2, robot_obj.get_position(), robot_obj.get_not_moving());
        if (current_task_status.finished)
        {
          free(curve_to_follow);
          free(coords_to_follow);
          // if (current_task_status.success)
          // {
            phase = 2;
            std::cout << "target 1 reached" << std::endl;
            std::cout << "x  =  " << robot_obj.get_x() << "     y  =  " << robot_obj.get_y() << "     phi  =  " << robot_obj.get_phi() << std::endl;
            std::cout << "time = " << my_robot->getTime() << std::endl;
          // }
          // else
          // {
          //   phase = 0;
          //   std::cout << "FAILED!" << std::endl;
          //   std::cout << "RETRYING..." << std::endl;
          //   std::cout << "time = " << my_robot->getTime() << std::endl;
          // }
          // break_controller = true;
        }
        break;

      case 2:
        curve_to_follow = (curve *)malloc(sizeof(curve));
        // create_curve_multi(curve_to_follow, robot_obj.get_position(), targets, sizeof(targets) / sizeof(*targets));
        create_curve(curve_to_follow, robot_obj.get_position(), create_target(0, 750, 0));

        curve_3 = (curve *)malloc(sizeof(curve));
        create_curve(curve_3, create_target(0, 750, 0), create_target(1250, 750, 0));
        add_to_curve_2(curve_to_follow, *curve_3);
        free(curve_3);

        coords_to_follow = (coord *)malloc((curve_to_follow->distance / POINT_DISTANCE + 1) * sizeof(coord));
        equidistant_coords(coords_to_follow, &number_of_points, &distance_2, *curve_to_follow);
        phase++;
        break;

      case 3:
        if (follow_curve_3(coords_to_follow, number_of_points, distance_2, robot_obj.get_position(), robot_obj.get_not_moving()).finished)
        {
          free(curve_to_follow);
          free(coords_to_follow);
          phase = 2;
          std::cout << "target 1 reached" << std::endl;
          std::cout << "x  =  " << robot_obj.get_x() << "     y  =  " << robot_obj.get_y() << "     phi  =  " << robot_obj.get_phi() << std::endl;
          std::cout << "time = " << my_robot->getTime() << std::endl;
          break_controller = true;
        }
        break;

      case 4:
        set_reg_type(1);
        if (calculate(robot_obj.get_x(), robot_obj.get_y(), robot_obj.get_phi(), 1250, 750, 0, robot_obj.get_not_moving()))
          phase++;
        break;

      case 5:
        set_reg_type(-1);
        if (calculate(robot_obj.get_x(), robot_obj.get_y(), robot_obj.get_phi(), 1250, 750, 0, robot_obj.get_not_moving()))
          break_controller = true;
        break;
      }

      // vel_ref = get_vel_ref();
      ang_vel_ref = get_ang_vel_ref();
      acc_ramp(&vel_ref, get_vel_ref(), 0.8);
      // acc_ramp(&ang_vel_ref, get_ang_vel_ref(), 10.0);

      // std::cout << "vel_ref  =  " << vel_ref << "                 ang_vel_ref  =  " << ang_vel_ref << std::endl;
    }

    robot_obj.is_moving(VEL_LIMIT, ANG_VEL_LIMIT);

    ang_vel_control = ang_vel_loop.calculate(ang_vel_ref, robot_obj.get_ang_vel());
    vel_control = vel_loop.calculate(vel_ref, robot_obj.get_vel());

    ang_vel_right = vel_control + ang_vel_control;
    ang_vel_left = vel_control - ang_vel_control;
    scale_vel_ref(&ang_vel_right, &ang_vel_left, 2);
    // std::cout << "ang_vel_right  =  " << ang_vel_right << "     ang_vel_left  =  " << ang_vel_left << std::endl;

    right_motor->setVelocity(ang_vel_right);
    left_motor->setVelocity(ang_vel_left);

    if (break_controller) // i >= sizeof(targets) / sizeof(target) ||
    {
      left_motor->setVelocity(0);
      right_motor->setVelocity(0);
      std::cout << "controller finished successfully " << std::endl;
      std::cout << "x  =  " << robot_obj.get_x() << "     y  =  " << robot_obj.get_y() << "     phi  =  " << robot_obj.get_phi() << std::endl;
      std::cout << "time = " << my_robot->getTime() << std::endl;
      break;
    }
  };

  delete my_robot;
  return 0;
}
