#include "main.hpp"

#ifndef __MYROBOT_HPP
#define __MYROBOT_HPP

class MyRobot
{
  private:
    double x_base;
    double y_base;
    double phi_base;    //degrees

    double cur_right_encoder = 0;
    double cur_left_encoder = 0;
    double prev_right_encoder = 0;
    double prev_left_encoder = 0;
    double ang_vel_r = 0;
    double ang_vel_l = 0;
    double vel_r = 0;
    double vel_l = 0;

    double radius_l = 33;       // mm
    double radius_r = 33;       // mm
    double wheel_dis = 178;     // mm

    double base_vel = 0;        // mm per ms = m per s
    double base_ang_vel = 0;    // degrees per ms
    bool not_moving = 1;

  public:
    MyRobot(double x_init, double y_init, double phi_init)
    {
        x_base = x_init;
        y_base = y_init;
        phi_base = phi_init;
    }

    void set_prev(double prev_right, double prev_left)
    {
        cur_right_encoder = prev_right;
        cur_left_encoder = prev_left;
    }

    void update_odom(double right_enc, double left_enc)
    {
        prev_right_encoder = cur_right_encoder;
        prev_left_encoder = cur_left_encoder;
        cur_right_encoder = right_enc;
        cur_left_encoder = left_enc;

        ang_vel_r = (cur_right_encoder - prev_right_encoder) / 1;
        ang_vel_l = (cur_left_encoder - prev_left_encoder) / 1;
        vel_r = ang_vel_r * radius_r;
        vel_l = ang_vel_l * radius_l;

        base_vel = (vel_r + vel_l) * 0.5;
        base_ang_vel = (vel_r - vel_l) / wheel_dis;

        x_base += base_vel * cos(phi_base/180*M_PI + base_ang_vel * 0.5);
        y_base += base_vel * sin(phi_base/180*M_PI + base_ang_vel * 0.5);
        phi_base += base_ang_vel * 180/M_PI;
        normalize_phi();
        // phi_base = normalize_angle(phi_base);
    }

    double get_phi()
    {
        return phi_base;
    }

    void normalize_phi()
    {
    if (phi_base > 180.0)
        phi_base -= 360.0;
    else if (phi_base < -180.0)
        phi_base += 360.0;
    }

    double get_x()
    {
        return x_base;
    }

    double get_y()
    {
        return y_base;
    }

    double get_vel()
    {
        return base_vel;
    }

    double get_ang_vel()
    {
        return base_ang_vel * 1000;
    }

    void is_moving(double vel_limit, double ang_vel_limit)
    {
        if (base_vel < vel_limit && base_ang_vel < ang_vel_limit)
            this->not_moving = true;
        else
            this->not_moving = false;
    }

    bool get_not_moving()
    {
        return not_moving;
    }
};

#endif