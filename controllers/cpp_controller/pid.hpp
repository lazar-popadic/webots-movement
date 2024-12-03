#include "main.hpp"

#ifndef __PID_HPP
#define __PID_HPP

class PID
{
    private:
        double Kp = 0;
        double Ki = 0;
        double Kd = 0;
        double control_limit = 0;
        double control = 0;
        double error = 0;
        double error_p = 0;
        double error_pp = 0;

        double signal_p = 0;
        double signal_pp = 0;

        void normalize_error()
        {
        if (error > 180.0)
           error -= 360.0;
        else if (error < -180.0)
            error += 360.0;
        }

    public:
        PID(double Kp, double Ki, double Kd, double limit)
        {
            this->Kp = Kp;
            this->Ki = Ki;
            this->Kd = Kd;
            this->control_limit = limit;
        }

        double calculate(double ref, double signal)
        {
            error = ref - signal;
            control += Kp * (error - error_p) + Ki * error_p + Kd * (error - 2 * error_p + error_pp);
            saturation();
            error_pp = error_p;
            error_p = error;
            return control;
        }

        double calculate_zero(double value)
        {
            error = value;
            control += Kp * (error - error_p) + Ki * error_p + Kd * (error - 2 * error_p + error_pp);
            saturation();
            error_pp = error_p;
            error_p = error;
            return control;
        }

        void saturation()
        {
            if (control > control_limit)
                control = control_limit;
            else if (control < -control_limit)
                control = -control_limit;
        }

        void clear_errors()
        {
            error_p = 0;
            error_pp = 0;
        }
};

#endif