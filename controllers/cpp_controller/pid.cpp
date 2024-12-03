#include "main.hpp"

double calculate(pid *pid_ptr, double err)
{
    pid_ptr->err_sum += err;
    saturation(&(pid_ptr->err_sum), pid_ptr->sum_lmt, -pid_ptr->sum_lmt);
    pid_ptr->ctrl = pid_ptr->p * err + pid_ptr->i * pid_ptr->err_sum + pid_ptr->d * pid_ptr->err_dif;
    saturation(&(pid_ptr->ctrl), pid_ptr->lmt, -pid_ptr->lmt);
    pid_ptr->err_dif = err - pid_ptr->err_p;
    pid_ptr->err_p = err;
    return pid_ptr->ctrl;
}

double calculate2(pid *pid_ptr, double ref, double val)
{
    return calculate(pid_ptr, ref - val);
}

void init_pid(pid *pid_ptr, double p, double i, double d, double limit, double sum_limit)
{
    pid_ptr->p = p;
    pid_ptr->i = i;
    pid_ptr->d = d;
    pid_ptr->lmt = limit;
    pid_ptr->sum_lmt = sum_limit;
    pid_ptr->ctrl = 0;
    pid_ptr->ctrl_p = 0;
    pid_ptr->ctrl_pp = 0;
    pid_ptr->err_p = 0;
    pid_ptr->err_sum = 0;
    pid_ptr->err_dif = 0;
}