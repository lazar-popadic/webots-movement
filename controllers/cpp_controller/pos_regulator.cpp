#include "main.hpp"

#define DISTANCE_LIMIT 20
#define DISTANCE_LIMIT2 75
#define PHI_LIMIT 1
#define PHI_PRIM_LIMIT 2

extern MyRobot robot_obj;

static double vel_ref = 0;
static double ang_vel_ref = 0;

double error_x = 0;
double error_y = 0;
double error_phi = 0;
double error_phi_prim = 0;
double distance = 0;
double error_phi_prim_1 = 0;
double error_phi_prim_2 = 0;
double error_phi_final = 0;

double error_sum = 0;

int curve_cnt = 0;

double cur_segment_len = 0;
coord cur_error = {0, 0};
coord next_error = {0, 0};
coord previous_error = {0, 0};
double cur_dis_error = 0;
double cur_dis_error_projected = 0;
double next_dis_error = 0;
double t = 0;

bool done = true;
int8_t reg_type = 0;
int8_t phase = 0;

pid dist_loop;
pid ang_loop;
pid curve_ang_loop;
bool cont_move;

bool move_init = false;

void pid_init()
{
    init_pid(&dist_loop, 0.08, 0.0, 0.0, 4, 4);
    init_pid(&ang_loop, 1.2, 0.0, 0.0, 40, 40);
    init_pid(&curve_ang_loop, 0.6, 0.0, 0.0, 40, 40);
}

double bezier_distance = 0;

static void rotate();
static void go_to_xy();

task task_status;
bool movement_status;

void follow_curve()
{
    switch (phase)
    {
    case 0: // curve
        task_status.finished = false;
        task_status.success = false;
        cur_error.x = get_curve_ptr()->equ_pts[curve_cnt].x - get_robot().get_position().x;
        cur_error.y = get_curve_ptr()->equ_pts[curve_cnt].y - get_robot().get_position().y;
        next_error.x = get_curve_ptr()->equ_pts[curve_cnt + 1].x - get_robot().get_position().x;
        next_error.y = get_curve_ptr()->equ_pts[curve_cnt + 1].y - get_robot().get_position().y;

        error_phi_prim_1 = atan2(cur_error.y, cur_error.x) * 180 / M_PI + (get_dir() - 1) * 90 - get_robot().get_position().phi;
        error_phi_prim_2 = atan2(next_error.y, next_error.x) * 180 / M_PI + (get_dir() - 1) * 90 - get_robot().get_position().phi;
        normalize_angle(&error_phi_prim_1);
        normalize_angle(&error_phi_prim_2);

        distance = get_curve_ptr()->dis - curve_cnt * POINT_DISTANCE;
        distance *= get_dir();
        cur_dis_error = sqrt(cur_error.x * cur_error.x + cur_error.y * cur_error.y);
        cur_dis_error_projected = cur_dis_error * cos(error_phi_prim_1 * M_PI / 180);

        t = cur_dis_error_projected / POINT_DISTANCE;
        saturation(&t, 1, 0);

        error_phi_prim = t * error_phi_prim_1 + (1 - t) * error_phi_prim_2;
        normalize_angle(&error_phi_prim);
        vel_ref = calculate(&dist_loop, distance);
        ang_vel_ref = calculate(&curve_ang_loop, error_phi_prim);

        if (cur_dis_error_projected < 0)
        {
            curve_cnt++;
            if (curve_cnt == get_curve_ptr()->num_equ_pts)
            {
                phase = 1;
                curve_cnt = 0;
            }
        }
        // if (cur_dis_error > POINT_DISTANCE * 1.2)
        // {
        //     vel_ref = 0;
        //     ang_vel_ref = 0;
        //     phase = 1;
        //     curve_cnt = 0;
        //     status.finished = true;
        //     status.success = false;
        // }
        break;

    case 1: // ending
        cur_error.x = get_curve_ptr()->equ_pts[get_curve_ptr()->num_equ_pts].x - get_robot().get_position().x;
        cur_error.y = get_curve_ptr()->equ_pts[get_curve_ptr()->num_equ_pts].y - get_robot().get_position().y;

        error_phi_prim = atan2(cur_error.y, cur_error.x) * 180 / M_PI + (get_dir() - 1) * 90 - get_robot().get_position().phi;
        normalize_angle(&error_phi_prim);
        error_phi = atan2(get_curve_ptr()->equ_pts[get_curve_ptr()->num_equ_pts].y - get_curve_ptr()->equ_pts[get_curve_ptr()->num_equ_pts - 1].y, get_curve_ptr()->equ_pts[get_curve_ptr()->num_equ_pts].x - get_curve_ptr()->equ_pts[get_curve_ptr()->num_equ_pts - 1].x) * 180 / M_PI + (get_dir() - 1) * 90 - get_robot().get_position().phi;
        normalize_angle(&error_phi);

        cur_dis_error = sqrt(cur_error.x * cur_error.x + cur_error.y * cur_error.y);
        cur_dis_error_projected = cur_dis_error * cos(error_phi_prim * M_PI / 180);
        if (cont_move)
            vel_ref = get_dir() * get_cruising_vel();
        else
            vel_ref = get_dir() * calculate(&dist_loop, cur_dis_error_projected);
        cur_segment_len = sqrt((get_curve_ptr()->equ_pts[get_curve_ptr()->num_equ_pts].x - get_curve_ptr()->equ_pts[get_curve_ptr()->num_equ_pts - 1].x) * (get_curve_ptr()->equ_pts[get_curve_ptr()->num_equ_pts].x - get_curve_ptr()->equ_pts[get_curve_ptr()->num_equ_pts - 1].x) + (get_curve_ptr()->equ_pts[get_curve_ptr()->num_equ_pts].y - get_curve_ptr()->equ_pts[get_curve_ptr()->num_equ_pts - 1].y) * (get_curve_ptr()->equ_pts[get_curve_ptr()->num_equ_pts].y - get_curve_ptr()->equ_pts[get_curve_ptr()->num_equ_pts - 1].y));
        t = cur_dis_error_projected / cur_segment_len;
        saturation(&t, 1, 0);

        error_phi_final = t * error_phi_prim + (1 - t) * error_phi;
        ang_vel_ref = calculate(&curve_ang_loop, error_phi_final);

        if (cur_dis_error_projected < 0)
        {
            task_status.finished = true;
            task_status.success = true;
            phase = 0;
            set_reg_type(0);
            movement_finished();

            free(get_curve_ptr()->equ_pts);
            free(get_curve_ptr());
        }
        break;
    }
}

void move()
{
    switch (reg_type)
    {
    case -1:
        rotate();
        break;
    case 1:
        go_to_xy();
        break;
    case 2:
        follow_curve();
        break;
    }
}

static void rotate()
{
    error_phi = get_desired().phi - get_robot().get_position().phi;
    normalize_angle(&error_phi);
    vel_ref = 0;
    ang_vel_ref = calculate(&ang_loop, error_phi);
    if (fabs(error_phi) < PHI_LIMIT)
    {
        vel_ref = 0;
        ang_vel_ref = 0;
        set_reg_type(0);
        movement_finished();
    }
}

static void go_to_xy()
{
    error_x = get_desired().x - get_robot().get_position().x;
    error_y = get_desired().y - get_robot().get_position().y;
    error_phi_prim = atan2(error_y, error_x) * 180 / M_PI + (get_dir() - 1) * 90 - get_robot().get_position().phi;
    normalize_angle(&error_phi_prim);

    switch (phase)
    {
    case 0: // rot2pos
        vel_ref = 0;
        ang_vel_ref = calculate(&ang_loop, error_phi_prim);
        if (fabs(error_phi_prim) < PHI_PRIM_LIMIT)
            phase = 1;
        break;

    case 1: // tran
        distance = get_dir() * sqrt(error_x * error_x + error_y * error_y);
        if (fabs(error_phi_prim) > 90)
            vel_ref = -calculate(&dist_loop, distance* cos(error_phi_prim * M_PI / 180));
        else
        vel_ref = calculate(&dist_loop, distance* cos(error_phi_prim * M_PI / 180));
            // vel_ref = calculate(&dist_loop, distance);
        
        if (fabs(distance) > DISTANCE_LIMIT2)
            ang_vel_ref = calculate(&ang_loop, error_phi_prim);
        else
            ang_vel_ref = 0;
        if (fabs(distance) * cos(error_phi_prim) < 0)
        {
            vel_ref = 0;
            ang_vel_ref = 0;
            phase = 0;
            set_reg_type(0);
            movement_finished();
        }
        break;
    }
}

void set_reg_type(int8_t type)
{
    reg_type = type;
}

double get_vel_ref()
{
    return vel_ref;
}

double get_ang_vel_ref()
{
    return ang_vel_ref;
}

void acc_ramp(double *signal, double reference, double acc)
{
    if (fabs(reference) - fabs(*signal) > acc)
        *signal += sign(reference) * acc;
    else
        *signal = reference;
}

void saturation(double *signal, double max, double min)
{
    if (*signal > max)
        *signal = max;
    else if (*signal < min)
        *signal = min;
}

void scale_vel_ref(double *ref_1, double *ref_2, double limit)
{
    double abs_max_var = abs_max(*ref_1, *ref_2);
    if (abs_max_var > limit)
    {
        double factor = limit / abs_max_var;
        *ref_1 *= factor;
        *ref_2 *= factor;
    }
}

double abs_max(double a, double b)
{
    if (fabs(a) > fabs(b))
        return fabs(a);
    return fabs(b);
}

int sign(double signal)
{
    if (signal > 0)
        return 1;
    if (signal < 0)
        return -1;
    return 0;
}

void normalize_angle(double *angle)
{
    if (*angle > 180.0)
        *angle -= 360.0;
    else if (*angle < -180.0)
        *angle += 360.0;
}

void movement_started()
{
    movement_status = 1;
}

void movement_finished()
{
    movement_status = 0;
    clear_cruising_vel();
    clear_max_ang_vel();
}

bool get_movement_status()
{
    return movement_status;
}

double abs_min3(double a, double b, double c)
{
    double min = fabs(a);
    if (fabs(b) < min)
        min = b;
    if (fabs(c) < min)
        min = c;
    return min;
}

double abs_min(double a, double b)
{
    if (fabs(b) < fabs(a))
        return  b;
    return a;
}

double vel_s_curve(double *vel, double prev_vel, double vel_ref, double jerk_slope)
{
    double acc_approx = *vel - prev_vel;
    double acc_calc = vel_ref - *vel;
    double out = *vel;

    // std::cout << "vel           =  " << *vel << std::endl;
    // std::cout << "prev_vel      =  " << prev_vel << std::endl;
    // std::cout << "acc_approx    =  " << acc_approx << std::endl;
    // std::cout << "acc_calc      =  " << acc_calc << std::endl;

    if (fabs(vel_ref) > fabs(*vel))
    {
        if (fabs(acc_calc) - fabs(acc_approx) > jerk_slope)
            out = *vel + acc_approx + sign(vel_ref) * jerk_slope;
        else
            out = vel_ref;
    }
    // out *= sign(vel_ref);
    // std::cout << "sign(vel_ref) =  " << sign(vel_ref) << std::endl;
    // std::cout << "out           =  " << out << std::endl;
    // std::cout << "               " << std::endl;
    return out;
}

status move_on_path(double x, double y, double phi, int dir, bool cont, double cruising_vel)
{
    status move_status = RUNNING;
	if (!move_init)
	{
		move_init = true;
        movement_started();
        set_reg_type(2);
        set_curve_ptr((curve *)malloc(sizeof(curve)));
        create_curve(get_curve_ptr(), create_target(x, y, phi), dir);
        // if (create_curve(get_curve_ptr(), create_target(x, y, phi), dir))
            // return 1;    // TODO: ovde uradi testiranje drugih krivih
        set_dir(dir);
        cont_move = cont;
        set_cruising_vel(cruising_vel);
	}
	if(get_movement_status() == false)    // ovde treba da se poredi sa uspehom
	{
		move_init = false;
		move_status = SUCCESS;
	}
	else if(get_movement_status() == true)  // ovde treba da se poredi sa failom
	{
		// move_init = false;               // obavezno ovde reseuj move_init kada failuje
		move_status = FAILURE;
	}
	// else
	// 	move_status = RUNNING;

    return move_status;
}

status move_to_xy(double x, double y, int dir, double cruising_vel, double max_ang_vel)
{
    status move_status = RUNNING;
	if (!move_init)
	{
		move_init = true;
        movement_started();
        set_reg_type(1);
        set_desired_x(x);
        set_desired_y(y);
        set_dir(dir);
        set_cruising_vel(cruising_vel);
        set_max_ang_vel(max_ang_vel);
	}
	if(get_movement_status() == false)    // ovde treba da se poredi sa uspehom
	{
		move_init = false;
		move_status = SUCCESS;
	}
	else if(get_movement_status() == true)  // ovde treba da se poredi sa failom
	{
		// move_init = false;               // obavezno ovde reseuj move_init kada failuje
		move_status = FAILURE;
	}
	// else
	// 	move_status = RUNNING;

    return move_status;
}

status rot_to_angle(double phi, double max_ang_vel)
{
    status move_status = RUNNING;
	if (!move_init)
	{
		move_init = true;
        movement_started();
        set_reg_type(-1);
        set_desired_phi(phi);
        set_max_ang_vel(max_ang_vel);
	}
	if(get_movement_status() == false)    // ovde treba da se poredi sa uspehom
	{
		move_init = false;
		move_status = SUCCESS;
	}
	else if(get_movement_status() == true)  // ovde treba da se poredi sa failom
	{
		// move_init = false;               // obavezno ovde reseuj move_init kada failuje
		move_status = FAILURE;
	}
	// else
	// 	move_status = RUNNING;

    return move_status;
}

status rot_to_xy(double x, double y, int dir, double max_ang_vel)
{
    status move_status = RUNNING;
	if (!move_init)
	{
		move_init = true;
        movement_started();
        set_reg_type(-1);
        set_desired_phi(atan2(y - get_robot().get_position().y, x - get_robot().get_position().x) * 180 / M_PI + (dir - 1) * 90);
        set_dir(dir);
        set_max_ang_vel(max_ang_vel);
	}
	if(get_movement_status() == false)    // ovde treba da se poredi sa uspehom
	{
		move_init = false;
		move_status = SUCCESS;
	}
	else if(get_movement_status() == true)  // ovde treba da se poredi sa failom
	{
		// move_init = false;               // obavezno ovde reseuj move_init kada failuje
		move_status = FAILURE;
	}
	// else
	// 	move_status = RUNNING;

    return move_status;
}

status move_on_dir(double distance, int dir, double cruising_vel)
{
    status move_status = RUNNING;
	if (!move_init)
	{
		move_init = true;
        movement_started();
        set_reg_type(1);
        set_desired_x(get_robot().get_position().x + dir * distance * cos(get_robot().get_position().phi * M_PI / 180));
        set_desired_y(get_robot().get_position().y + dir * distance * sin(get_robot().get_position().phi * M_PI / 180));
        set_dir(dir);
        set_cruising_vel(cruising_vel);
	}
	if(get_movement_status() == false)    // ovde treba da se poredi sa uspehom
	{
		move_init = false;
		move_status = SUCCESS;
	}
	else if(get_movement_status() == true)  // ovde treba da se poredi sa failom
	{
		// move_init = false;               // obavezno ovde reseuj move_init kada failuje
		move_status = FAILURE;
	}
	// else
	// 	move_status = RUNNING;

    return move_status;
    
}