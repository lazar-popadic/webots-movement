#include "main.hpp"

int create_curve(curve *curve_ptr, target desired_position, int dir)
{
    curve_ptr->max_ang_change = 0;
    target robot_position = get_robot().get_position();
    static coord p0;
    static coord p1;
    static coord p2;
    static coord p3;

    normalize_angle(&desired_position.phi);
    p0.x = robot_position.x;
    p0.y = robot_position.y;

    p1.x = robot_position.x + OFFS_ROBOT * cos((robot_position.phi / 180 + dir * (-0.5) + 0.5) * M_PI);
    p1.y = robot_position.y + OFFS_ROBOT * sin((robot_position.phi / 180 + dir * (-0.5) + 0.5) * M_PI);

    p2.x = desired_position.x - OFFS_DESIRED * cos((desired_position.phi / 180 + dir * (-0.5) + 0.5) * M_PI);
    p2.y = desired_position.y - OFFS_DESIRED * sin((desired_position.phi / 180 + dir * (-0.5) + 0.5) * M_PI);

    p3.x = desired_position.x;
    p3.y = desired_position.y;

    curve_ptr->dis = 0;

    if (cubic_bezier_pts(curve_ptr, p0, p1, p2, p3))
        return 1;

    curve_ptr->equ_pts = (coord *)malloc((curve_ptr->dis / POINT_DISTANCE + 2) * sizeof(coord));

    equ_coords(curve_ptr);
    // std::cout << curve_ptr->max_ang_change << std::endl;
    return 0;
}

int cubic_bezier_pts(curve *curve_ptr, coord p0, coord p1, coord p2, coord p3)
{
    for (int i = 0; i < BEZIER_RESOLUTION; i++)
    {
        double t = (double)i / BEZIER_RESOLUTION;
        curve_ptr->pts[i].x = pow(1 - t, 3) * p0.x + 3 * t * pow(1 - t, 2) * p1.x + 3 * pow(t, 2) * (1 - t) * p2.x + pow(t, 3) * p3.x;
        curve_ptr->pts[i].y = pow(1 - t, 3) * p0.y + 3 * t * pow(1 - t, 2) * p1.y + 3 * pow(t, 2) * (1 - t) * p2.y + pow(t, 3) * p3.y;
        if (i > 0)
        {
            curve_ptr->dis += sqrt((curve_ptr->pts[i].x - curve_ptr->pts[i - 1].x) * (curve_ptr->pts[i].x - curve_ptr->pts[i - 1].x) + (curve_ptr->pts[i].y - curve_ptr->pts[i - 1].y) * (curve_ptr->pts[i].y - curve_ptr->pts[i - 1].y));
        }
        // if (sqrt((curve_ptr->pts[i].x - get_obstacle().x) * (curve_ptr->pts[i].x - get_obstacle().x) + (curve_ptr->pts[i].y - get_obstacle().y) * (curve_ptr->pts[i].y - get_obstacle().y)) < AVOID_DIS)
        // return 1;    // TODO: ovo dodaj
    }
    return 0;
}

void equ_coords(curve *curve_ptr)
{
    double temp_length = 0;
    double cur_dis = 0;
    curve_ptr->num_equ_pts = 0;
    curve_ptr->equ_pts[0] = curve_ptr->pts[0];
    for (int i = 1; i < BEZIER_RESOLUTION; i++)
    {
        cur_dis = sqrt((curve_ptr->pts[i].x - curve_ptr->pts[i - 1].x) * (curve_ptr->pts[i].x - curve_ptr->pts[i - 1].x) + (curve_ptr->pts[i].y - curve_ptr->pts[i - 1].y) * (curve_ptr->pts[i].y - curve_ptr->pts[i - 1].y));
        curve_ptr->dis += cur_dis;
        temp_length = sqrt((curve_ptr->equ_pts[curve_ptr->num_equ_pts].x - curve_ptr->pts[i].x) * (curve_ptr->equ_pts[curve_ptr->num_equ_pts].x - curve_ptr->pts[i].x) + (curve_ptr->equ_pts[curve_ptr->num_equ_pts].y - curve_ptr->pts[i].y) * (curve_ptr->equ_pts[curve_ptr->num_equ_pts].y - curve_ptr->pts[i].y));
        if (temp_length >= POINT_DISTANCE)
        {
            curve_ptr->num_equ_pts++;
            temp_length = 0;
            curve_ptr->equ_pts[curve_ptr->num_equ_pts] = curve_ptr->pts[i];
            // std::cout << curve_ptr->equ_pts[curve_ptr->num_equ_pts].x << "    " << curve_ptr->equ_pts[curve_ptr->num_equ_pts].y << std::endl;
            if (curve_ptr->num_equ_pts > 1)
            {
                curve_ptr->max_ang_change = abs_max (curve_ptr->max_ang_change, 180 / M_PI * (atan2(curve_ptr->equ_pts[curve_ptr->num_equ_pts].y - curve_ptr->equ_pts[curve_ptr->num_equ_pts - 1].y, curve_ptr->equ_pts[curve_ptr->num_equ_pts].x - curve_ptr->equ_pts[curve_ptr->num_equ_pts - 1].x) - atan2(curve_ptr->equ_pts[curve_ptr->num_equ_pts - 1].y - curve_ptr->equ_pts[curve_ptr->num_equ_pts - 2].y, curve_ptr->equ_pts[curve_ptr->num_equ_pts - 1].x - curve_ptr->equ_pts[curve_ptr->num_equ_pts - 2].x)));
            }
        }
    }
    // cur_dis = sqrt((curve_ptr->pts[BEZIER_RESOLUTION - 1].x - curve_ptr->equ_pts[curve_ptr->num_equ_pts].x) * (curve_ptr->pts[BEZIER_RESOLUTION - 1].x - curve_ptr->equ_pts[curve_ptr->num_equ_pts].x) + (curve_ptr->pts[BEZIER_RESOLUTION - 1].y - curve_ptr->equ_pts[curve_ptr->num_equ_pts].y) * (curve_ptr->pts[BEZIER_RESOLUTION - 1].y - curve_ptr->equ_pts[curve_ptr->num_equ_pts].y));
    // curve_ptr->num_equ_pts++;
    curve_ptr->equ_pts[curve_ptr->num_equ_pts] = curve_ptr->pts[BEZIER_RESOLUTION - 1];
    // std::cout << curve_ptr->equ_pts[curve_ptr->num_equ_pts].x << "    " << curve_ptr->equ_pts[curve_ptr->num_equ_pts].y << std::endl;
    // std::cout << curve_ptr->num_equ_pts << std::endl;
}

target create_target(double x, double y, double phi)
{
    target temp_target;
    temp_target.x = x;
    temp_target.y = y;
    temp_target.phi = phi;
    return temp_target;
}
