#include "main.hpp"

void create_curve(curve *curve_ptr, target desired_position)
{
    target robot_position = get_robot().get_position();
    static coord p0;
    static coord p1;
    static coord p2;
    static coord p3;

    normalize_angle(&desired_position.phi);
    p0.x = robot_position.x;
    p0.y = robot_position.y;

    p1.x = robot_position.x + OFFS_ROBOT * cos(robot_position.phi / 180 * M_PI);
    p1.y = robot_position.y + OFFS_ROBOT * sin(robot_position.phi / 180 * M_PI);

    p2.x = desired_position.x - OFFS_DESIRED * cos(desired_position.phi / 180 * M_PI);
    p2.y = desired_position.y - OFFS_DESIRED * sin(desired_position.phi / 180 * M_PI);

    p3.x = desired_position.x;
    p3.y = desired_position.y;

    curve_ptr->dis = 0;

    cubic_bezier_pts(curve_ptr, p0, p1, p2, p3);

    curve_ptr->equ_pts = (coord *)malloc((curve_ptr->dis / POINT_DISTANCE + 2)* sizeof(coord));

    equ_coords(curve_ptr);
}

void cubic_bezier_pts(curve *curve_ptr, coord p0, coord p1, coord p2, coord p3)
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
    }
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
        temp_length += cur_dis;
        if (temp_length >= POINT_DISTANCE)
        {
            curve_ptr->num_equ_pts++;
            temp_length = 0;
            curve_ptr->equ_pts[curve_ptr->num_equ_pts] = curve_ptr->pts[i];
            // std::cout << curve_ptr->equ_pts[curve_ptr->num_equ_pts].x << "    " << curve_ptr->equ_pts[curve_ptr->num_equ_pts].y << std::endl;
        }
    }
    curve_ptr->num_equ_pts++;
    curve_ptr->equ_pts[curve_ptr->num_equ_pts] = curve_ptr->pts[BEZIER_RESOLUTION - 1];
    // std::cout << curve_ptr->equ_pts[curve_ptr->num_equ_pts].x << "    " << curve_ptr->equ_pts[curve_ptr->num_equ_pts].y << std::endl;
    // std::cout << curve_ptr->num_equ_pts << std::endl;
}

// void add_straight(curve *bezier, double distance)
// {
//     for (int i = 0; i < distance / POINT_DISTANCE - 1; i++)
//     {
//         bezier->points[bezier->number_of_points_2].x = bezier->points[bezier->number_of_points_2 - 1].x + POINT_DISTANCE * cos(bezier->end_target.phi * M_PI / 180);
//         bezier->points[bezier->number_of_points_2].y = bezier->points[bezier->number_of_points_2 - 1].y + POINT_DISTANCE * sin(bezier->end_target.phi * M_PI / 180);
//         bezier->number_of_points_2++;
//     }
//     bezier->points[bezier->number_of_points_2].x = bezier->points[bezier->number_of_points_2 - 1].x + POINT_DISTANCE * cos(bezier->end_target.phi);
//     bezier->points[bezier->number_of_points_2].y = bezier->points[bezier->number_of_points_2 - 1].y + POINT_DISTANCE * sin(bezier->end_target.phi);
//     bezier->number_of_points_2++;
// }

target create_target(double x, double y, double phi)
{
    target temp_target;
    temp_target.x = x;
    temp_target.y = y;
    temp_target.phi = phi;
    return temp_target;
}