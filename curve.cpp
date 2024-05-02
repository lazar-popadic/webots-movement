#include "main.hpp"

void create_full_curve(new_curve *curve_ptr, target desired_position)
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

void create_curve(curve *bezier, target robot_position, target desired_position)
{
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

    bezier->distance = 0;

    cubic_bezier_simple(bezier, p0, p1, p2, p3);
    // cubic_bezier_curve(bezier, p0, p1, p2, p3);

    bezier->end_target.x = desired_position.x;
    bezier->end_target.y = desired_position.y;
    bezier->end_target.phi = desired_position.phi;
    bezier->number_of_points_2 = BEZIER_RESOLUTION;
}

void cubic_bezier_curve(curve *bezier, coord p0, coord p1, coord p2, coord p3)
{
    double temp_length = 0;
    for (int i = 0; i < BEZIER_RESOLUTION; i++)
    {
        double t = (double)i / BEZIER_RESOLUTION;
        bezier->points[i].x = pow(1 - t, 3) * p0.x + 3 * t * pow(1 - t, 2) * p1.x + 3 * pow(t, 2) * (1 - t) * p2.x + pow(t, 3) * p3.x;
        bezier->points[i].y = pow(1 - t, 3) * p0.y + 3 * t * pow(1 - t, 2) * p1.y + 3 * pow(t, 2) * (1 - t) * p2.y + pow(t, 3) * p3.y;
        if (i > 0)
        {
            double cur_length = sqrt((bezier->points[i].x - bezier->points[i - 1].x) * (bezier->points[i].x - bezier->points[i - 1].x) + (bezier->points[i].y - bezier->points[i - 1].y) * (bezier->points[i].y - bezier->points[i - 1].y));
            bezier->distance += cur_length;

            temp_length += cur_length;
            if (temp_length >= POINT_DISTANCE)
            {
                temp_length = 0;
                // bezier->points_low_res[bezier->number_of_points] = bezier->points[i];
                // bezier->number_of_points++;
            }
        }
    }
    // std::cout << bezier->number_of_points << std::endl;
}

void cubic_bezier_simple(curve *bezier, coord p0, coord p1, coord p2, coord p3)
{
    for (int i = 0; i < BEZIER_RESOLUTION; i++)
    {
        double t = (double)i / BEZIER_RESOLUTION;
        bezier->points[i].x = pow(1 - t, 3) * p0.x + 3 * t * pow(1 - t, 2) * p1.x + 3 * pow(t, 2) * (1 - t) * p2.x + pow(t, 3) * p3.x;
        bezier->points[i].y = pow(1 - t, 3) * p0.y + 3 * t * pow(1 - t, 2) * p1.y + 3 * pow(t, 2) * (1 - t) * p2.y + pow(t, 3) * p3.y;
        if (i > 0)
        {
            bezier->distance += sqrt((bezier->points[i].x - bezier->points[i - 1].x) * (bezier->points[i].x - bezier->points[i - 1].x) + (bezier->points[i].y - bezier->points[i - 1].y) * (bezier->points[i].y - bezier->points[i - 1].y));
        }
    }
}

void cubic_bezier_pts(new_curve *curve_ptr, coord p0, coord p1, coord p2, coord p3)
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

void equ_coords(new_curve *curve_ptr)
{
    double temp_length = 0;
    double cur_dis = 0;
    curve_ptr->num_equ_pts = 0;
    curve_ptr->equ_pts[0] = curve_ptr->pts[0];
    for (int i = 0; i < BEZIER_RESOLUTION; i++)
    {
        cur_dis = sqrt((curve_ptr->pts[i].x - curve_ptr->pts[i - 1].x) * (curve_ptr->pts[i].x - curve_ptr->pts[i - 1].x) + (curve_ptr->pts[i].y - curve_ptr->pts[i - 1].y) * (curve_ptr->pts[i].y - curve_ptr->pts[i - 1].y));
        curve_ptr->dis += cur_dis;
        temp_length += cur_dis;
        if (temp_length >= POINT_DISTANCE)
        {
            curve_ptr->num_equ_pts++;
            temp_length = 0;
            curve_ptr->equ_pts[curve_ptr->num_equ_pts] = curve_ptr->pts[i];
            std::cout << curve_ptr->equ_pts[curve_ptr->num_equ_pts].x << "    " << curve_ptr->equ_pts[curve_ptr->num_equ_pts].y << std::endl;
        }
    }
    curve_ptr->num_equ_pts++;
    curve_ptr->equ_pts[curve_ptr->num_equ_pts] = curve_ptr->pts[BEZIER_RESOLUTION - 1];
    std::cout << curve_ptr->equ_pts[curve_ptr->num_equ_pts].x << "    " << curve_ptr->equ_pts[curve_ptr->num_equ_pts].y << std::endl;
    std::cout << curve_ptr->num_equ_pts << std::endl;
}

void equidistant_coords(coord *new_curve, int *number_of_points, double *distance, curve bezier)
{
    double temp_length = 0;
    double cur_dis = 0;
    *number_of_points = 0;
    new_curve[0] = bezier.points[0];
    for (int i = 1; i < bezier.number_of_points_2; i++)
    {
        cur_dis = sqrt((bezier.points[i].x - bezier.points[i - 1].x) * (bezier.points[i].x - bezier.points[i - 1].x) + (bezier.points[i].y - bezier.points[i - 1].y) * (bezier.points[i].y - bezier.points[i - 1].y));
        *distance += cur_dis;
        temp_length += cur_dis;
        if (temp_length >= POINT_DISTANCE)
        {
            (*number_of_points)++;
            temp_length = 0;
            new_curve[*number_of_points] = bezier.points[i];
            std::cout << new_curve[*number_of_points].x << "    " << new_curve[*number_of_points].y << std::endl;
        }
    }
    (*number_of_points)++;
    new_curve[*number_of_points] = bezier.points[bezier.number_of_points_2 - 1];
    std::cout << new_curve[*number_of_points].x << "    " << new_curve[*number_of_points].y << std::endl;
    std::cout << *number_of_points << std::endl;
}

void add_to_curve_2(curve *bezier, curve added_curve)
{
    for (int i = 0; i < BEZIER_RESOLUTION; i++)
    {
        bezier->points[bezier->number_of_points_2 + i] = added_curve.points[i];
    }
    bezier->distance += added_curve.distance;
    bezier->number_of_points_2 += BEZIER_RESOLUTION;
    bezier->end_target = added_curve.end_target;
}

void add_straight(curve *bezier, double distance)
{
    for (int i = 0; i < distance / POINT_DISTANCE - 1; i++)
    {
        bezier->points[bezier->number_of_points_2].x = bezier->points[bezier->number_of_points_2 - 1].x + POINT_DISTANCE * cos(bezier->end_target.phi * M_PI / 180);
        bezier->points[bezier->number_of_points_2].y = bezier->points[bezier->number_of_points_2 - 1].y + POINT_DISTANCE * sin(bezier->end_target.phi * M_PI / 180);
        bezier->number_of_points_2++;
    }
    bezier->points[bezier->number_of_points_2].x = bezier->points[bezier->number_of_points_2 - 1].x + POINT_DISTANCE * cos(bezier->end_target.phi);
    bezier->points[bezier->number_of_points_2].y = bezier->points[bezier->number_of_points_2 - 1].y + POINT_DISTANCE * sin(bezier->end_target.phi);
    bezier->number_of_points_2++;
}

target create_target(double x, double y, double phi)
{
    target temp_target;
    temp_target.x = x;
    temp_target.y = y;
    temp_target.phi = phi;
    return temp_target;
}