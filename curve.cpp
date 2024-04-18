#include "main.hpp"

static coord p0;
static coord p1;
static coord p2;
static coord p3;
static coord p4;

void create_curve(curve *bezier, target robot_position, double desired_x, double desired_y, double desired_phi)
{
    normalize_angle(&desired_phi);
    p0.x = robot_position.x;
    p0.y = robot_position.y;

    p4.x = desired_x;
    p4.y = desired_y;

    p1.x = robot_position.x + OFFS_ROBOT * cos(robot_position.phi / 180 * M_PI);
    p1.y = robot_position.y + OFFS_ROBOT * sin(robot_position.phi / 180 * M_PI);

    p2.x = desired_x - (OFFS_DESIRED + OFFS_DESIRED_TRAN) * cos(desired_phi / 180 * M_PI);
    p2.y = desired_y - (OFFS_DESIRED + OFFS_DESIRED_TRAN) * sin(desired_phi / 180 * M_PI);

    p3.x = desired_x - OFFS_DESIRED_TRAN * cos(desired_phi / 180 * M_PI);
    p3.y = desired_y - OFFS_DESIRED_TRAN * sin(desired_phi / 180 * M_PI);

    cubic_bezier_curve(bezier, p0, p1, p2, p3);

    bezier->end_target.x = desired_x;
    bezier->end_target.y = desired_y;
    bezier->end_target.phi = desired_phi;
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
                bezier->points_low_res[bezier->number_of_points] = bezier->points[i];
                bezier->number_of_points++;
            }
        }
    }
}

void add_to_curve(curve *bezier, curve added_curve)
{
    for (int i = 0; i < added_curve.number_of_points; i++)
    {
        bezier->points_low_res[bezier->number_of_points + i] = added_curve.points_low_res[i];
    }
    bezier->distance += added_curve.distance;
    bezier->number_of_points += added_curve.number_of_points;
    bezier->end_target = added_curve.end_target;
}

target create_target(double x, double y, double phi)
{
    target temp_target;
    temp_target.x = x;
    temp_target.y = y;
    temp_target.phi = phi;
    return temp_target;
}