#include "main.hpp"

void create_curve_multi(curve *bezier, target robot_position, target targets[], int number_of_targets)
{
    curve temp_curve[number_of_targets];
    create_curve(bezier, robot_position, targets[0]);
    for (int i = 1; i < number_of_targets; i++)
    {
        create_curve(&temp_curve[i], targets[i - 1], targets[i]);
        add_to_curve_2(bezier, temp_curve[i]);
        // curve *temp_curve = (curve*)malloc(sizeof(target));
        // free(&temp_curve);
    }
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

    cubic_bezier_simple(bezier, p0, p1, p2, p3);

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
                bezier->points_low_res[bezier->number_of_points] = bezier->points[i];
                bezier->number_of_points++;
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

void equidistant_coords(coord* new_curve, int* number_of_points, double* distance, curve bezier)
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

void add_straight (curve *bezier, double distance)
{
    for (int i = 0; i < distance / POINT_DISTANCE - 1; i++)
    {
        bezier->points[bezier->number_of_points_2].x = bezier->points[bezier->number_of_points_2 - 1].x + POINT_DISTANCE * cos(bezier->end_target.phi * M_PI / 180);
        bezier->points[bezier->number_of_points_2].y = bezier->points[bezier->number_of_points_2 - 1].y + POINT_DISTANCE * sin(bezier->end_target.phi * M_PI / 180);
        bezier->number_of_points_2 ++;
    }
    bezier->points[bezier->number_of_points_2].x = bezier->points[bezier->number_of_points_2 - 1].x + POINT_DISTANCE * cos(bezier->end_target.phi);
    bezier->points[bezier->number_of_points_2].y = bezier->points[bezier->number_of_points_2 - 1].y + POINT_DISTANCE * sin(bezier->end_target.phi);
    bezier->number_of_points_2 ++;
}

target create_target(double x, double y, double phi)
{
    target temp_target;
    temp_target.x = x;
    temp_target.y = y;
    temp_target.phi = phi;
    return temp_target;
}