#include "main.hpp"

void create_curve(curve *curve_ptr, target desired_position, int dir)
{
    target robot_position = get_robot().get_position();
    static coord p0;
    static coord p1;
    static coord p2;
    static coord p3;

    normalize_angle(&desired_position.phi);
    p0.x = robot_position.x;
    p0.y = robot_position.y;

    p1.x = robot_position.x + OFFS_ROBOT * cos((robot_position.phi / 180 + dir) * M_PI);
    p1.y = robot_position.y + OFFS_ROBOT * sin((robot_position.phi / 180 + dir) * M_PI);

    p2.x = desired_position.x - OFFS_DESIRED * cos((desired_position.phi / 180 + dir) * M_PI);
    p2.y = desired_position.y - OFFS_DESIRED * sin((desired_position.phi / 180 + dir) * M_PI);

    p3.x = desired_position.x;
    p3.y = desired_position.y;

    curve_ptr->dis = 0;

    cubic_bezier_pts(curve_ptr, p0, p1, p2, p3);

    curve_ptr->equ_pts = (coord *)malloc((curve_ptr->dis / POINT_DISTANCE + 2) * sizeof(coord));

    equ_coords(curve_ptr);
}

void cubic_bezier_pts(curve *curve_ptr, coord p0, coord p1, coord p2, coord p3)
{
    for (int i = 0; i < BEZIER_RESOLUTION; i++)
    {
        double t = (double)i / BEZIER_RESOLUTION;
        curve_ptr->pts[i].x = pow(1 - t, 3) * p0.x + 3 * t * pow(1 - t, 2) * p1.x + 3 * pow(t, 2) * (1 - t) * p2.x + pow(t, 3) * p3.x;
        curve_ptr->pts[i].y = pow(1 - t, 3) * p0.y + 3 * t * pow(1 - t, 2) * p1.y + 3 * pow(t, 2) * (1 - t) * p2.y + pow(t, 3) * p3.y;
        push_pt(&curve_ptr->pts[i], get_obstacle());
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
        // temp_length += cur_dis;
        temp_length = sqrt((curve_ptr->equ_pts[curve_ptr->num_equ_pts].x - curve_ptr->pts[i].x) * (curve_ptr->equ_pts[curve_ptr->num_equ_pts].x - curve_ptr->pts[i].x) + (curve_ptr->equ_pts[curve_ptr->num_equ_pts].y - curve_ptr->pts[i].y) * (curve_ptr->equ_pts[curve_ptr->num_equ_pts].y - curve_ptr->pts[i].y));
        if (temp_length >= POINT_DISTANCE)
        {
            curve_ptr->num_equ_pts++;
            temp_length = 0;
            curve_ptr->equ_pts[curve_ptr->num_equ_pts] = curve_ptr->pts[i];
            // std::cout << curve_ptr->equ_pts[curve_ptr->num_equ_pts].x << "    " << curve_ptr->equ_pts[curve_ptr->num_equ_pts].y << std::endl;
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

void push_pt(coord *pt, coord center)
{
    coord pushed;
    if (sqrt((pt->x - center.x) * (pt->x - center.x) + (pt->y - center.y) * (pt->y - center.y)) < AVOID_DIS)
    {
        double k = (center.y - pt->y) / (center.x - pt->x);
        double n = center.y - k * center.x;

        double a = 1 + k * k;
        double b = 2 * k * (n - center.y) - 2 * center.x;
        double c = center.x * center.x + (n - center.y) * (n - center.y) - (double)AVOID_DIS * AVOID_DIS;

        pushed.x = plus_quadratic_eq(a, b, c);
        if (sign(center.x - pushed.x) != sign(center.x - pt->x))
            pushed.x = minus_quadratic_eq(a, b, c);
        pushed.y = center.y - (center.y - pt->y) / (center.x - pt->x) * center.x + (center.y - pt->y) / (center.x - pt->x) * pushed.x;
        *pt = pushed;
    }
    return;
}

double plus_quadratic_eq(double a, double b, double c)
{
    return (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
}

double minus_quadratic_eq(double a, double b, double c)
{
    return (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
}