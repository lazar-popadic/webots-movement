#include "main.hpp"

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