#include "main.hpp"

int cubic_bezier(coord bezier[], coord bezier_low_res[], double* distance, coord p0, coord p1, coord p2, coord p3, double point_distance)
{
    double temp_length = 0;
    int j = 0;
    for (int i = 0; i < BEZIER_RESOLUTION; i++)
    {
        double t = (double)i / BEZIER_RESOLUTION;
        bezier[i].x = pow(1 - t, 3) * p0.x + 3 * t * pow(1 - t, 2) * p1.x + 3 * pow(t, 2) * (1 - t) * p2.x + pow(t, 3) * p3.x;
        bezier[i].y = pow(1 - t, 3) * p0.y + 3 * t * pow(1 - t, 2) * p1.y + 3 * pow(t, 2) * (1 - t) * p2.y + pow(t, 3) * p3.y;
        if (i > 0)
        {
            double cur_length = sqrt((bezier[i].x - bezier[i - 1].x) * (bezier[i].x - bezier[i - 1].x) + (bezier[i].y - bezier[i - 1].y) * (bezier[i].y - bezier[i - 1].y));
            *distance += cur_length;

            temp_length += cur_length;
            if (temp_length >= point_distance)
            { 
                temp_length = 0;
                bezier_low_res[j] = bezier[i];
                j++;
            }
        }

    }
    return j;
}
// podeli na podeoke od 100mm i kad predju 50mm onda prebacuj na sledeci