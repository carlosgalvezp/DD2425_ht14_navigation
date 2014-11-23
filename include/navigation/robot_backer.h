#ifndef ROBOT_BACKER_H
#define ROBOT_BACKER_H

#include <geometry_msgs/Pose2D.h>
#include <math.h>


class Robot_backer
{
public:
    Robot_backer() {}

    void init(double wanted_v, double wanted_distance)
    {
        this->start_x = -1;
        this->start_y = -1;
        this->wanted_v = wanted_v;
        this->wanted_distance = wanted_distance;
        backing = true;
    }

    void set_start_pos(double start_x, double start_y)
    {
        this->start_x = start_x;
        this->start_y = start_y;
    }

    void compute_commands(double x, double y, double &v, double &w )
    {
        double traveled_distance = get_traveled_distance(x, y);

        w = 0;
        if(traveled_distance >= wanted_distance) {
            backing = false;
            v = 0;
        } else
        {
            v = wanted_v;
        }
    }

    bool is_backing()
    {
        return backing;
    }

    bool is_start_pos_set()
    {
        return start_x >= 0;
    }

private:

    bool backing;
    double wanted_v;
    double wanted_distance;
    double start_x, start_y;

    double get_traveled_distance(double x, double y)
    {
        return sqrt(pow(x-start_x, 2) + pow(y-start_y, 2));
    }

};

#endif // ROBOT_BACKER_H
