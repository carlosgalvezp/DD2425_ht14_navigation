#ifndef ROBOT_BACKER_H
#define ROBOT_BACKER_H

#include <geometry_msgs/Pose2D.h>
#include <math.h>


class RobotBacker
{
public:
    RobotBacker() : activated(false) {}

    void activate(double wanted_v, double wanted_distance)
    {
        resetStartPos();
        this->wanted_v = wanted_v;
        this->wanted_distance = wanted_distance / 100;
        activated = true;
    }

    void setStartPos(double start_x, double start_y)
    {
        this->start_x = start_x;
        this->start_y = start_y;
    }

    void run(double x, double y, double &v, double &w )
    {
        double traveled_distance = getTraveledDistance(x, y);

        w = 0;
        if(traveled_distance >= wanted_distance) {
            resetStartPos();
            activated = false;
            v = 0;
        } else
        {
            v = wanted_v;
        }
    }

    bool isActive()
    {
        return activated;
    }

    bool isStartPosSet()
    {
        return start_x >= 0;
    }

private:

    bool activated;
    double wanted_v;
    double wanted_distance;
    double start_x, start_y;

    double getTraveledDistance(double x, double y)
    {
        return sqrt(pow(x-start_x, 2) + pow(y-start_y, 2));
    }

    void resetStartPos()
    {
        start_x = -1;
        start_y = -1;
    }

};

#endif // ROBOT_BACKER_H
