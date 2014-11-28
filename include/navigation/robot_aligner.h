#ifndef ROBOT_ALIGNER_H
#define ROBOT_ALIGNER_H

#include <ros/ros.h>
#include <ras_utils/ras_sensor_utils.h>

#define MAX_DIST_SIDE_WALL     25       // [cm]

#define MAX_SENSOR_DIFF        1.0      // cm for alignment


class RobotAligner {
public:
    RobotAligner() : currently_aligning_(false) {}

    void activate() {
        currently_aligning_ = true;
    }

    void run(double &v, double &w, RAS_Utils::sensors::SensorDistances sd)
    {
        double sensor_diff;
        if(canFollowWall(sd.left_front_, sd.left_back_))
        {
            //we can align to the left wall!
            double sensor_diff = sd.left_front_ - sd.left_back_;
            ROS_INFO("left sensor diff: %f", sensor_diff);
        } else if(canFollowWall(sd.right_front_, sd.right_back_))
        {
            //we can align to the right wall!
            double sensor_diff = sd.right_front_ - sd.right_back_;
            ROS_INFO("right sensor diff: %f", sensor_diff);
        } else
        {
            ROS_WARN("No wall to align, abort");
            // No wall we can align to, abort
            w = 0;
            v = 0;
            currently_aligning_ = false;
            return;
        }

        if( fabs(sensor_diff) < MAX_SENSOR_DIFF )
        {
            ROS_WARN("Good enough aligning. sensor_diff: %f  MAX: %f", sensor_diff, MAX_SENSOR_DIFF);
            currently_aligning_ = false;
            w = 0;
            v = 0;
            return;
        }

        w = 0.5 * RAS_Utils::sign(sensor_diff);
        v = 0;
    }

    bool isActive()
    {
        return currently_aligning_;
    }

private:
    bool currently_aligning_;

    bool canFollowWall(double d_front, double d_back)
    {
        return d_front < MAX_DIST_SIDE_WALL && d_back < MAX_DIST_SIDE_WALL;
    }
};

#endif // ROBOT_ALIGNER_H
