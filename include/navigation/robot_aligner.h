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
        if(RAS_Utils::sensors::canFollowLeftWall(sd))
        {
            //we can align to the left wall!
            sensor_diff = sd.left_front_ - sd.left_back_;
//            ROS_INFO("left sensor diff: %f", sensor_diff);
        } else if(RAS_Utils::sensors::canFollowRightWall(sd))
        {
            //we can align to the right wall!
            sensor_diff = sd.right_back_ - sd.right_front_;
//            ROS_INFO("right sensor diff: %f", sensor_diff);
        } else
        {
//            ROS_WARN("No wall to align to");
            // No wall we can align to, abort
            w = 0;
            v = 0;
            currently_aligning_ = false;
            return;
        }

        if( fabs(sensor_diff) < MAX_SENSOR_DIFF )
        {
//            ROS_WARN("Good enough aligning. sensor_diff: %f  MAX: %f", sensor_diff, MAX_SENSOR_DIFF);
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
};

#endif // ROBOT_ALIGNER_H
