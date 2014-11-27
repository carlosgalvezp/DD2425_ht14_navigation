#ifndef ROBOT_ALIGNER_H
#define ROBOT_ALIGNER_H

#define MAX_DIST_SIDE_WALL     25       // [cm]

#define MAX_SENSOR_DIFF        1.0      // cm for alignment

class RobotAligner {
public:
    RobotAligner() : currently_aligning_(false) {}

    void run(double &v, double &w, double d_right_front, double d_left_front, double d_right_back, double d_left_back)
    {
        double sensor_diff;
        if(canFollowWall(d_left_front, d_left_back))
        {
            //we can align to the left wall!
            double sensor_diff = d_left_front - d_left_back;
            ROS_INFO("left sensor diff: %f", sensor_diff);
        } else if(canFollowWall(d_right_front, d_right_back))
        {
            //we can align to the right wall!
            double sensor_diff = d_right_back - d_right_front;
            ROS_INFO("right sensor diff: %f", sensor_diff);
        } else {
            // No wall we can align to, abort
            w = 0;
            v = 0;
            return;
        }

        if( fabs(sensor_diff) < MAX_SENSOR_DIFF )
        {
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
