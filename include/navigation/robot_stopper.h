#ifndef ROBOT_STOPPER_H
#define ROBOT_STOPPER_H

#include <ros/ros.h>

#define DEFAULT_STOP_TIME_SECONDS 0.2

class RobotStopper
{
public:
    RobotStopper() : stopped_(false) {}

    /**
      Sets the v and w to zero to indicate it should stop immideatly
      Remember that the robot won't continue beeing stopped unless sleepIfStopped() is called
    */
    void activate(double &v, double &w, double stop_time_in_sec = DEFAULT_STOP_TIME_SECONDS)
    {
        stop_time_ = stop_time_in_sec;
        v = 0;
        w = 0;
        stopped_ = true;
    }

    void sleepIfStopped()
    {
        if(stopped_) {
//            ROS_WARN("Sleeping");
            ros::Duration(stop_time_).sleep();
            stopped_ = false;
        }
    }

private:
    bool stopped_;
    double stop_time_;
};

#endif // ROBOT_STOPPER_H
