#ifndef ROBOT_ANGLE_FOLLOWER_H
#define ROBOT_ANGLE_FOLLOWER_H


#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include <geometry_msgs/Twist.h>
#include "ras_utils/controller.h"
#include "ras_utils/ras_utils.h"
#include <math.h>

struct RAF_PARAMS
{
   double wanted_v;
   double kp_w, kd_w, ki_w;
};

class RobotAngleFollower
{
public:
    RobotAngleFollower() {}
    void setParams(const RAF_PARAMS &params)
    {

        // params from launch file
        kp_w = params.kp_w;
        kd_w = params.kd_w;
        ki_w = params.ki_w;

        wanted_v = params.wanted_v;

        controller_w = Controller(kp_w,kd_w,ki_w);
    }

    void run(double &v, double &w, double current_angle, double wanted_angle )
    {
        // compute angular velocity
        current_angle = RAS_Utils::normalize_angle( current_angle );
        wanted_angle = RAS_Utils::normalize_angle( wanted_angle );
        controller_w.setData(wanted_angle, current_angle);
        double diff = wanted_angle - current_angle;
        diff = RAS_Utils::normalize_angle( diff );
        w = controller_w.computeControl();


        double v_retract =  fmin((wanted_v / (M_PI / 3.0)) * fabs(diff), wanted_v);

        v = wanted_v - v_retract;
    }

private:

    double wanted_v;
    // pid controller for angular velocity
    Controller controller_w;
    double kp_w, ki_w, kd_w;
};


#endif // ROBOT_ANGLE_FOLLOWER_H
