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
        double diff = wanted_angle - current_angle;
//        ROS_ERROR("DIFF: %.3f Current_angle: %.3f sWanted: %.3f", diff, current_angle, wanted_angle);

        diff = RAS_Utils::normalize_angle( diff );
//        ROS_ERROR("DIFF: %.3f", diff);



        if(diff > M_PI_2)
        {
            diff = M_PI_2;
        }
        if(diff < -M_PI_2)
        {
            diff = -M_PI_2;
        }
        
      //  ROS_INFO("DIFF: %.3f", diff);


        controller_w.setData(0, -diff);
        w = controller_w.computeControl();

//        ROS_ERROR("Diff: %.3f, Currrent_angle: %.3f, wanted_angle: %.3f", diff, current_angle, wanted_angle);


        double v_retract =  fmin((wanted_v / (M_PI / 5.0)) * fabs(diff), wanted_v);

        v = wanted_v - v_retract;
    }

private:

    double wanted_v;
    // pid controller for angular velocity
    Controller controller_w;
    double kp_w, ki_w, kd_w;
};


#endif // ROBOT_ANGLE_FOLLOWER_H
