#ifndef ROBOT_TURNER_H
#define ROBOT_TURNER_H

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include <geometry_msgs/Twist.h>
#include "ras_utils/controller.h"
#include "ras_utils/ras_utils.h"

#define MAX_ANGLE_DIFF 0.1 // rad

struct RT_PARAMS
{
   bool debug_print;
   double kp_w, kd_w, ki_w;
};

class RobotTurner
{
public:
    RobotTurner() : rotating_(false) {}
    void setParams(const RT_PARAMS &params)
    {
        // Initial values
        w = 0.0;

        // params from launch file
        kp_w = params.kp_w;
        kd_w = params.kd_w;
        ki_w = params.ki_w;

        controller_w = Controller(kp_w,kd_w,ki_w);
    }

    void run(double angle, double &v, double &w )
    {
        double current_angle = angle;
        // compute angular velocity
        controller_w.setData(target_angle, current_angle);
        double diff = target_angle - current_angle;
        diff = RAS_Utils::normalize_angle( diff );
    //    w = controller_w.computeControl();
        w = 1.0 * RAS_Utils::sign(diff);
        v = 0;

        if(fabs(diff) < MAX_ANGLE_DIFF)
        {
            rotating_ = false;
        }
//        ROS_INFO("Current_angle:%.3f  Target:%.3f  Diff:%.3f", current_angle, target_angle, diff);
    }

    bool isRotating() {
        return rotating_;
    }

    void activate(double base_angle, double delta_angle)
    {
        this->base_angle = base_angle;
        this->target_angle = base_angle+ delta_angle;
        rotating_ = true;
    }

private:
    // true if we are currenty rotating
    bool rotating_;

    // rotating angular speed
    double w;

    double target_angle;

    // angle obtained from odometry first time
    double base_angle;
    bool first_time_called;

    // pid controller for angular velocity
    Controller controller_w;
    double kp_w, ki_w, kd_w;
};

#endif
