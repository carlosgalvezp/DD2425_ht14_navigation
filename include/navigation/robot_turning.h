#ifndef ROBOT_TURNING_H
#define ROBOT_TURNING_H

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include <geometry_msgs/Twist.h>
#include "ras_utils/controller.h"

#define MAX_ANGLE_DIFF 0.1 // rad

struct RT_PARAMS
{
   bool debug_print;
   double kp_w, kd_w, ki_w;
};


class Robot_turning
{
public:
    Robot_turning();
    Robot_turning(const RT_PARAMS &params);

    void compute_commands( const geometry_msgs::Pose2D::ConstPtr &msg, double &v, double &w );

    bool isRotating() {
        return rotating_;
    }

    void init(double base_angle, double delta_angle);

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
