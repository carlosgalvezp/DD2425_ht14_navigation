#include <navigation/robot_turning.h>

Robot_turning::Robot_turning(const RT_PARAMS &params)
{
    // Initial values
    w = 0.0;

    // params from launch file
    kp_w = params.kp_w;
    kd_w = params.kd_w;
    ki_w = params.ki_w;

    controller_w = Controller(kp_w,kd_w,ki_w);
}

void Robot_turning::compute_commands(const geometry_msgs::Pose2D::ConstPtr &msg, double &v, double &w)
{
    double current_angle = msg->theta;
    // compute angular velocity
    controller_w.setData(target_angle, current_angle);
    w = controller_w.computeControl();
    v = 0;
    double diff = current_angle - target_angle;

    if(fabs(diff) < MAX_ANGLE_DIFF)
    {
        rotating_ = false;
    }
    ROS_INFO("Current_angle:%.3f  Target:%.3f  Diff:%.3f", current_angle, target_angle, diff);
}

void Robot_turning::init(double base_angle, double delta_angle)
{
    this->base_angle = base_angle;
    this->target_angle = base_angle+ delta_angle;
}

