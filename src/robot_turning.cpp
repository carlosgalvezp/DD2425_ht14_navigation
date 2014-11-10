#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include <geometry_msgs/Twist.h>
#include "ras_utils/controller.h"

#define PUBLISH_RATE 10 // Hz
#define QUEUE_SIZE 1000


class Robot_turning
{
public:

    Robot_turning(const ros::NodeHandle& n);
    void run( double turning_angle );

private:

    ros::NodeHandle n_;

    ros::Subscriber pose2d_sub_;
    ros::Publisher twist_pub_;

    // angle obtained from imu or odometry.. or both?
    double current_angle;

    // rotating angular speed
    double w;

    // angle obtained from odometry first time
    double base_angle;
    bool first_time_called;

    // Callback func when pose data received
    void poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg);

    // pid controller for angular velocity
    Controller controller_w;
    double kp_w, ki_w, kd_w;
};

int main (int argc, char* argv[])
{  
    // ** Init node
    ros::init(argc, argv, "robot_turning");
    ros::NodeHandle n;

    // ** Create object
    Robot_turning turn(n);

    // ** turn 90 degrees (to the right)
    turn.run( -M_PI / 2 );
}

Robot_turning::Robot_turning(const ros::NodeHandle &n)
    : n_(n)
{
    // Initial values
    w = 0.0;
    current_angle = 0;

    // Publisher
    twist_pub_ = n_.advertise<geometry_msgs::Twist>("/motor_controller/twist", QUEUE_SIZE);
    // Subscriber
    pose2d_sub_ = n_.subscribe("/robot/pose2d", QUEUE_SIZE,  &Robot_turning::poseCallback, this);

    // params from launch file
    n_.getParam("Robot_turning/W/KP", kp_w);
    n_.getParam("Robot_turning/W/KD", kd_w);
    n_.getParam("Robot_turning/W/KI", ki_w);

    controller_w = Controller(kp_w,kd_w,ki_w, 10);
}

void Robot_turning::poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    if (first_time_called == true)
    {
        base_angle = msg->theta;
        first_time_called = false;
    }
    else
    {
        current_angle = msg->theta - base_angle;

        //std::cout << "update:" << current_angle << " base: " << base_angle << std::endl;
    }
}

void Robot_turning::run( double turning_angle )
{
    first_time_called = true;

    ros::Rate loop_rate(PUBLISH_RATE);

    while(ros::ok() && ( ( ( current_angle < turning_angle) && ( turning_angle > 0 ) ) || ( ( current_angle > turning_angle) && ( turning_angle < 0 ) ) ) )
    {
        geometry_msgs::Twist msg;

        // compute angular velocity
        controller_w.setData(turning_angle, current_angle);
        w = controller_w.computeControl();
        // turning
        msg.angular.z = -w;

        std::cout << "current_angle: " << current_angle << " turning_angle: " << turning_angle << " w: " << w << std::endl;

        twist_pub_.publish(msg);

        // ** Sleep
        ros::spinOnce();
        loop_rate.sleep();
    }

    // publish 0 for stopping motion
    geometry_msgs::Twist msg;
    msg.angular.z = 0;
    twist_pub_.publish(msg);

    // maybe should publish: done ?

    std::cout << "Exiting...\n";
}

