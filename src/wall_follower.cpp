#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ras_arduino_msgs/ADConverter.h"

#define PUBLISH_RATE 10 // Hz
#define QUEUE_SIZE 1000

class Wall_follower
{
public:

    Wall_follower(const ros::NodeHandle& n);
    void run();

private:
    // adc data from IR sensors
    int adc_front;
    int adc_back;

    double gain;
    double linear_speed;
    double angular_speed;
    // In which side is wall
    bool wall_is_left;

    ros::NodeHandle n_;
    ros::Publisher twist_pub_;
    ros::Subscriber adc_sub_;
    // Callback func when adc data recieved
    void adcCallback(const ras_arduino_msgs::ADConverter::ConstPtr& msg);
};

int main (int argc, char* argv[])
{
    // ** Init node
    ros::init(argc, argv, "wall_follower");
    ros::NodeHandle n;

    // ** Create wall follower object
    Wall_follower wf(n);

    // ** Run
    wf.run();
}

Wall_follower::Wall_follower(const ros::NodeHandle &n)
    : n_(n)
{
    // initial values
    adc_front = 0;
    adc_back = 0;
    angular_speed = 0;

    // Publisher
    twist_pub_ = n_.advertise<geometry_msgs::Twist>("/motor_controller/twist", QUEUE_SIZE);
    // Subscriber
    adc_sub_ = n_.subscribe("/arduino/adc", 1,  &Wall_follower::adcCallback, this);

    // params from launch file
    n_.getParam("Wall_follower/gain", gain);
    n_.getParam("Wall_follower/linear_speed", linear_speed);
    n_.getParam("Wall_follower/wall_is_left", wall_is_left);
}

void Wall_follower::run()
{   
    // ** Publish data
    ros::Rate loop_rate(PUBLISH_RATE);

    while(ros::ok())
    {
        if(wall_is_left)
            angular_speed = gain * ( adc_back - adc_front );
        else
            angular_speed = gain * ( adc_front - adc_back );

        geometry_msgs::Twist msg;

        msg.linear.x = linear_speed;
        msg.linear.y = 0.0;
        msg.linear.z = 0.0;

        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = angular_speed;

        ROS_INFO("(v,w): %f, %f\n", linear_speed, angular_speed);

        twist_pub_.publish(msg);

        // ** Sleep
        ros::spinOnce();
        loop_rate.sleep();
    }
    std::cout << "Exiting...\n";
}

void Wall_follower::adcCallback(const ras_arduino_msgs::ADConverter::ConstPtr& msg)
{
    if(wall_is_left)
    {
        adc_front = msg->ch1;       //TODO: determine which sensor is which for robot
        adc_back = msg->ch2;
    }
    else
    {
        adc_front = msg->ch7;
        adc_back = msg->ch8;
    }
}
